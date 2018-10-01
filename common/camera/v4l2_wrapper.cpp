/*
 * Copyright 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//#define LOG_NDEBUG 0
#define LOG_TAG "V4L2Wrapper"

#include "v4l2_wrapper.h"
#include "v4l2_generic_wrapper.h"

#include <algorithm>
#include <array>
#include <limits>
#include <mutex>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <android-base/unique_fd.h>

#include "arc/cached_frame.h"

namespace v4l2_camera_hal {

using arc::SupportedFormat;
using arc::SupportedFormats;
using default_camera_hal::CaptureRequest;

const int32_t kStandardSizes[][2] = {
  {4096, 2160}, // 4KDCI (for USB camera)
  {3840, 2160}, // 4KUHD (for USB camera)
  {3280, 2464}, // 8MP
  {2560, 1440}, // QHD
  {1920, 1080}, // HD1080
  {1640, 1232}, // 2MP
  {1280,  720}, // HD
  {1024,  768}, // XGA
  { 640,  480}, // VGA
  { 320,  240}, // QVGA
  { 176,  144}  // QCIF
};

V4L2Wrapper* V4L2Wrapper::NewV4L2Wrapper(const std::string device_path) {
  return new V4L2GenericWrapper(device_path);
}

V4L2Wrapper::V4L2Wrapper(const std::string device_path)
    : device_path_(std::move(device_path)), connection_count_(0) {}

V4L2Wrapper::~V4L2Wrapper() {}

int V4L2Wrapper::Connect() {
  HAL_LOG_ENTER();
  std::lock_guard<std::mutex> lock(connection_lock_);

  if (connected()) {
    HAL_LOGV("Camera device %s is already connected.", device_path_.c_str());
    ++connection_count_;
    return 0;
  }

  // Open in nonblocking mode (DQBUF may return EAGAIN).
  int fd = TEMP_FAILURE_RETRY(open(device_path_.c_str(), O_RDWR | O_NONBLOCK));
  if (fd < 0) {
    HAL_LOGE("failed to open %s (%s)", device_path_.c_str(), strerror(errno));
    return -ENODEV;
  }
  device_fd_.reset(fd);
  ++connection_count_;

  // Check if this connection has the extended control query capability.
  v4l2_query_ext_ctrl query;
  query.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
  extended_query_supported_ = (IoctlLocked(VIDIOC_QUERY_EXT_CTRL, &query) == 0);

  // TODO(b/29185945): confirm this is a supported device.
  // This is checked by the HAL, but the device at device_path_ may
  // not be the same one that was there when the HAL was loaded.
  // (Alternatively, better hotplugging support may make this unecessary
  // by disabling cameras that get disconnected and checking newly connected
  // cameras, so Connect() is never called on an unsupported camera)

  supported_formats_ = GetSupportedFormats();
  qualified_formats_ = StreamFormat::GetQualifiedFormats(supported_formats_);

  return 0;
}

void V4L2Wrapper::Disconnect() {
  HAL_LOG_ENTER();
  std::lock_guard<std::mutex> lock(connection_lock_);

  if (connection_count_ == 0) {
    // Not connected.
    HAL_LOGE("Camera device %s is not connected, cannot disconnect.",
             device_path_.c_str());
    return;
  }

  --connection_count_;
  if (connection_count_ > 0) {
    HAL_LOGV("Disconnected from camera device %s. %d connections remain.",
             device_path_.c_str(), connection_count_);
    return;
  }

  device_fd_.reset(-1);  // Includes close().
  format_.reset();
}

int V4L2Wrapper::StreamOn() {
  if (!format_) {
    HAL_LOGE("Stream format must be set before turning on stream.");
    return -EINVAL;
  }

  int32_t type = format_->type();
  if (IoctlLocked(VIDIOC_STREAMON, &type) < 0) {
    HAL_LOGE("STREAMON fails (%d): %s", errno, strerror(errno));
    return -ENODEV;
  }

  HAL_LOGV("Stream turned on.");
  return 0;
}

int V4L2Wrapper::StreamOff() {
  if (!format_) {
    // Can't have turned on the stream without format being set,
    // so nothing to turn off here.
    return 0;
  }

  int32_t type = format_->type();
  int res = IoctlLocked(VIDIOC_STREAMOFF, &type);
  // Calling STREAMOFF releases all queued buffers back to the user.
  // No buffers in flight.
  if (res < 0) {
    HAL_LOGE("STREAMOFF fails: %s", strerror(errno));
    return -ENODEV;
  }
  HAL_LOGV("Stream turned off.");
  return 0;
}

int V4L2Wrapper::QueryControl(uint32_t control_id,
                              v4l2_query_ext_ctrl* result) {
  int res;

  memset(result, 0, sizeof(*result));

  if (extended_query_supported_) {
    result->id = control_id;
    res = IoctlLocked(VIDIOC_QUERY_EXT_CTRL, result);
    // Assuming the operation was supported (not ENOTTY), no more to do.
    if (errno != ENOTTY) {
      if (res) {
        HAL_LOGE("QUERY_EXT_CTRL fails: %s", strerror(errno));
        return -ENODEV;
      }
      return 0;
    }
  }

  // Extended control querying not supported, fall back to basic control query.
  v4l2_queryctrl query;
  query.id = control_id;
  if (IoctlLocked(VIDIOC_QUERYCTRL, &query)) {
    HAL_LOGE("QUERYCTRL fails: %s", strerror(errno));
    return -ENODEV;
  }

  // Convert the basic result to the extended result.
  result->id = query.id;
  result->type = query.type;
  memcpy(result->name, query.name, sizeof(query.name));
  result->minimum = query.minimum;
  if (query.type == V4L2_CTRL_TYPE_BITMASK) {
    // According to the V4L2 documentation, when type is BITMASK,
    // max and default should be interpreted as __u32. Practically,
    // this means the conversion from 32 bit to 64 will pad with 0s not 1s.
    result->maximum = static_cast<uint32_t>(query.maximum);
    result->default_value = static_cast<uint32_t>(query.default_value);
  } else {
    result->maximum = query.maximum;
    result->default_value = query.default_value;
  }
  result->step = static_cast<uint32_t>(query.step);
  result->flags = query.flags;
  result->elems = 1;
  switch (result->type) {
    case V4L2_CTRL_TYPE_INTEGER64:
      result->elem_size = sizeof(int64_t);
      break;
    case V4L2_CTRL_TYPE_STRING:
      result->elem_size = result->maximum + 1;
      break;
    default:
      result->elem_size = sizeof(int32_t);
      break;
  }

  return 0;
}

int V4L2Wrapper::GetControl(uint32_t control_id, int32_t* value) {
  // For extended controls (any control class other than "user"),
  // G_EXT_CTRL must be used instead of G_CTRL.
  if (V4L2_CTRL_ID2CLASS(control_id) != V4L2_CTRL_CLASS_USER) {
    v4l2_ext_control control;
    v4l2_ext_controls controls;
    memset(&control, 0, sizeof(control));
    memset(&controls, 0, sizeof(controls));

    control.id = control_id;
    controls.ctrl_class = V4L2_CTRL_ID2CLASS(control_id);
    controls.count = 1;
    controls.controls = &control;

    if (IoctlLocked(VIDIOC_G_EXT_CTRLS, &controls) < 0) {
      HAL_LOGE("G_EXT_CTRLS fails: %s", strerror(errno));
      return -ENODEV;
    }
    *value = control.value;
  } else {
    v4l2_control control{control_id, 0};
    if (IoctlLocked(VIDIOC_G_CTRL, &control) < 0) {
      HAL_LOGE("G_CTRL fails: %s", strerror(errno));
      return -ENODEV;
    }
    *value = control.value;
  }
  return 0;
}

int V4L2Wrapper::SetControl(uint32_t control_id,
                            int32_t desired,
                            int32_t* result) {
  int32_t result_value = 0;

  // TODO(b/29334616): When async, this may need to check if the stream
  // is on, and if so, lock it off while setting format. Need to look
  // into if V4L2 supports adjusting controls while the stream is on.

  // For extended controls (any control class other than "user"),
  // S_EXT_CTRL must be used instead of S_CTRL.
  if (V4L2_CTRL_ID2CLASS(control_id) != V4L2_CTRL_CLASS_USER) {
    v4l2_ext_control control;
    v4l2_ext_controls controls;
    memset(&control, 0, sizeof(control));
    memset(&controls, 0, sizeof(controls));

    control.id = control_id;
    control.value = desired;
    controls.ctrl_class = V4L2_CTRL_ID2CLASS(control_id);
    controls.count = 1;
    controls.controls = &control;

    if (IoctlLocked(VIDIOC_S_EXT_CTRLS, &controls) < 0) {
      HAL_LOGE("S_EXT_CTRLS fails: %s", strerror(errno));
      return -ENODEV;
    }
    result_value = control.value;
  } else {
    v4l2_control control{control_id, desired};
    if (IoctlLocked(VIDIOC_S_CTRL, &control) < 0) {
      HAL_LOGE("S_CTRL fails: %s", strerror(errno));
      return -ENODEV;
    }
    result_value = control.value;
  }

  // If the caller wants to know the result, pass it back.
  if (result != nullptr) {
    *result = result_value;
  }
  return 0;
}

const SupportedFormats V4L2Wrapper::GetSupportedFormats() {
  SupportedFormats formats;
  std::set<uint32_t> pixel_formats;
  int res = GetFormats(&pixel_formats);
  if (res) {
    HAL_LOGE("Failed to get device formats.");
    return formats;
  }

  arc::SupportedFormat supported_format;
  std::set<std::array<int32_t, 2>> frame_sizes;

  for (auto pixel_format : pixel_formats) {
    supported_format.fourcc = pixel_format;

    frame_sizes.clear();
    res = GetFormatFrameSizes(pixel_format, &frame_sizes);
    if (res) {
      HAL_LOGE("Failed to get frame sizes for format: 0x%x", pixel_format);
      continue;
    }
    for (auto frame_size : frame_sizes) {
      supported_format.width = frame_size[0];
      supported_format.height = frame_size[1];
      formats.push_back(supported_format);
    }
  }
  return formats;
}

int V4L2Wrapper::GetFormats(std::set<uint32_t>* v4l2_formats) {
  HAL_LOG_ENTER();

  v4l2_fmtdesc format_query;
  memset(&format_query, 0, sizeof(format_query));
  // TODO(b/30000211): multiplanar support.
  format_query.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  while (IoctlLocked(VIDIOC_ENUM_FMT, &format_query) >= 0) {
    v4l2_formats->insert(format_query.pixelformat);
    ++format_query.index;
  }

  if (errno != EINVAL) {
    HAL_LOGE(
        "ENUM_FMT fails at index %d: %s", format_query.index, strerror(errno));
    return -ENODEV;
  }
  return 0;
}

int V4L2Wrapper::GetQualifiedFormats(std::vector<uint32_t>* v4l2_formats) {
  HAL_LOG_ENTER();
  if (!connected()) {
    HAL_LOGE(
        "Device is not connected, qualified formats may not have been set.");
    return -EINVAL;
  }
  v4l2_formats->clear();
  std::set<uint32_t> unique_fourccs;
  for (auto& format : qualified_formats_) {
    unique_fourccs.insert(format.fourcc);
  }
  v4l2_formats->assign(unique_fourccs.begin(), unique_fourccs.end());
  return 0;
}

int V4L2Wrapper::GetFormatFrameSizes(uint32_t v4l2_format,
                                     std::set<std::array<int32_t, 2>>* sizes) {
  v4l2_frmsizeenum size_query;
  memset(&size_query, 0, sizeof(size_query));
  size_query.pixel_format = v4l2_format;
  if (IoctlLocked(VIDIOC_ENUM_FRAMESIZES, &size_query) < 0) {
    HAL_LOGE("ENUM_FRAMESIZES failed: %s", strerror(errno));
    return -ENODEV;
  }
  if (size_query.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
    // Discrete: enumerate all sizes using VIDIOC_ENUM_FRAMESIZES.
    // Assuming that a driver with discrete frame sizes has a reasonable number
    // of them.
    do {
      sizes->insert({{{static_cast<int32_t>(size_query.discrete.width),
                       static_cast<int32_t>(size_query.discrete.height)}}});
      ++size_query.index;
    } while (IoctlLocked(VIDIOC_ENUM_FRAMESIZES, &size_query) >= 0);
    if (errno != EINVAL) {
      HAL_LOGE("ENUM_FRAMESIZES fails at index %d: %s",
               size_query.index,
               strerror(errno));
      return -ENODEV;
    }
  } else {
    // Continuous/Step-wise: based on the stepwise struct returned by the query.
    // Fully listing all possible sizes, with large enough range/small enough
    // step size, may produce far too many potential sizes. Instead, find the
    // closest to a set of standard sizes.
    for (const auto size : kStandardSizes) {
      // Find the closest size, rounding up.
      uint32_t desired_width = size[0];
      uint32_t desired_height = size[1];
      if (desired_width < size_query.stepwise.min_width ||
          desired_height < size_query.stepwise.min_height) {
        HAL_LOGV("Standard size %u x %u is too small for format %d",
                 desired_width,
                 desired_height,
                 v4l2_format);
        continue;
      } else if (desired_width > size_query.stepwise.max_width ||
                 desired_height > size_query.stepwise.max_height) {
        HAL_LOGV("Standard size %u x %u is too big for format %d",
                 desired_width,
                 desired_height,
                 v4l2_format);
        continue;
      }

      // Round up.
      uint32_t width_steps = (desired_width - size_query.stepwise.min_width +
                              size_query.stepwise.step_width - 1) /
                             size_query.stepwise.step_width;
      uint32_t height_steps = (desired_height - size_query.stepwise.min_height +
                               size_query.stepwise.step_height - 1) /
                              size_query.stepwise.step_height;
      sizes->insert(
          {{{static_cast<int32_t>(size_query.stepwise.min_width +
                                  width_steps * size_query.stepwise.step_width),
             static_cast<int32_t>(size_query.stepwise.min_height +
                                  height_steps *
                                      size_query.stepwise.step_height)}}});
    }
  }
  return 0;
}

// Converts a v4l2_fract with units of seconds to an int64_t with units of ns.
inline int64_t FractToNs(const v4l2_fract& fract) {
  return (1000000000LL * fract.numerator) / fract.denominator;
}

int V4L2Wrapper::GetFormatFrameDurationRange(
    uint32_t v4l2_format,
    const std::array<int32_t, 2>& size,
    std::array<int64_t, 2>* duration_range) {
  // Potentially called so many times logging entry is a bad idea.

  v4l2_frmivalenum duration_query;
  memset(&duration_query, 0, sizeof(duration_query));
  duration_query.pixel_format = v4l2_format;
  duration_query.width = size[0];
  duration_query.height = size[1];
  if (IoctlLocked(VIDIOC_ENUM_FRAMEINTERVALS, &duration_query) < 0) {
    HAL_LOGE("ENUM_FRAMEINTERVALS failed: %s", strerror(errno));
    return -ENODEV;
  }

  int64_t min = std::numeric_limits<int64_t>::max();
  int64_t max = std::numeric_limits<int64_t>::min();
  if (duration_query.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
    // Discrete: enumerate all durations using VIDIOC_ENUM_FRAMEINTERVALS.
    do {
      min = std::min(min, FractToNs(duration_query.discrete));
      max = std::max(max, FractToNs(duration_query.discrete));
      ++duration_query.index;
    } while (IoctlLocked(VIDIOC_ENUM_FRAMEINTERVALS, &duration_query) >= 0);
    if (errno != EINVAL) {
      HAL_LOGE("ENUM_FRAMEINTERVALS fails at index %d: %s",
               duration_query.index,
               strerror(errno));
      return -ENODEV;
    }
  } else {
    // Continuous/Step-wise: simply convert the given min and max.
    min = FractToNs(duration_query.stepwise.min);
    max = FractToNs(duration_query.stepwise.max);
  }
  (*duration_range)[0] = min;
  (*duration_range)[1] = max;
  return 0;
}

}  // namespace v4l2_camera_hal
