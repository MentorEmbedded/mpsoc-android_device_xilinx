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

#define LOG_NDEBUG 0
#define LOG_TAG "V4L2XilinxHdmiWrapper"

#include "v4l2_xilinx_hdmi_wrapper.h"

#include <algorithm>
#include <array>
#include <limits>
#include <mutex>
#include <vector>

#include <dirent.h>
#include <inttypes.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <android-base/unique_fd.h>
#include <cutils/properties.h>

#include "arc/cached_frame.h"

namespace v4l2_camera_hal {

using arc::V4L2MmapedFrameBuffer;
using arc::SupportedFormat;
using arc::SupportedFormats;
using default_camera_hal::CaptureRequest;

V4L2XilinxHdmiWrapper::V4L2XilinxHdmiWrapper(const std::string device_path) :
  V4L2Wrapper(device_path) {
}

V4L2XilinxHdmiWrapper::~V4L2XilinxHdmiWrapper() {}

int V4L2XilinxHdmiWrapper::FindSubdevices() {
  HAL_LOG_ENTER();
  std::lock_guard<std::mutex> lock(subdevs_lock_);
  bool found_scaler = false;
  bool found_rx = false;

  // name of main hdmi capture device in device tree
  char hdmi_device_name[PROPERTY_VALUE_MAX];

  // name of scaler subdevice
  char scaler_device_name[PROPERTY_VALUE_MAX];

  // name of hdmi_rx subdevice
  char rx_device_name[PROPERTY_VALUE_MAX];

  property_get("xlnx.v4l2.hdmi.main_dts_name", hdmi_device_name, "vcap_hdmi_1");
  property_get("xlnx.v4l2.hdmi.scaler_dts_name", scaler_device_name, "a0080000.scaler");
  property_get("xlnx.v4l2.hdmi.rx_dts_name", rx_device_name, "a0000000.hdmi_rx_ss");

  std::string sysfs_path = "/sys/devices/platform/amba/amba:";
  sysfs_path += hdmi_device_name;
  sysfs_path += "/video4linux/";

  DIR* dir = opendir(sysfs_path.c_str());
  if (dir == NULL) {
    HAL_LOGE("Failed to open %s", sysfs_path.c_str());
    return -ENODEV;
  }
  dirent* ent;
  while ((ent = readdir(dir))) {
    std::string desired = "v4l-subdev";
    size_t len = desired.size();
    if (strncmp(desired.c_str(), ent->d_name, len) == 0) {
      if (strlen(ent->d_name) > len && isdigit(ent->d_name[len])) {
        // ent is a numbered v4l-subdev node.
        // open file with a subdevice name
        std::string sbd_name_file = sysfs_path;
        sbd_name_file += ent->d_name;
        sbd_name_file += "/name";
        int fd = open(sbd_name_file.c_str(), O_RDONLY);
        if (fd < 0)
          continue;
        char tmp_name[PROPERTY_VALUE_MAX];
        memset (tmp_name, 0, PROPERTY_VALUE_MAX);
        int ret = read(fd, tmp_name, PROPERTY_VALUE_MAX);
        if (ret <= 0)
          continue;

        if(!found_scaler && (strncmp(scaler_device_name, tmp_name, strlen(scaler_device_name)) == 0)){
          found_scaler = true;
          scaler_dev_path_ = "/dev/";
          scaler_dev_path_ += ent->d_name;
          continue;
        }
        if(!found_rx && (strncmp(rx_device_name, tmp_name, strlen(rx_device_name)) == 0)){
          found_rx = true;
          hdmi_rx_dev_path_ = "/dev/";
          hdmi_rx_dev_path_ += ent->d_name;
          continue;
        }
        if (found_scaler && found_rx)
          break;
      }
    }
  }

  if (found_rx && found_scaler)
    return 0;
  return -ENODEV;
}


// Initialize source pad of the scaler.
// Check current HDMI link format and resolution.
// Set scaler's source pad params accordingly
int V4L2XilinxHdmiWrapper::InitScalerSource() {
  std::lock_guard<std::mutex> lock(subdevs_lock_);
  int fd = open(hdmi_rx_dev_path_.c_str(), O_RDONLY);
  if (fd < 0){
      HAL_LOGE("Failed to open HDMI RX subdevice %s", hdmi_rx_dev_path_.c_str());
      return -ENODEV;
  }
  base::ScopedFD rx_fd(fd);

  // Get current HDMI input resolution
  struct v4l2_dv_timings timings;
  memset(&timings, 0, sizeof(timings));
  int ret = ioctl(rx_fd.get(), VIDIOC_SUBDEV_QUERY_DV_TIMINGS, &timings);
  if (ret < 0){
    HAL_LOGE("Failed to get HDMI RX timings: %s", strerror(errno));
    return ret;
  }

  // Get bus pixel format
  struct v4l2_subdev_format fmt_req;
  memset(&fmt_req, 0, sizeof(fmt_req));
  //HDMI RX has only 1 pad
  fmt_req.pad = 0;
  fmt_req.which = V4L2_SUBDEV_FORMAT_ACTIVE;
  ret = ioctl(rx_fd.get(), VIDIOC_SUBDEV_G_FMT, &fmt_req);
  if (ret < 0){
    HAL_LOGE("Failed to get HDMI RX format: %s", strerror(errno));
    return ret;
  }

  // Now open and setup scaler subdevice
  fd = open(scaler_dev_path_.c_str(), O_RDWR);
  if (fd < 0){
      HAL_LOGE("Failed to open scaler subdevice %s", scaler_dev_path_.c_str());
      return ret;
  }
  base::ScopedFD scaler_fd(fd);

  // Use format we received from HDMI RX
  // pad 0 (Sink) is connected to HDMI RX
  fmt_req.pad = 0;
  fmt_req.which = V4L2_SUBDEV_FORMAT_ACTIVE;
  ret = ioctl(scaler_fd.get(), VIDIOC_SUBDEV_S_FMT, &fmt_req);
  if (ret < 0){
    HAL_LOGE("Failed to set scaler format");
    return ret;
  }

  return 0;
}

int V4L2XilinxHdmiWrapper::Connect() {
  HAL_LOG_ENTER();
  // First search for scaler and HDMI RX subdevices
  // Do it each time on Connect() to be sure we
  // use proper nodes
  int ret = FindSubdevices();
  if(ret){
    HAL_LOGE("Failed to find required v4l-subdevives");
    return ret;
  }

  ret = InitScalerSource();
  if(ret){
    HAL_LOGE("Failed to initialize video scaler");
    return ret;
  }

  // Now we can call base's Connect()
  // It will call GetFormats() which will use initialized
  // subdevices nodes
  return V4L2Wrapper::Connect();
}

void V4L2XilinxHdmiWrapper::Disconnect() {
  HAL_LOG_ENTER();
  V4L2Wrapper::Disconnect();
  {
    std::lock_guard<std::mutex> buffer_lock(buffer_queue_lock_);
    buffers_.clear();
  }
}

int V4L2XilinxHdmiWrapper::GetFormats(std::set<uint32_t>* v4l2_formats) {
  HAL_LOG_ENTER();
  v4l2_formats->insert(V4L2_PIX_FMT_YUYV);
  return 0;
}

int V4L2XilinxHdmiWrapper::GetFormatFrameSizes(uint32_t v4l2_format,
                                     std::set<std::array<int32_t, 2>>* sizes) {
  sizes->insert({{{static_cast<int32_t>(640),
                   static_cast<int32_t>(480)}}});
  return 0;
}

// Converts a v4l2_fract with units of seconds to an int64_t with units of ns.
inline int64_t FractToNs(const v4l2_fract& fract) {
  return (1000000000LL * fract.numerator) / fract.denominator;
}

int V4L2XilinxHdmiWrapper::GetFormatFrameDurationRange(
    uint32_t v4l2_format,
    const std::array<int32_t, 2>& size,
    std::array<int64_t, 2>* duration_range) {
  // Potentially called so many times logging entry is a bad idea.

  v4l2_frmivalenum duration_query;
  memset(&duration_query, 0, sizeof(duration_query));
  int64_t min = std::numeric_limits<int64_t>::max();
  int64_t max = std::numeric_limits<int64_t>::min();
  duration_query.discrete.numerator = 1;
  duration_query.discrete.denominator = 60;
  min = FractToNs(duration_query.discrete);
  duration_query.discrete.denominator = 30;
  max = FractToNs(duration_query.discrete);
  (*duration_range)[0] = min;
  (*duration_range)[1] = max;
  return 0;
}

int V4L2XilinxHdmiWrapper::SetFormat(const StreamFormat& desired_format,
                           uint32_t* result_max_buffers) {
  HAL_LOG_ENTER();

  if (format_ && desired_format == *format_) {
    HAL_LOGV("Already in correct format, skipping format setting.");
    *result_max_buffers = buffers_.size();
    return 0;
  }

  // Select the matching format, or if not available, select a qualified format
  // we can convert from.
  SupportedFormat format;
  if (!StreamFormat::FindBestFitFormat(supported_formats_, qualified_formats_,
                                       desired_format.v4l2_pixel_format(),
                                       desired_format.width(),
                                       desired_format.height(), &format)) {
    HAL_LOGE(
        "Unable to find supported resolution in list, "
        "width: %d, height: %d",
        desired_format.width(), desired_format.height());
    return -EINVAL;
  }

  // Set the camera to the new format.
  v4l2_format new_format;
  const StreamFormat resolved_format(format);
  resolved_format.FillFormatRequest(&new_format);

  // TODO(b/29334616): When async, this will need to check if the stream
  // is on, and if so, lock it off while setting format.
  if (IoctlLocked(VIDIOC_S_FMT, &new_format) < 0) {
    HAL_LOGE("S_FMT failed: %s", strerror(errno));
    return -ENODEV;
  }

  // Check that the driver actually set to the requested values.
  if (resolved_format != new_format) {
    HAL_LOGE("Device doesn't support desired stream configuration.");
    return -EINVAL;
  }

  // Keep track of our new format.
  format_.reset(new StreamFormat(new_format));

  // Format changed, request new buffers.
  int res = RequestBuffers(2);
  if (res) {
    HAL_LOGE("Requesting buffers for new format failed.");
    return res;
  }
  *result_max_buffers = buffers_.size();
  return 0;
}

int V4L2XilinxHdmiWrapper::EnqueueRequest(
    std::shared_ptr<default_camera_hal::CaptureRequest> request) {
  if (!format_) {
    HAL_LOGE("Stream format must be set before enqueuing buffers.");
    return -ENODEV;
  }

  // Find a free buffer index. Could use some sort of persistent hinting
  // here to improve expected efficiency, but buffers_.size() is expected
  // to be low enough (<10 experimentally) that it's not worth it.
  int index = -1;
  {
    std::lock_guard<std::mutex> guard(buffer_queue_lock_);
    for (size_t i = 0; i < buffers_.size(); ++i) {
      if (!buffers_[i].active) {
        index = i;
        break;
      }
    }
  }
  if (index < 0) {
    // Note: The HAL should be tracking the number of buffers in flight
    // for each stream, and should never overflow the device.
    HAL_LOGE("Cannot enqueue buffer: stream is already full.");
    return -ENODEV;
  }

  // Set up a v4l2 buffer struct.
  v4l2_buffer device_buffer;
  memset(&device_buffer, 0, sizeof(device_buffer));

  device_buffer.type = format_->type();
  device_buffer.index = index;
  device_buffer.memory = V4L2_MEMORY_MMAP;
  // Use QUERYBUF to ensure our buffer/device is in good shape,
  // and fill out remaining fields.
  if (IoctlLocked(VIDIOC_QUERYBUF, &device_buffer) < 0) {
    HAL_LOGE("QUERYBUF fails: %s", strerror(errno));
    // Return buffer index.
    std::lock_guard<std::mutex> guard(buffer_queue_lock_);
    buffers_[index].active = false;
    return -ENODEV;
  }

  // Setup our request context
  XilinxRequestContext* request_context;
  {
    std::lock_guard<std::mutex> guard(buffer_queue_lock_);
    request_context = &buffers_[index];
    request_context->request = request;
  }

  // Pass the buffer to the camera.
  if (IoctlLocked(VIDIOC_QBUF, &device_buffer) < 0) {
    HAL_LOGE("QBUF fails: %s", strerror(errno));
    return -ENODEV;
  }

  // Mark the buffer as in flight.
  std::lock_guard<std::mutex> guard(buffer_queue_lock_);
  buffers_[index].active = true;

  return 0;
}

int V4L2XilinxHdmiWrapper::DequeueRequest(std::shared_ptr<CaptureRequest>* request) {
  if (!format_) {
    HAL_LOGV(
        "Format not set, so stream can't be on, "
        "so no buffers available for dequeueing");
    return -EAGAIN;
  }

  v4l2_buffer buffer;
  memset(&buffer, 0, sizeof(buffer));
  buffer.type = format_->type();
  buffer.memory = V4L2_MEMORY_MMAP;
  int res = IoctlLocked(VIDIOC_DQBUF, &buffer);
  if (res) {
    if (errno == EAGAIN) {
      // Expected failure.
      return -EAGAIN;
    } else {
      // Unexpected failure.
      HAL_LOGE("DQBUF fails: %s", strerror(errno));
      return -ENODEV;
    }
  }
  std::lock_guard<std::mutex> guard(buffer_queue_lock_);
  XilinxRequestContext* request_context = &buffers_[buffer.index];

  // Lock the camera stream buffer for painting.
  const camera3_stream_buffer_t* stream_buffer =
      &request_context->request->output_buffers[0];
  uint32_t fourcc =
      StreamFormat::HalToV4L2PixelFormat(stream_buffer->stream->format);
  if (request) {
    *request = request_context->request;
  }
  // Note that the device buffer length is passed to the output frame. If the
  // GrallocFrameBuffer does not have support for the transformation to
  // |fourcc|, it will assume that the amount of data to lock is based on
  // |buffer.length|, otherwise it will use the ImageProcessor::ConvertedSize.
  arc::GrallocFrameBuffer output_frame(
      *stream_buffer->buffer, stream_buffer->stream->width,
      stream_buffer->stream->height, fourcc, buffer.length,
      stream_buffer->stream->usage);
  res = output_frame.Map();
  if (res) {
    HAL_LOGE("Failed to map output frame.");
    request_context->request.reset();
    return -EINVAL;
  }

  if (request_context->camera_buffer->GetFourcc() == fourcc &&
      request_context->camera_buffer->GetWidth() ==
          stream_buffer->stream->width &&
      request_context->camera_buffer->GetHeight() ==
          stream_buffer->stream->height) {
    // If no format conversion needs to be applied, directly copy the data over.
    memcpy(output_frame.GetData(), request_context->camera_buffer->GetData(),
           request_context->camera_buffer->GetDataSize());
  } else {
    // Perform the format conversion.
    arc::CachedFrame cached_frame;
    cached_frame.SetSource(request_context->camera_buffer.get(), 0);
    cached_frame.Convert(request_context->request->settings, &output_frame);
  }
  request_context->request.reset();
  // Mark the buffer as not in flight.
  request_context->active = false;
  return 0;
}

int V4L2XilinxHdmiWrapper::RequestBuffers(uint32_t num_requested) {
  v4l2_requestbuffers req_buffers;
  memset(&req_buffers, 0, sizeof(req_buffers));
  req_buffers.type = format_->type();
  req_buffers.memory = V4L2_MEMORY_MMAP;
  req_buffers.count = num_requested;

  int res = IoctlLocked(VIDIOC_REQBUFS, &req_buffers);
  // Calling REQBUFS releases all queued buffers back to the user.
  if (res < 0) {
    HAL_LOGE("REQBUFS failed: %s", strerror(errno));
    return -ENODEV;
  }

  // V4L2 will set req_buffers.count to a number of buffers it can handle.
  if (num_requested > 0 && req_buffers.count < 1) {
    HAL_LOGE("REQBUFS claims it can't handle any buffers.");
    return -ENODEV;
  }

  buffers_.resize(req_buffers.count);

  std::lock_guard<std::mutex> guard(buffer_queue_lock_);

  for (size_t i = 0; i < buffers_.size(); ++i) {
    if (!buffers_[i].active) {
      struct v4l2_buffer device_buffer;
      memset(&device_buffer, 0, sizeof(device_buffer));
      device_buffer.type = format_->type();
      device_buffer.index = i;
      device_buffer.memory = V4L2_MEMORY_MMAP;
      if (IoctlLocked(VIDIOC_QUERYBUF, &device_buffer) < 0) {
        HAL_LOGE("QUERYBUF fails: %s", strerror(errno));
        return -ENODEV;
      }
      XilinxRequestContext* request_context = &buffers_[i];
      request_context->camera_buffer->Reset();
      request_context->camera_buffer->SetDataSize(device_buffer.length);
      request_context->camera_buffer->SetFourcc(format_->v4l2_pixel_format());
      request_context->camera_buffer->SetWidth(format_->width());
      request_context->camera_buffer->SetHeight(format_->height());
      request_context->camera_buffer->SetFd(device_fd_.get());
      request_context->camera_buffer->SetOffset(device_buffer.m.offset);
      if (request_context->camera_buffer->Map()){
        HAL_LOGE("Buffer Map() fails");
        return -ENODEV;
      }
    }
  }
  return 0;
}

int V4L2XilinxHdmiWrapper::GetInFlightBufferCount() {
  int count = 0;
  std::lock_guard<std::mutex> guard(buffer_queue_lock_);
  for (auto& buffer : buffers_) {
    if (buffer.active) {
      count++;
    }
  }
  return count;
}

}