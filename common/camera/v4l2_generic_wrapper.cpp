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
#define LOG_TAG "V4L2GenericWrapper"

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

using arc::AllocatedFrameBuffer;
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

V4L2GenericWrapper::V4L2GenericWrapper(const std::string device_path) :
	V4L2Wrapper(device_path) {
}

V4L2GenericWrapper::~V4L2GenericWrapper() {}

void V4L2GenericWrapper::Disconnect() {
  HAL_LOG_ENTER();
  V4L2Wrapper::Disconnect();
  {
    std::lock_guard<std::mutex> buffer_lock(buffer_queue_lock_);
    buffers_.clear();
  }
}

int V4L2GenericWrapper::StreamOff() {
  int ret = V4L2Wrapper::StreamOff();
  if (ret)
    return ret;

  std::lock_guard<std::mutex> lock(buffer_queue_lock_);
  for (auto& buffer : buffers_) {
    buffer.active = false;
    buffer.request.reset();
  }
  return 0;
}

int V4L2GenericWrapper::SetFormat(const StreamFormat& desired_format,
                           uint32_t* result_max_buffers) {
  HAL_LOG_ENTER();

  if (format_ && desired_format == *format_) {
    HAL_LOGV("Already in correct format, skipping format setting.");
    *result_max_buffers = buffers_.size();
    return 0;
  }

  if (format_) {
    // If we had an old format, first request 0 buffers to inform the device
    // we're no longer using any previously "allocated" buffers from the old
    // format. This seems like it shouldn't be necessary for USERPTR memory,
    // and/or should happen from turning the stream off, but the driver
    // complained. May be a driver issue, or may be intended behavior.
    int res = RequestBuffers(0);
    if (res) {
      return res;
    }
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
  int res = RequestBuffers(1);
  if (res) {
    HAL_LOGE("Requesting buffers for new format failed.");
    return res;
  }
  *result_max_buffers = buffers_.size();
  return 0;
}

int V4L2GenericWrapper::RequestBuffers(uint32_t num_requested) {
  v4l2_requestbuffers req_buffers;
  memset(&req_buffers, 0, sizeof(req_buffers));
  req_buffers.type = format_->type();
  req_buffers.memory = V4L2_MEMORY_USERPTR;
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
  return 0;
}

int V4L2GenericWrapper::EnqueueRequest(
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

  // Use QUERYBUF to ensure our buffer/device is in good shape,
  // and fill out remaining fields.
  if (IoctlLocked(VIDIOC_QUERYBUF, &device_buffer) < 0) {
    HAL_LOGE("QUERYBUF fails: %s", strerror(errno));
    // Return buffer index.
    std::lock_guard<std::mutex> guard(buffer_queue_lock_);
    buffers_[index].active = false;
    return -ENODEV;
  }

  // Setup our request context and fill in the user pointer field.
  RequestContext* request_context;
  void* data;
  {
    std::lock_guard<std::mutex> guard(buffer_queue_lock_);
    request_context = &buffers_[index];
    request_context->camera_buffer->SetDataSize(device_buffer.length);
    request_context->camera_buffer->Reset();
    request_context->camera_buffer->SetFourcc(format_->v4l2_pixel_format());
    request_context->camera_buffer->SetWidth(format_->width());
    request_context->camera_buffer->SetHeight(format_->height());
    request_context->request = request;
    data = request_context->camera_buffer->GetData();
  }
  device_buffer.m.userptr = reinterpret_cast<unsigned long>(data);

  // Pass the buffer to the camera.
  if (IoctlLocked(VIDIOC_QBUF, &device_buffer) < 0) {
    HAL_LOGE("QBUF fails: %s", strerror(errno));
    return -ENODEV;
  }

  // Mark the buffer as in flight.
  std::lock_guard<std::mutex> guard(buffer_queue_lock_);
  request_context->active = true;

  return 0;
}

int V4L2GenericWrapper::DequeueRequest(std::shared_ptr<CaptureRequest>* request) {
  if (!format_) {
    HAL_LOGV(
        "Format not set, so stream can't be on, "
        "so no buffers available for dequeueing");
    return -EAGAIN;
  }

  v4l2_buffer buffer;
  memset(&buffer, 0, sizeof(buffer));
  buffer.type = format_->type();
  buffer.memory = V4L2_MEMORY_USERPTR;
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
  RequestContext* request_context = &buffers_[buffer.index];

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

int V4L2GenericWrapper::GetInFlightBufferCount() {
  int count = 0;
  std::lock_guard<std::mutex> guard(buffer_queue_lock_);
  for (auto& buffer : buffers_) {
    if (buffer.active) {
      count++;
    }
  }
  return count;
}

}  // namespace v4l2_camera_hal
