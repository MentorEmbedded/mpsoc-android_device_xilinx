/*
 * Copyright (C) 2016 The Android Open Source Project
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

#ifndef V4L2_CAMERA_HAL_V4L2_GENERIC_WRAPPER_H_
#define V4L2_CAMERA_HAL_V4L2_GENERIC_WRAPPER_H_

#include <array>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <android-base/unique_fd.h>

#include "v4l2_wrapper.h"

#include "arc/common_types.h"
#include "arc/frame_buffer.h"
#include "capture_request.h"
#include "common.h"
#include "stream_format.h"

namespace v4l2_camera_hal {

class V4L2GenericWrapper: public V4L2Wrapper {
 public:
  V4L2GenericWrapper(const std::string device_path);
  ~V4L2GenericWrapper();

  int StreamOff();  

  int SetFormat(const StreamFormat& desired_format,
                        uint32_t* result_max_buffers);
  // Manage buffers.
  int EnqueueRequest(
      std::shared_ptr<default_camera_hal::CaptureRequest> request);
  int DequeueRequest(
      std::shared_ptr<default_camera_hal::CaptureRequest>* request);
  int GetInFlightBufferCount();

 protected:
  // Override Disconnect() to release buffers on device closing
  void Disconnect();

  // Request/release userspace buffer mode via VIDIOC_REQBUFS.
  int RequestBuffers(uint32_t num_buffers);

 private:
  // Lock protecting use of the buffer tracker.
  std::mutex buffer_queue_lock_;

  class RequestContext {
   public:
    RequestContext()
        : active(false),
          camera_buffer(std::make_shared<arc::AllocatedFrameBuffer>(0)){};
    ~RequestContext(){};
    // Indicates whether this request context is in use.
    bool active;
    // Buffer handles of the context.
    std::shared_ptr<arc::AllocatedFrameBuffer> camera_buffer;
    std::shared_ptr<default_camera_hal::CaptureRequest> request;
  };

  // Map of in flight requests.
  // |buffers_.size()| will always be the maximum number of buffers this device
  // can handle in its current format.
  std::vector<RequestContext> buffers_;

  DISALLOW_COPY_AND_ASSIGN(V4L2GenericWrapper);
};

}  // namespace v4l2_camera_hal

#endif  // V4L2_CAMERA_HAL_V4L2_GENERIC_WRAPPER_H_
