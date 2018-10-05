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

#ifndef V4L2_CAMERA_HAL_V4L2_XILINX_HDMI_WRAPPER_H_
#define V4L2_CAMERA_HAL_V4L2_XILINX_HDMI_WRAPPER_H_

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

class V4L2XilinxHdmiWrapper: public V4L2Wrapper {
 public:
  V4L2XilinxHdmiWrapper(const std::string device_path);
  ~V4L2XilinxHdmiWrapper();

  // Manage format.
  int GetFormats(std::set<uint32_t>* v4l2_formats);
  int GetFormatFrameSizes(uint32_t v4l2_format,
                                  std::set<std::array<int32_t, 2>>* sizes);

  // Durations are returned in ns.
  int GetFormatFrameDurationRange(
      uint32_t v4l2_format,
      const std::array<int32_t, 2>& size,
      std::array<int64_t, 2>* duration_range);
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
  int Connect();

  int RequestBuffers(uint32_t num_buffers);
 private:
  // Finds /dev/v4l-subdevX devices for scaler and HDMI RX
  // Returns -1 on error, 0 otherwise
  int FindSubdevices();

  // Initialize source pad of the scaler.
  // Check current HDMI link format and resolution.
  // Set scaler's source pad params accordingly
  int InitScalerSource();

  // Set format of scaler's source pad.
  // Scaler's source pad is connected to the capture node /dev/videoN.
  // This function converts specified StreamFormat to the matching
  // MEDIA_BUS_FMT_
  int SetScalerFormat(const StreamFormat& format);

  // Convert V4L2 fourcc to MEDIA_BUS_FMT_
  // Should be aligned with xilinx_vip.c driver
  // returns 0 on error
  static uint32_t V4L2ToScalerMbusFormat(uint32_t v4l2_pixel_format);

  class XilinxRequestContext {
   public:
    XilinxRequestContext()
        : active(false),
          camera_buffer(std::make_shared<arc::V4L2MmapedFrameBuffer>()){};
    ~XilinxRequestContext(){};
    // Indicates whether this request context is in use.
    bool active;
    // Buffer handles of the context.
    std::shared_ptr<arc::V4L2MmapedFrameBuffer> camera_buffer;
    std::shared_ptr<default_camera_hal::CaptureRequest> request;
  };

  // Lock protecting use of the buffer tracker.
  std::mutex buffer_queue_lock_;

  std::vector<XilinxRequestContext> buffers_;

  std::string scaler_dev_path_;
  std::string hdmi_rx_dev_path_;
  std::mutex subdevs_lock_;

  DISALLOW_COPY_AND_ASSIGN(V4L2XilinxHdmiWrapper);
};

}  // namespace v4l2_camera_hal

#endif  // V4L2_CAMERA_HAL_V4L2_XILINX_HDMI_WRAPPER_H_
