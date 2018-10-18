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

#ifndef V4L2_CAMERA_HAL_V4L2_XILINX_CSI_WRAPPER_H_
#define V4L2_CAMERA_HAL_V4L2_XILINX_CSI_WRAPPER_H_

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

class V4L2XilinxCsiWrapper: public V4L2Wrapper {
 public:
  V4L2XilinxCsiWrapper(const std::string device_path);
  ~V4L2XilinxCsiWrapper();

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
  // Finds /dev/v4l-subdevX devices for CSI pipeline
  // Returns -1 on error, 0 otherwise
  // locks on subdevs_lock_
  int FindSubdevices();

  // Initialize pipeline of subdevices
  // Use procedure from VCU TRD 2018.1
  // locks on subdevs_lock_
  int InitPipeline();

  // Set pipeline format
  // Use procedure from VCU TRD:
  //    - color format of sensor is hardcoded, only resolution can be changed
  //    - format is propogated from sensor through all the subdevices
  //    - scaler's source pad is connected to video output device
  //    - scaler's source pad format is set according to video output format
  // locks on subdevs_lock_
  int SetPipelineFormat(const StreamFormat& format);

  // set various subdev controls
  // doesn't provide any locking
  // must be called with subdevs_lock_ held
  int SubdevSetCtrl(const std::string subdev_path, uint32_t id, uint32_t value);
  int CsiSetActLanes(uint32_t lanes);
  int GammaSetBlueCorrection(uint32_t blue);
  int GammaSetGreenCorrection(uint32_t green);
  int GammaSetRedCorrection(uint32_t red);
  int SetGammaCorrections(uint32_t blue, uint32_t green, uint32_t red);
  int SetCscDefaults();
  int CscSetBrightness(uint32_t value);
  int CscSetContrast(uint32_t value);
  int CscSetBlueGain(uint32_t value);
  int CscSetGreenGain(uint32_t value);
  int CscSetRedGain(uint32_t value);  
  int SetSensorDefaults();
  int SensorSetTestPattern(uint32_t test_pattern);
  int SensorSetVerticalFlip(uint32_t value);
  int SensorSetExposure(uint32_t value);
  int SensorSetGain(uint32_t value);
  int SensorInitTestPatternNames();
  int SetSensorFormat(uint32_t width, uint32_t height, uint32_t fmt_code);
  int SetCsiSsInFormat(uint32_t width, uint32_t height, uint32_t fmt_code);
  int SetCsiSsOutFormat(uint32_t width, uint32_t height, uint32_t fmt_code);
  int SetDmscInFormat(uint32_t width, uint32_t height, uint32_t fmt_code);
  int SetDmscOutFormat(uint32_t width, uint32_t height, uint32_t fmt_code);
  int SetGammaInFormat(uint32_t width, uint32_t height, uint32_t fmt_code);
  int SetGammaOutFormat(uint32_t width, uint32_t height, uint32_t fmt_code);
  int SetCscInFormat(uint32_t width, uint32_t height, uint32_t fmt_code);
  int SetCscOutFormat(uint32_t width, uint32_t height, uint32_t fmt_code);
  int SetScalerInFormat(uint32_t width, uint32_t height, uint32_t fmt_code);
  int SetScalerOutFormat(uint32_t width, uint32_t height, uint32_t fmt_code);
  int SetSubdevFormat(const std::string dev_path,
  uint32_t width, uint32_t height, uint32_t fmt_code, uint32_t pad);

  // Convert V4L2 fourcc to MEDIA_BUS_FMT_
  // Should be aligned with xilinx_vip.c driver
  // returns 0 on error
  static uint32_t V4L2ToScalerMbusFormat(uint32_t v4l2_pixel_format);

  // Perform ioctl call on  the specifed v4l subdevice
  // should be called with subdevs_lock_ held
  template <typename T>
  int SubdevIoctl(const std::string subdev_path, int request, T data);

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

  // CSI pipeline v4l-subdevs paths used to configure formats
  // FIXME: pipeline is hardcoded for now, make it dynamic:
  std::string scaler_dev_path_;
  std::string gamma_dev_path_;
  std::string csiss_dev_path_;
  std::string csc_dev_path_;
  std::string sensor_dev_path_;
  std::string dmsc_dev_path_;
  // all access to subdevs should be protected with this lock
  std::mutex subdevs_lock_;

  DISALLOW_COPY_AND_ASSIGN(V4L2XilinxCsiWrapper);
};

}  // namespace v4l2_camera_hal

#endif  // V4L2_CAMERA_HAL_V4L2_XILINX_CSI_WRAPPER_H_
