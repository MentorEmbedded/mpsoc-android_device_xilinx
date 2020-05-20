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

#ifndef V4L2_CAMERA_HAL_V4L2_WRAPPER_H_
#define V4L2_CAMERA_HAL_V4L2_WRAPPER_H_

#include <array>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <android-base/unique_fd.h>

#include "arc/common_types.h"
#include "arc/frame_buffer.h"
#include "capture_request.h"
#include "common.h"
#include "stream_format.h"

namespace v4l2_camera_hal {

class V4L2Wrapper {
 public:
  // Use this method to create V4L2Wrapper objects. Functionally equivalent
  // to "new V4L2Wrapper", except that it may return nullptr in case of failure.
  // Depending on the specified device_path it will create proper device specific
  // wrapper
  static V4L2Wrapper* NewV4L2Wrapper(const std::string device_path);
  virtual ~V4L2Wrapper();

  // Helper class to ensure all opened connections are closed.
  class Connection {
   public:
    Connection(std::shared_ptr<V4L2Wrapper> device)
        : device_(std::move(device)), connect_result_(device_->Connect()) {}
    ~Connection() {
      if (connect_result_ == 0) {
        device_->Disconnect();
      }
    }
    // Check whether the connection succeeded or not.
    inline int status() const { return connect_result_; }

   private:
    std::shared_ptr<V4L2Wrapper> device_;
    const int connect_result_;
  };

  // Turn the stream on or off.
  virtual int StreamOn();
  virtual int StreamOff();
  // Manage controls.
  virtual int QueryControl(uint32_t control_id, v4l2_query_ext_ctrl* result);
  virtual int GetControl(uint32_t control_id, int32_t* value);
  virtual int SetControl(uint32_t control_id,
                         int32_t desired,
                         int32_t* result = nullptr);
  // Manage format.
  virtual int GetFormats(std::set<uint32_t>* v4l2_formats);
  virtual int GetQualifiedFormats(std::vector<uint32_t>* v4l2_formats);
  virtual int GetFormatFrameSizes(uint32_t v4l2_format,
                                  std::set<std::array<int32_t, 2>>* sizes);

  // Durations are returned in ns.
  virtual int GetFormatFrameDurationRange(
      uint32_t v4l2_format,
      const std::array<int32_t, 2>& size,
      std::array<int64_t, 2>* duration_range);
  virtual int SetFormat(const StreamFormat& desired_format,
                        uint32_t* result_max_buffers) = 0;
  // Manage buffers.
  virtual int EnqueueRequest(
      std::shared_ptr<default_camera_hal::CaptureRequest> request) = 0;
  virtual int DequeueRequest(
      std::shared_ptr<default_camera_hal::CaptureRequest>* request) = 0;
  virtual int GetInFlightBufferCount() = 0;

 protected:
  // Constructor is protected to allow failing on bad input.
  // Use NewV4L2Wrapper instead.
  V4L2Wrapper(const std::string device_path);

  // Connect or disconnect to the device. Access by creating/destroying
  // a V4L2Wrapper::Connection object.
  virtual int Connect();
  virtual void Disconnect();
  // Perform an ioctl call in a thread-safe fashion.
  template <typename T>
  int IoctlLocked(int request, T data) {
    // Potentially called so many times logging entry is a bad idea.
    std::lock_guard<std::mutex> lock(device_lock_);

    if (!connected()) {
      HAL_LOGE("Device %s not connected.", device_path_.c_str());
      return -ENODEV;
    }
    return TEMP_FAILURE_RETRY(ioctl(device_fd_.get(), request, data));
  }

  template <typename T>
  int SubdevIoctl(const std::string subdev_path, int request, T data) {

    int fd = open(subdev_path.c_str(), O_RDWR);
    if (fd < 0){
        HAL_LOGE("Failed to open V4L subdevice %s", subdev_path.c_str());
        return -ENODEV;
    }
    android::base::unique_fd dev_fd(fd);
    return TEMP_FAILURE_RETRY(ioctl(dev_fd.get(), request, data));
  }

  int SubdevSetCtrl(const std::string subdev_path, uint32_t id, uint32_t value);
  int SetSubdevFormat(const std::string dev_path,
    uint32_t width, uint32_t height, uint32_t fmt_code, uint32_t pad);

  // Request/release userspace buffer mode via VIDIOC_REQBUFS.
  virtual int RequestBuffers(uint32_t num_buffers) = 0;

  std::unique_ptr<StreamFormat> format_;
  // Supported formats.
  arc::SupportedFormats supported_formats_;
  // Qualified formats.
  arc::SupportedFormats qualified_formats_; 
  // The opened device fd.
  android::base::unique_fd device_fd_;
private:

  inline bool connected() { return device_fd_.get() >= 0; }

  // Format management.
  const arc::SupportedFormats GetSupportedFormats();

  // The camera device path. For example, /dev/video0.
  const std::string device_path_;
  // The underlying gralloc module.
  // std::unique_ptr<V4L2Gralloc> gralloc_;
  // Whether or not the device supports the extended control query.
  bool extended_query_supported_;

  // Lock protecting use of the device.
  std::mutex device_lock_;
  // Lock protecting connecting/disconnecting the device.
  std::mutex connection_lock_;
  // Reference count connections.
  int connection_count_;

  friend class Connection;
  friend class V4L2WrapperMock;

  DISALLOW_COPY_AND_ASSIGN(V4L2Wrapper);
};

}  // namespace v4l2_camera_hal

#endif  // V4L2_CAMERA_HAL_V4L2_WRAPPER_H_
