/* Copyright 2017 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#ifndef HAL_USB_FRAME_BUFFER_H_
#define HAL_USB_FRAME_BUFFER_H_

#include <stdint.h>

#include <memory>
#include <string>

#include <android-base/unique_fd.h>
#include <base/synchronization/lock.h>

#include <hardware/gralloc.h>

namespace arc {

class FrameBuffer {
 public:
  FrameBuffer();
  virtual ~FrameBuffer();

  // If mapped successfully, the address will be assigned to |data_| and return
  // 0. Otherwise, returns -EINVAL.
  virtual int Map() = 0;

  // Unmaps the mapped address. Returns 0 for success.
  virtual int Unmap() = 0;

  uint8_t* GetData() const { return data_; }
  size_t GetDataSize() const { return data_size_; }
  size_t GetBufferSize() const { return buffer_size_; }
  uint32_t GetWidth() const { return width_; }
  uint32_t GetHeight() const { return height_; }
  uint32_t GetFourcc() const { return fourcc_; }

  void SetFourcc(uint32_t fourcc) { fourcc_ = fourcc; }
  virtual int SetDataSize(size_t data_size);

 protected:
  uint8_t* data_;

  // The number of bytes used in the buffer.
  size_t data_size_;

  // The number of bytes allocated in the buffer.
  size_t buffer_size_;

  // Frame resolution.
  uint32_t width_;
  uint32_t height_;

  // This is V4L2_PIX_FMT_* in linux/videodev2.h.
  uint32_t fourcc_;
};

// AllocatedFrameBuffer is used for the buffer from hal malloc-ed. User should
// be aware to manage the memory.
class AllocatedFrameBuffer : public FrameBuffer {
 public:
  explicit AllocatedFrameBuffer(int buffer_size);
  explicit AllocatedFrameBuffer(uint8_t* buffer, int buffer_size);
  ~AllocatedFrameBuffer() override;

  // No-op for the two functions.
  int Map() override { return 0; }
  int Unmap() override { return 0; }

  void SetWidth(uint32_t width) { width_ = width; }
  void SetHeight(uint32_t height) { height_ = height; }
  int SetDataSize(size_t data_size) override;
  void Reset();

 private:
  std::unique_ptr<uint8_t[]> buffer_;
};

// V4L2DmaFrameBuffer is used for the buffer from V4L2CameraDevice. Maps the fd
// in constructor. Unmaps and closes the fd in destructor.
class V4L2DmaFrameBuffer : public FrameBuffer {
 public:
  V4L2DmaFrameBuffer();
  // Unmaps |data_| and closes |fd_|.
  ~V4L2DmaFrameBuffer();

  int Map() override;
  int Unmap() override;
  int SetFd(int fd);
  void SetWidth(uint32_t width) { width_ = width; }
  void SetHeight(uint32_t height) { height_ = height; }
  int SetDataSize(size_t data_size) override;
  int GetFd() const { return fd_.get(); }
  // Unmaps |data_| and closes |fd_|.
  void Reset();
 private:
  // File descriptor of V4L2 frame buffer.
  android::base::unique_fd fd_;

  bool is_mapped_;

  // Lock to guard |is_mapped_|.
  base::Lock lock_;
};

// V4L2MmapedFrameBuffer is used for the V4L2_MEMORY_MMAP buffer from V4L2CameraDevice.
// Constructor just initializes empty buffer.
// User should SetOffset with offset retrieved from VIDIOC_QUERYBUF and call Map()
class V4L2MmapedFrameBuffer : public FrameBuffer {
 public:
  V4L2MmapedFrameBuffer();
  // Unmaps |data_|
  ~V4L2MmapedFrameBuffer();

  int Map() override;
  int Unmap() override;
  int SetOffset(uint32_t offset);
  uint32_t GetOffset() const { return offset_; }
  int SetFd(int fd);
  void SetWidth(uint32_t width) { width_ = width; }
  void SetHeight(uint32_t height) { height_ = height; }
  int SetDataSize(size_t data_size) override;
  void Reset();
 private:
  // offset of the buffer from the start of the V4L2 device memory
  // as returned by VIDIOC_QUERYBUF
  uint32_t offset_;

  // FD of V4L2 device to perform mmap'ing from
  // cannot be changed if current buffer is mmap'ed
  int dev_fd_;

  bool is_mapped_;

  // Lock to guard |is_mapped_|.
  base::Lock lock_;
};

// GrallocFrameBuffer is used for the buffer from Android framework. Uses
// CameraBufferMapper to lock and unlock the buffer.
class GrallocFrameBuffer : public FrameBuffer {
 public:
  GrallocFrameBuffer(buffer_handle_t buffer, uint32_t width, uint32_t height,
                     uint32_t fourcc, uint32_t device_buffer_length,
                     uint32_t stream_usage);
  ~GrallocFrameBuffer();

  int Map() override;
  int Unmap() override;

 private:
  // The currently used buffer for |buffer_mapper_| operations.
  buffer_handle_t buffer_;

  // Used to import gralloc buffer.
  const gralloc_module_t* gralloc_module_;

  bool is_mapped_;

  // Lock to guard |is_mapped_|.
  base::Lock lock_;

  // Camera stream and device buffer context.
  uint32_t device_buffer_length_;
  uint32_t stream_usage_;
};

}  // namespace arc

#endif  // HAL_USB_FRAME_BUFFER_H_
