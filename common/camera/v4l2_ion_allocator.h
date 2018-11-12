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

#ifndef V4L2_CAMERA_HAL_V4L2_ION_ALLOCATOR_H_
#define V4L2_CAMERA_HAL_V4L2_ION_ALLOCATOR_H_

#include <string>
#include <android-base/unique_fd.h>

namespace v4l2_camera_hal {

// V4L2Gralloc is a wrapper around ION memory allocator
class V4L2IonAllocator {
 public:
  // Use this method to create V4L2IonAllocator objects. Functionally equivalent
  // to "new V4L2IonAllocator", except that it may return nullptr in case of failure.
  static V4L2IonAllocator* NewV4L2IonAllocator(const std::string heap_name);
  virtual ~V4L2IonAllocator();

  // Allocates buffer from ION heap and returns FD ready for mmap()
  // or -1 on error
  int AllocateDmaFd(size_t size);
 private:
  // Constructor is private to allow failing on bad input.
  // Use V4L2IonAllocator instead.
  V4L2IonAllocator(int ion_fd, unsigned int heap_id);

  android::base::unique_fd ion_fd_;
  // Heap ID we are allocating from
  unsigned int heap_id_;
};
}  // namespace v4l2_camera_hal
#endif  // V4L2_CAMERA_HAL_V4L2_ION_ALLOCATOR_H_
