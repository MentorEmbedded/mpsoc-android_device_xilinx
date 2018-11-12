#include "v4l2_ion_allocator.h"
#include <ion/ion.h>
#include "ion_4.12.h"

#include "common.h"

namespace v4l2_camera_hal {

V4L2IonAllocator* V4L2IonAllocator::NewV4L2IonAllocator(const std::string heap_name) {
	int fd = ion_open();
	if (fd < 0) {
		HAL_LOGE("Cannot open ION device");
		return nullptr;
	}

	int i, ret, cnt, heap_id = -1;
	ret = ion_query_heap_cnt(fd, &cnt);
	if (ret) {
		HAL_LOGE("ion count query failed with %s", strerror(errno));
		ion_close(fd);
		return nullptr;
	}

	if (cnt < 1) {
		HAL_LOGE("No ION heaps available");
		ion_close(fd);
		return nullptr;
	}

	std::unique_ptr<ion_heap_data[]> data;
	data.reset(new ion_heap_data[cnt]);
	ret = ion_query_get_heaps(fd, cnt, data.get());
	if (ret) {
		HAL_LOGE("Error querying heaps from ion %s", strerror(errno));
		ion_close(fd);
		data.reset();
		return nullptr;
	}

	for (i = 0; i < cnt; i++) {
		if (strcmp(data[i].name, heap_name.c_str()) == 0) {
			heap_id = data[i].heap_id;
			break;
		}
	}

	if (i == cnt) {
		HAL_LOGE("No %s Heap Found amongst %d heaps\n", heap_name.c_str(), cnt);
		ion_close(fd);
		data.reset();
		return nullptr;
	}
	data.reset();
  return new V4L2IonAllocator(fd, heap_id);
}

V4L2IonAllocator::V4L2IonAllocator(int ion_fd, unsigned int heap_id)
	:	ion_fd_(ion_fd), 
		heap_id_(heap_id) {}

V4L2IonAllocator::~V4L2IonAllocator() {
	ion_fd_.reset(-1); //Includes close
}

int V4L2IonAllocator::AllocateDmaFd(size_t size) {
	int share_fd = -1;
	int ret = ion_alloc_fd(ion_fd_.get(), size, 0, 1 << heap_id_, 0, &share_fd);
	if (ret) {
		HAL_LOGE("Failed to ion_alloc_fd with %s", strerror(errno));
		return -1;
	}
	return share_fd;
}

} // namespace v4l2_camera_hal
