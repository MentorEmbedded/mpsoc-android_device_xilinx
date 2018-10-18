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

// #define LOG_NDEBUG 0
#define LOG_TAG "V4L2XilinxCsiWrapper"

#include "v4l2_xilinx_csi_wrapper.h"

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
#include <linux/media-bus-format.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <android-base/unique_fd.h>
#include <cutils/properties.h>

#include "arc/cached_frame.h"

#include "xilinx-v4l2-controls.h"

// FIX ME: Xilinx's specific definition is not in kernel-headers yet
#define MEDIA_BUS_FMT_VYYUYY8_1X24    0x202c

// FIX ME: using hardcoded values from VCU TRD 2018.1 for now
// currently HAL is using QueryControl/SetControl/GetControl directly
// on video node (/dev/video0). With Xilinx pipeline this approach not always works.
// We need to intercept some controls and use proper subdev nodes instead.
#define CSI_ACT_LANES 4
#define GAMMA_BLUE_COR  10 /* 10 equals passthrough */
#define GAMMA_GREEN_COR 10 /* 10 equals passthrough */
#define GAMMA_RED_COR 10 /* 10 equals passthrough */
#define CSC_BRIGHTNESS  80
#define CSC_CONTRAST  55
#define CSC_BLUE_GAIN 40
#define CSC_GREEN_GAIN  24
#define CSC_RED_GAIN  35
#define IMX274_EXPOSURE   16636
#define IMX274_GAIN   5120
#define IMX274_VERTICAL_FLIP  0
#define IMX274_TEST_PATTERN 0
#define IMX274_DEF_WIDTH 1920
#define IMX274_DEF_HEIGHT 1080
#define IMX274_DEF_FORMAT MEDIA_BUS_FMT_SRGGB8_1X8 /* SRGGB8 */

#define DMSC_OUT_DEF_FORMAT MEDIA_BUS_FMT_RBG888_1X24 /* RBG24 */

namespace v4l2_camera_hal {

using arc::V4L2MmapedFrameBuffer;
using arc::SupportedFormat;
using arc::SupportedFormats;
using default_camera_hal::CaptureRequest;

// Xilinx scaler supports resolutions from 32x32 to 4096x4096
// For now just declare that we support a set of standard sizes
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

V4L2XilinxCsiWrapper::V4L2XilinxCsiWrapper(const std::string device_path) :
  V4L2Wrapper(device_path) {
}

V4L2XilinxCsiWrapper::~V4L2XilinxCsiWrapper() {}

// Find all required subdevices paths
// for the CSI pipeline
// FIXME: pipeline content is now hardcoded, make it dynamic.
int V4L2XilinxCsiWrapper::FindSubdevices() {
  HAL_LOG_ENTER();
  std::lock_guard<std::mutex> lock(subdevs_lock_);
  bool found_scaler = false;
  bool found_csiss = false;
  bool found_dmsc = false;
  bool found_csc = false;
  bool found_sensor = false;
  bool found_gamma = false;

  // name of main csi capture device in device tree
  char csi_device_name[PROPERTY_VALUE_MAX];

  // name of scaler subdevice
  char scaler_device_name[PROPERTY_VALUE_MAX];

  // name of CSI SS subdevice
  char csiss_device_name[PROPERTY_VALUE_MAX];

  // name of sensor subdevice
  char sensor_device_name[PROPERTY_VALUE_MAX];

  // name of VPSS CSC subdevice
  char csc_device_name[PROPERTY_VALUE_MAX];

  // name of Gamma subdevice
  char gamma_device_name[PROPERTY_VALUE_MAX];

  // name of Demosaic subdevice
  char dmsc_device_name[PROPERTY_VALUE_MAX];

  property_get("xlnx.v4l2.csi.main_dts_name", csi_device_name, "vcap_csi");
  property_get("xlnx.v4l2.csi.scaler_dts_name", scaler_device_name, "a0200000.scaler");
  property_get("xlnx.v4l2.csi.csc_dts_name", csc_device_name, "a0240000.csc");
  property_get("xlnx.v4l2.csi.gamma_dts_name", gamma_device_name, "a0270000.v_gamma");
  property_get("xlnx.v4l2.csi.dmsc_dts_name", dmsc_device_name, "a0250000.v_demosaic");
  property_get("xlnx.v4l2.csi.csiss_dts_name", csiss_device_name, "a00f0000.csiss");
  property_get("xlnx.v4l2.csi.sensor_dts_name", sensor_device_name, "IMX274");

  std::string sysfs_path = "/sys/devices/platform/amba/amba:";
  sysfs_path += csi_device_name;
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
        if(!found_csiss && (strncmp(csiss_device_name, tmp_name, strlen(csiss_device_name)) == 0)){
          found_csiss = true;
          csiss_dev_path_ = "/dev/";
          csiss_dev_path_ += ent->d_name;
          continue;
        }
        if(!found_dmsc && (strncmp(dmsc_device_name, tmp_name, strlen(dmsc_device_name)) == 0)){
          found_dmsc = true;
          dmsc_dev_path_ = "/dev/";
          dmsc_dev_path_ += ent->d_name;
          continue;
        }
        if(!found_csc && (strncmp(csc_device_name, tmp_name, strlen(csc_device_name)) == 0)){
          found_csc = true;
          csc_dev_path_ = "/dev/";
          csc_dev_path_ += ent->d_name;
          continue;
        }
        if(!found_sensor && (strncmp(sensor_device_name, tmp_name, strlen(sensor_device_name)) == 0)){
          found_sensor = true;
          sensor_dev_path_ = "/dev/";
          sensor_dev_path_ += ent->d_name;
          continue;
        }
        if(!found_gamma && (strncmp(gamma_device_name, tmp_name, strlen(gamma_device_name)) == 0)){
          found_gamma = true;
          gamma_dev_path_ = "/dev/";
          gamma_dev_path_ += ent->d_name;
          continue;
        }              
        if (found_scaler && found_csiss && found_csc &&
            found_dmsc && found_sensor && found_gamma)
          break;
      }
    }
  }

  if (found_scaler && found_csiss && found_csc &&
      found_dmsc && found_sensor && found_gamma)
    return 0;
  return -ENODEV;
}

template <typename T>
int V4L2XilinxCsiWrapper::SubdevIoctl(const std::string subdev_path, int request, T data) {

  int fd = open(subdev_path.c_str(), O_RDWR);
  if (fd < 0){
      HAL_LOGE("Failed to open V4L subdevice %s", subdev_path.c_str());
      return -ENODEV;
  }
  base::ScopedFD dev_fd(fd);
  return TEMP_FAILURE_RETRY(ioctl(dev_fd.get(), request, data));
}

int V4L2XilinxCsiWrapper::SubdevSetCtrl(const std::string subdev_path, uint32_t id, uint32_t value) {
  int ret;
  struct v4l2_queryctrl query;
  struct v4l2_control ctrl;

  memset(&query, 0, sizeof(query));
  query.id = id;
  ret = SubdevIoctl(subdev_path, VIDIOC_QUERYCTRL, &query);
  if (ret)
    return ret;

  if (query.flags & V4L2_CTRL_FLAG_DISABLED) {
    HAL_LOGW("V4L2_CID_%d is disabled\n", id);
  } else {
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = id;
    ctrl.value = value;
    ret = SubdevIoctl(subdev_path, VIDIOC_S_CTRL, &ctrl);
    if (ret)
      return ret;
  }
  return 0;

}

int V4L2XilinxCsiWrapper::CsiSetActLanes(uint32_t lanes) {
  return SubdevSetCtrl(csiss_dev_path_, V4L2_CID_XILINX_MIPICSISS_ACT_LANES, lanes);
}

int V4L2XilinxCsiWrapper::GammaSetBlueCorrection(uint32_t blue){
  return SubdevSetCtrl(gamma_dev_path_, V4L2_CID_XILINX_GAMMA_CORR_BLUE_GAMMA, blue);
}

int V4L2XilinxCsiWrapper::GammaSetGreenCorrection(uint32_t green){
  return SubdevSetCtrl(gamma_dev_path_, V4L2_CID_XILINX_GAMMA_CORR_GREEN_GAMMA, green);
}

int V4L2XilinxCsiWrapper::GammaSetRedCorrection(uint32_t red){
  return SubdevSetCtrl(gamma_dev_path_, V4L2_CID_XILINX_GAMMA_CORR_RED_GAMMA, red);
}

int V4L2XilinxCsiWrapper::SetGammaCorrections(uint32_t blue, uint32_t green, uint32_t red) {
  int ret;
  ret = GammaSetBlueCorrection(blue);
  if (ret)
    return ret;

  ret = GammaSetRedCorrection(red);
  if (ret)
    return ret;

  ret = GammaSetGreenCorrection(green);
  if (ret)
    return ret;

  return 0;
}

int V4L2XilinxCsiWrapper::CscSetBrightness(uint32_t value){
  return SubdevSetCtrl(csc_dev_path_, V4L2_CID_XILINX_CSC_BRIGHTNESS, value);
}

int V4L2XilinxCsiWrapper::CscSetContrast(uint32_t value){
  return SubdevSetCtrl(csc_dev_path_, V4L2_CID_XILINX_CSC_CONTRAST, value);
}

int V4L2XilinxCsiWrapper::CscSetBlueGain(uint32_t value){
  return SubdevSetCtrl(csc_dev_path_, V4L2_CID_XILINX_CSC_BLUE_GAIN, value);
}

int V4L2XilinxCsiWrapper::CscSetGreenGain(uint32_t value){
  return SubdevSetCtrl(csc_dev_path_, V4L2_CID_XILINX_CSC_GREEN_GAIN, value);
}

int V4L2XilinxCsiWrapper::CscSetRedGain(uint32_t value){
  return SubdevSetCtrl(csc_dev_path_, V4L2_CID_XILINX_CSC_RED_GAIN, value);
}

int V4L2XilinxCsiWrapper::SetCscDefaults() {
  int ret;

  /* Set CSC defaults */
  ret = CscSetBrightness(CSC_BRIGHTNESS);
  if (ret)
    return ret;

  ret = CscSetContrast(CSC_CONTRAST);
  if (ret)
    return ret;  

  ret = CscSetBlueGain(CSC_BLUE_GAIN);
  if (ret)
    return ret;  

  ret = CscSetGreenGain(CSC_GREEN_GAIN);
  if (ret)
    return ret;

  ret = CscSetRedGain(CSC_RED_GAIN);
  if (ret)
    return ret;
  return 0;
}

int V4L2XilinxCsiWrapper::SensorSetTestPattern(uint32_t value) {
  return SubdevSetCtrl(sensor_dev_path_, V4L2_CID_TEST_PATTERN, value);
}

int V4L2XilinxCsiWrapper::SensorSetVerticalFlip(uint32_t value) {
  return SubdevSetCtrl(sensor_dev_path_, V4L2_CID_VFLIP, value);
}

int V4L2XilinxCsiWrapper::SensorSetExposure(uint32_t value) {
  return SubdevSetCtrl(sensor_dev_path_, V4L2_CID_EXPOSURE, value);
}

int V4L2XilinxCsiWrapper::SensorSetGain(uint32_t value) {
  return SubdevSetCtrl(sensor_dev_path_, V4L2_CID_GAIN, value);
}

int V4L2XilinxCsiWrapper::SetSensorDefaults() {
  int ret;

  /* Set sensor controls */
  ret = SensorSetTestPattern(IMX274_TEST_PATTERN);
  if (ret){
    HAL_LOGE("Failed to set sensor test pattern: %s", strerror(errno));
    return ret;    
  }

  ret = SensorSetVerticalFlip(IMX274_VERTICAL_FLIP);
  if (ret){
    HAL_LOGE("Failed to set sensor vertical flip: %s", strerror(errno));
    return ret;    
  }  

  ret = SensorSetExposure(IMX274_EXPOSURE);
  if (ret){
    HAL_LOGE("Failed to set sensor exposure: %s", strerror(errno));
    return ret;    
  }

  ret = SensorSetGain(IMX274_GAIN);
  if (ret){
    HAL_LOGE("Failed to set sensor gain: %s", strerror(errno));
    return ret;    
  }
  return 0;
}

// Initialize pipeline of subdevices
// Use procedure from VCU TRD 2018.1
int V4L2XilinxCsiWrapper::InitPipeline() {
  HAL_LOG_ENTER();
  std::lock_guard<std::mutex> lock(subdevs_lock_);
  int ret;

  /* Set active number of lanes */
  ret = CsiSetActLanes(CSI_ACT_LANES);
  if (ret) {
    HAL_LOGE("Failed to set active number of lanes on CSI: %s", strerror(errno));
    return ret;
  }

  /* Set gamma corrections */
  ret = SetGammaCorrections(GAMMA_BLUE_COR, GAMMA_GREEN_COR, GAMMA_RED_COR);
  if (ret) {
    HAL_LOGE("Failed to set gamma corrections: %s", strerror(errno));
    return ret;
  }

  ret = SetCscDefaults();
  if (ret) {
    HAL_LOGE("Failed to set defaults for CSC: %s", strerror(errno));
    return ret;
  }

  ret = SetSensorDefaults();
  if (ret) {
    HAL_LOGE("Failed to set defaults for IMX274 sensor: %s", strerror(errno));
    return ret;
  }

  return 0;
}

int V4L2XilinxCsiWrapper::Connect() {
  HAL_LOG_ENTER();
  // First search CSI pipeline subdevices
  // Do it each time on Connect() to be sure we
  // use proper nodes
  int ret = FindSubdevices();
  if(ret){
    HAL_LOGE("Failed to find required v4l-subdevices");
    return ret;
  }

  ret = InitPipeline();
  if(ret){
    HAL_LOGE("Failed to initialize pipeline of v4l-subdevices");
    return ret;
  }

  return V4L2Wrapper::Connect();
}

void V4L2XilinxCsiWrapper::Disconnect() {
  HAL_LOG_ENTER();
  V4L2Wrapper::Disconnect();
  {
    std::lock_guard<std::mutex> buffer_lock(buffer_queue_lock_);
    buffers_.clear();
  }
}

int V4L2XilinxCsiWrapper::GetFormats(std::set<uint32_t>* v4l2_formats) {
  HAL_LOG_ENTER();
  v4l2_formats->insert(V4L2_PIX_FMT_YUYV);
  v4l2_formats->insert(V4L2_PIX_FMT_UYVY);
  v4l2_formats->insert(V4L2_PIX_FMT_NV12);
  v4l2_formats->insert(V4L2_PIX_FMT_BGR24);
  v4l2_formats->insert(V4L2_PIX_FMT_RGB24);
  return 0;
}

int V4L2XilinxCsiWrapper::GetFormatFrameSizes(uint32_t v4l2_format,
                                     std::set<std::array<int32_t, 2>>* sizes) {

  for (const auto size : kStandardSizes) {
    sizes->insert({{{static_cast<int32_t>(size[0]),
                   static_cast<int32_t>(size[1])}}});
  }
  return 0;
}

// Converts a v4l2_fract with units of seconds to an int64_t with units of ns.
inline int64_t FractToNs(const v4l2_fract& fract) {
  return (1000000000LL * fract.numerator) / fract.denominator;
}

int V4L2XilinxCsiWrapper::GetFormatFrameDurationRange(
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

// Convert V4L2 fourcc to MEDIA_BUS_FMT_
// Should be aligned with xilinx_vip.c driver
uint32_t V4L2XilinxCsiWrapper::V4L2ToScalerMbusFormat(uint32_t v4l2_pixel_format) {
  switch (v4l2_pixel_format) {
    case V4L2_PIX_FMT_NV12:
      return MEDIA_BUS_FMT_VYYUYY8_1X24;
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_NV16:
      return MEDIA_BUS_FMT_UYVY8_1X16;
    case V4L2_PIX_FMT_BGR24:
    case V4L2_PIX_FMT_RGB24:
      return MEDIA_BUS_FMT_RBG888_1X24;
    default:
      // Unsupported format
      HAL_LOGV("v4l2 pixel format %u is not supported by scaler", v4l2_pixel_format);
      break;
  }
  return 0;
}

int V4L2XilinxCsiWrapper::SetSubdevFormat(const std::string dev_path,
  uint32_t width, uint32_t height, uint32_t fmt_code, uint32_t pad) {
  int ret;

  struct v4l2_subdev_format fmt_req;
  memset(&fmt_req, 0, sizeof(fmt_req));

  fmt_req.pad = pad;
  fmt_req.which = V4L2_SUBDEV_FORMAT_ACTIVE;
  fmt_req.format.width = width;
  fmt_req.format.height = height;
  fmt_req.format.code = fmt_code;
  fmt_req.format.colorspace = V4L2_COLORSPACE_DEFAULT;
  fmt_req.format.field = V4L2_FIELD_NONE;

  ret = SubdevIoctl(dev_path, VIDIOC_SUBDEV_S_FMT, &fmt_req);
  if (ret < 0){
    HAL_LOGE("Failed to set subdevice format: %s", strerror(errno));
    return ret;
  }
  return 0;
}

int V4L2XilinxCsiWrapper::SetSensorFormat(uint32_t width, uint32_t height, uint32_t fmt_code) {
  // sensor has only one pad0
  return SetSubdevFormat(sensor_dev_path_, width, height, fmt_code, 0);
}

int V4L2XilinxCsiWrapper::SetCsiSsInFormat(uint32_t width, uint32_t height, uint32_t fmt_code) {
  // CSI SS pad1 is sink(input)
  return SetSubdevFormat(csiss_dev_path_, width, height, fmt_code, 1);
}

int V4L2XilinxCsiWrapper::SetCsiSsOutFormat(uint32_t width, uint32_t height, uint32_t fmt_code) {
  // CSI SS pad0 is source(output)
  return SetSubdevFormat(csiss_dev_path_, width, height, fmt_code, 0);
}

int V4L2XilinxCsiWrapper::SetDmscInFormat(uint32_t width, uint32_t height, uint32_t fmt_code) {
  // DMSC SS pad0 is sink(input)
  return SetSubdevFormat(dmsc_dev_path_, width, height, fmt_code, 0);
}

int V4L2XilinxCsiWrapper::SetDmscOutFormat(uint32_t width, uint32_t height, uint32_t fmt_code) {
  // DMSC pad1 is source(output)
  return SetSubdevFormat(dmsc_dev_path_, width, height, fmt_code, 1);
}

int V4L2XilinxCsiWrapper::SetGammaInFormat(uint32_t width, uint32_t height, uint32_t fmt_code) {
  // gamma pad0 is sink(input)
  return SetSubdevFormat(gamma_dev_path_, width, height, fmt_code, 0);
}

int V4L2XilinxCsiWrapper::SetGammaOutFormat(uint32_t width, uint32_t height, uint32_t fmt_code) {
  // gamma pad1 is source(output)
  return SetSubdevFormat(gamma_dev_path_, width, height, fmt_code, 1);
}

int V4L2XilinxCsiWrapper::SetCscInFormat(uint32_t width, uint32_t height, uint32_t fmt_code) {
  // CSC pad0 is sink(input)
  return SetSubdevFormat(csc_dev_path_, width, height, fmt_code, 0);
}

int V4L2XilinxCsiWrapper::SetCscOutFormat(uint32_t width, uint32_t height, uint32_t fmt_code) {
  // CSC pad1 is source(output)
  return SetSubdevFormat(csc_dev_path_, width, height, fmt_code, 1);
}

int V4L2XilinxCsiWrapper::SetScalerInFormat(uint32_t width, uint32_t height, uint32_t fmt_code) {
  // scaler pad0 is sink(input)
  return SetSubdevFormat(scaler_dev_path_, width, height, fmt_code, 0);
}

int V4L2XilinxCsiWrapper::SetScalerOutFormat(uint32_t width, uint32_t height, uint32_t fmt_code) {
  // scaler pad1 is source(output)
  return SetSubdevFormat(scaler_dev_path_, width, height, fmt_code, 1);
}

// Set pipeline format
// Use procedure from VCU TRD:
//    - color format of sensor is hardcoded, only resolution can be changed
//    - format is propogated from sensor through all the subdevices
//    - scaler's source pad is connected to video output device
//    - scaler's source pad format is set according to video output format
int V4L2XilinxCsiWrapper::SetPipelineFormat(const StreamFormat& format) {
  HAL_LOG_ENTER();
  // first check specified format
  uint32_t mbus_fmt_code = V4L2ToScalerMbusFormat(format.v4l2_pixel_format());
  if (mbus_fmt_code == 0){
      HAL_LOGE("Wrong v4l2 pixel format %u", format.v4l2_pixel_format());
      return -EINVAL;
  }

  // now we should lock
  std::lock_guard<std::mutex> lock(subdevs_lock_);
  uint32_t sensor_width = format.width();
  uint32_t sensor_height = format.height();

  /* 
    Pipeline formats:
      sensor -> demosaic IN: IMX274_DEF_FORMAT
      demosaic OUT -> scaler IN: DMSC_OUT_DEF_FORMAT
      scaler OUT -> video OUT(/dev/videoN): matches StreamFormat specified by HAL
  */

  // try to set specified resultion on sensor
  int ret = SetSensorFormat(sensor_width, sensor_height, IMX274_DEF_FORMAT);
  if (ret){
    HAL_LOGW("Failed to set sensor resultion: %s", strerror(errno));
    HAL_LOGW("Trying to set default resultion");
    sensor_width = IMX274_DEF_WIDTH;
    sensor_height = IMX274_DEF_HEIGHT;
    // try to set default one
    ret = SetSensorFormat(sensor_width, sensor_height, IMX274_DEF_FORMAT);
    if(ret){
      HAL_LOGE("Failed to set default sensor resolution: %s", strerror(errno));
      return ret;  
    }
  }

  /* Set CSI SS IN format */
  ret = SetCsiSsInFormat(sensor_width, sensor_height, IMX274_DEF_FORMAT);
  if (ret){
    HAL_LOGE("Failed to set CSI SS in format: %s", strerror(errno));
    return ret;
  }

  /* Set CSI SS OUT format */
  ret = SetCsiSsOutFormat(sensor_width, sensor_height, IMX274_DEF_FORMAT);
  if (ret){
    HAL_LOGE("Failed to set CSI SS out format: %s", strerror(errno));
    return ret;
  }

  /* Set Demosaic In pad format */
  ret = SetDmscInFormat(sensor_width, sensor_height, IMX274_DEF_FORMAT);
  if (ret){
    HAL_LOGE("Failed to set demosaic in format: %s", strerror(errno));
    return ret;
  }

  /* Set Demosaic Out pad format */
  ret = SetDmscOutFormat(sensor_width, sensor_height, DMSC_OUT_DEF_FORMAT);
  if (ret){
    HAL_LOGE("Failed to set demosaic out format: %s", strerror(errno));
    return ret;
  }

  /* Set Gamma In pad format */
  ret = SetGammaInFormat(sensor_width, sensor_height, DMSC_OUT_DEF_FORMAT);
  if (ret){
    HAL_LOGE("Failed to set gamma in format: %s", strerror(errno));
    return ret;
  }

  /* Set Gamma Out pad format */
  ret = SetGammaOutFormat(sensor_width, sensor_height, DMSC_OUT_DEF_FORMAT);
  if (ret){
    HAL_LOGE("Failed to set gamma out format: %s", strerror(errno));
    return ret;
  }

  /* Set CSC In pad format */
  ret = SetCscInFormat(sensor_width, sensor_height, DMSC_OUT_DEF_FORMAT);
  if (ret){
    HAL_LOGE("Failed to set csc in format: %s", strerror(errno));
    return ret;
  }

  /* Set CSC Out pad format */
  ret = SetCscOutFormat(sensor_width, sensor_height, DMSC_OUT_DEF_FORMAT);
  if (ret){
    HAL_LOGE("Failed to set csc out format: %s", strerror(errno));
    return ret;
  }

  /* Set Scaler In pad format */
  ret = SetScalerInFormat(sensor_width, sensor_height, DMSC_OUT_DEF_FORMAT);
  if (ret){
    HAL_LOGE("Failed to set scaler in format: %s", strerror(errno));
    return ret;
  }

  /* Set Scaler Out pad format */
  ret = SetScalerOutFormat(sensor_width, sensor_height, mbus_fmt_code);
  if (ret){
    HAL_LOGE("Failed to set scaler out format: %s", strerror(errno));
    return ret;
  }

  return 0;
}

int V4L2XilinxCsiWrapper::SetFormat(const StreamFormat& desired_format,
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
    // format. Otherwise driver can return EBUSY
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
  int res;

  res = SetPipelineFormat(resolved_format);
  if (res){
    HAL_LOGE("Failed to set pipeline format: %s", strerror(errno));
    return res;
  }

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
  res = RequestBuffers(2);
  if (res) {
    HAL_LOGE("Requesting buffers for new format failed.");
    return res;
  }
  *result_max_buffers = buffers_.size();
  return 0;
}

int V4L2XilinxCsiWrapper::EnqueueRequest(
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

int V4L2XilinxCsiWrapper::DequeueRequest(std::shared_ptr<CaptureRequest>* request) {
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

int V4L2XilinxCsiWrapper::RequestBuffers(uint32_t num_requested) {
  // before requesting new buffers we need to unmap and delete all
  // current buffers. Otherwise driver can return EBUSY
  std::lock_guard<std::mutex> buffer_lock(buffer_queue_lock_);
  buffers_.clear();

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

int V4L2XilinxCsiWrapper::GetInFlightBufferCount() {
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