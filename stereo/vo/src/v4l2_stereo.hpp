// 左右拼接 UVC 双目相机的 V4L2 采集：YUYV mmap，只取 Y 平面劈成左右两目。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

class V4l2Stereo {
 public:
  ~V4l2Stereo();

  // width/height 是拼接帧尺寸（如 1280x480，每目 640x480）。失败返回 false 并写 err
  bool open(const std::string& dev, int width, int height, int fps, std::string& err);
  // 写 V4L2 控件（曝光、增益），失败返回 false
  bool setControl(uint32_t id, int32_t value);

  // 阻塞等新帧；把队列里已就绪的帧全部出队，只用最新一帧（VO 慢于相机时自动丢旧帧，保证低延迟）。
  // t_mono：驱动缓冲区时间戳（CLOCK_MONOTONIC 秒）；skipped：本次丢掉的旧帧数（含驱动层序号跳变）。
  // swap = true 时拼接帧右半当左目（由排线接法决定，见 AGENTS.md 5.8）
  bool grab(cv::Mat& left, cv::Mat& right, double& t_mono, int& skipped, bool swap, std::string& err);

 private:
  struct Buffer {
    void* ptr = nullptr;
    size_t len = 0;
  };
  int fd_ = -1;
  int width_ = 0, height_ = 0, bytes_per_line_ = 0;
  std::vector<Buffer> buffers_;
  int64_t last_seq_ = -1;
};
