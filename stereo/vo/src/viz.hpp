// VO 预览窗口（调试用，机载不开）：左目画面 + 跟踪点（按深度着色）+ 俯视轨迹（FLU，前上左左）+ 状态栏。
#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "vo.hpp"

class VoViz {
 public:
  // scale：窗口初始缩放（之后可拖拽/最大化）。本机 GNOME 下原尺寸 1120×520 显示不全（实测），默认 0.8
  explicit VoViz(double scale = 0.8) : scale_(scale) {}

  // 累积轨迹并画出整幅预览图（不显示，便于离线检查）
  cv::Mat render(const cv::Mat& left, const StereoVO& vo, const VoResult& r, const cv::Vec3d& pos_flu, double yaw_deg,
                 double fps);
  // render 后刷新窗口；返回 false 表示用户按了 q/Esc
  bool show(const cv::Mat& left, const StereoVO& vo, const VoResult& r, const cv::Vec3d& pos_flu, double yaw_deg,
            double fps);

 private:
  double scale_;
  bool window_ready_ = false;
  std::vector<cv::Point2d> track_;  // 俯视轨迹（前, 左），只记有效帧
  std::vector<cv::Point2d> lost_;   // 丢失/重置发生的位置
  int last_reset_ = 0;
};
