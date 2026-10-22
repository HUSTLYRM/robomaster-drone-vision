// 双目标定：读 stereo/calibrate.py 导出的 OpenCV FileStorage（stereo.yml），提供特征点级校正。
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

struct StereoCalib {
  cv::Size size;                   // 每目分辨率
  cv::Mat K1, D1, R1, P1;          // 左目：内参、畸变、校正旋转、校正后投影
  cv::Mat K2, D2, R2, P2;          // 右目；P2(0,3) = -fx * 基线（ROS/OpenCV 约定）

  // 读文件，缺字段或基线符号不对直接抛 std::runtime_error
  static StereoCalib load(const std::string& path);

  double fx() const { return P1.at<double>(0, 0); }
  double fy() const { return P1.at<double>(1, 1); }
  double cx() const { return P1.at<double>(0, 2); }
  double cy() const { return P1.at<double>(1, 2); }
  double baseline() const { return -P2.at<double>(0, 3) / P2.at<double>(0, 0); }
  cv::Matx33d rectK() const;       // 校正后左目内参（无畸变）

  // 原始像素坐标 -> 校正后像素坐标（只算点，不 remap 整幅图）
  void rectifyLeft(const std::vector<cv::Point2f>& raw, std::vector<cv::Point2f>& rect) const;
  void rectifyRight(const std::vector<cv::Point2f>& raw, std::vector<cv::Point2f>& rect) const;
  // 校正后左目坐标 + 假设视差 -> 右目原始像素坐标，用作左右 KLT 的初值（两目主点差几十像素，不给初值容易跟丢）
  void guessRightRaw(const std::vector<cv::Point2f>& left_rect, double disparity,
                     std::vector<cv::Point2f>& right_raw) const;
  // 校正后左目相机系的 3D 点 -> 左目原始像素（运动预测给 KLT 初值用）；z <= 0 的点输出 (-1,-1)
  void projectLeftRaw(const std::vector<cv::Point3d>& pts_rect, std::vector<cv::Point2f>& raw) const;
};
