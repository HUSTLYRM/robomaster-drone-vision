#include "calib.hpp"

#include <stdexcept>

#include <opencv2/calib3d.hpp>

namespace {

cv::Mat readMat(const cv::FileStorage& fs, const char* name, int rows, int cols) {
  cv::Mat m;
  fs[name] >> m;
  if (m.empty()) throw std::runtime_error(std::string("标定文件缺少 ") + name);
  m.convertTo(m, CV_64F);
  if (cols > 0 && m.total() == static_cast<size_t>(rows * cols)) m = m.reshape(1, rows);
  return m;
}

}  // namespace

StereoCalib StereoCalib::load(const std::string& path) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) throw std::runtime_error("打不开标定文件 " + path);
  StereoCalib c;
  c.size = cv::Size(static_cast<int>(fs["image_width"]), static_cast<int>(fs["image_height"]));
  c.K1 = readMat(fs, "K1", 3, 3);
  c.D1 = readMat(fs, "D1", 1, 0);
  c.R1 = readMat(fs, "R1", 3, 3);
  c.P1 = readMat(fs, "P1", 3, 4);
  c.K2 = readMat(fs, "K2", 3, 3);
  c.D2 = readMat(fs, "D2", 1, 0);
  c.R2 = readMat(fs, "R2", 3, 3);
  c.P2 = readMat(fs, "P2", 3, 4);
  if (c.size.width <= 0 || c.size.height <= 0) throw std::runtime_error("标定文件分辨率无效");
  // 基线为负说明左右接反（AGENTS.md 5.8），继续算出来的深度全是负的
  if (c.baseline() <= 0) throw std::runtime_error("基线 <= 0：左右目接反了，检查 swap 设置并重标");
  return c;
}

cv::Matx33d StereoCalib::rectK() const {
  return cv::Matx33d(fx(), 0, cx(), 0, fy(), cy(), 0, 0, 1);
}

void StereoCalib::rectifyLeft(const std::vector<cv::Point2f>& raw, std::vector<cv::Point2f>& rect) const {
  if (raw.empty()) {
    rect.clear();
    return;
  }
  cv::undistortPoints(raw, rect, K1, D1, R1, P1);
}

void StereoCalib::rectifyRight(const std::vector<cv::Point2f>& raw, std::vector<cv::Point2f>& rect) const {
  if (raw.empty()) {
    rect.clear();
    return;
  }
  cv::undistortPoints(raw, rect, K2, D2, R2, P2);
}

void StereoCalib::projectLeftRaw(const std::vector<cv::Point3d>& pts_rect, std::vector<cv::Point2f>& raw) const {
  raw.assign(pts_rect.size(), cv::Point2f(-1.f, -1.f));
  if (pts_rect.empty()) return;
  const cv::Matx33d R1t = cv::Matx33d(R1).t();
  std::vector<cv::Point3d> cam;
  std::vector<size_t> idx;
  for (size_t i = 0; i < pts_rect.size(); ++i) {
    if (pts_rect[i].z <= 0) continue;  // 相机背后的点投影无意义
    cam.emplace_back(R1t * cv::Vec3d(pts_rect[i].x, pts_rect[i].y, pts_rect[i].z));
    idx.push_back(i);
  }
  if (cam.empty()) return;
  std::vector<cv::Point2d> out;
  cv::projectPoints(cam, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K1, D1, out);
  for (size_t k = 0; k < idx.size(); ++k) raw[idx[k]] = cv::Point2f(static_cast<float>(out[k].x), static_cast<float>(out[k].y));
}

void StereoCalib::guessRightRaw(const std::vector<cv::Point2f>& left_rect, double disparity,
                                std::vector<cv::Point2f>& right_raw) const {
  right_raw.clear();
  if (left_rect.empty()) return;
  // 校正后右目像素 -> 右目校正系下的射线 -> 转回右目原始相机系 -> 加畸变投影
  const cv::Matx33d R2t = cv::Matx33d(R2).t();
  std::vector<cv::Point3d> rays;
  rays.reserve(left_rect.size());
  for (const auto& p : left_rect) {
    const cv::Vec3d ray((p.x - disparity - cx()) / fx(), (p.y - cy()) / fy(), 1.0);
    rays.emplace_back(R2t * ray);
  }
  std::vector<cv::Point2d> out;
  cv::projectPoints(rays, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K2, D2, out);
  right_raw.reserve(out.size());
  for (const auto& p : out) right_raw.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
}
