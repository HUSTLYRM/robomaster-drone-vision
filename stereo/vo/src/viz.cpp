#include "viz.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace {

constexpr int kImgW = 640, kImgH = 480, kMapW = 480, kBarH = 40;
const char* kWindow = "stereo_vo";

// 深度 -> 颜色：近红远蓝（0.5～8 m）
cv::Scalar depthColor(float z) {
  const float t = std::clamp((std::log(std::max(z, 0.5f)) - std::log(0.5f)) / (std::log(8.f) - std::log(0.5f)), 0.f, 1.f);
  return cv::Scalar(255 * t, 255 * (1 - std::abs(2 * t - 1)), 255 * (1 - t));
}

}  // namespace

bool VoViz::show(const cv::Mat& left, const StereoVO& vo, const VoResult& r, const cv::Vec3d& pos_flu, double yaw_deg,
                 double fps) {
  // 点 × 关窗后 imshow 会把窗口重新建出来、关不掉（实测），所以先查窗口还在不在，不在就退出
  if (window_ready_ && cv::getWindowProperty(kWindow, cv::WND_PROP_AUTOSIZE) < 0) return false;
  cv::Mat canvas = render(left, vo, r, pos_flu, yaw_deg, fps);
  if (!window_ready_) {
    // 可拖拽缩放、可最大化，画面随窗口缩放；初始按 scale_ 缩小
    cv::namedWindow(kWindow, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(kWindow, static_cast<int>(canvas.cols * scale_), static_cast<int>(canvas.rows * scale_));
    window_ready_ = true;
  }
  cv::imshow(kWindow, canvas);
  const int k = cv::waitKey(1) & 0xFF;
  return !(k == 'q' || k == 27);
}

cv::Mat VoViz::render(const cv::Mat& left, const StereoVO& vo, const VoResult& r, const cv::Vec3d& pos_flu,
                      double yaw_deg, double fps) {
  if (r.ok) track_.emplace_back(pos_flu[0], pos_flu[1]);
  if (r.reset_count != last_reset_) {
    lost_.emplace_back(pos_flu[0], pos_flu[1]);
    last_reset_ = r.reset_count;
  }

  cv::Mat canvas(kImgH + kBarH, kImgW + kMapW, CV_8UC3, cv::Scalar(30, 30, 30));

  // 左：左目画面 + 跟踪点
  cv::Mat img;
  cv::cvtColor(left, img, cv::COLOR_GRAY2BGR);
  const auto& pts = vo.trackedPoints();
  const auto& pts3 = vo.keyframePoints();
  for (size_t i = 0; i < pts.size() && i < pts3.size(); ++i) cv::circle(img, pts[i], 3, depthColor(pts3[i].z), -1);
  if (!r.ok) cv::rectangle(img, cv::Rect(0, 0, kImgW, kImgH), r.rejected ? cv::Scalar(0, 200, 255) : cv::Scalar(0, 0, 255), 6);
  img.copyTo(canvas(cv::Rect(0, kBarH, kImgW, kImgH)));

  // 右：俯视轨迹，上 = 前（x），左 = 左（y）。比例自适应，至少显示 ±1 m
  cv::Mat map = canvas(cv::Rect(kImgW, kBarH, kMapW, kImgH));
  double span = 1.0;
  cv::Point2d c(0, 0);
  if (!track_.empty()) {
    double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
    for (const auto& p : track_) {
      xmin = std::min(xmin, p.x), xmax = std::max(xmax, p.x), ymin = std::min(ymin, p.y), ymax = std::max(ymax, p.y);
    }
    c = cv::Point2d((xmin + xmax) / 2, (ymin + ymax) / 2);
    span = std::max({1.0, (xmax - xmin) / 2 + 0.3, (ymax - ymin) / 2 + 0.3});
  }
  const double s = (kImgH / 2 - 10) / span;  // 像素/米
  auto toPx = [&](const cv::Point2d& p) {
    return cv::Point(static_cast<int>(kMapW / 2 - (p.y - c.y) * s), static_cast<int>(kImgH / 2 - (p.x - c.x) * s));
  };
  for (int k = static_cast<int>(std::floor(c.x - span)) - 1; k <= c.x + span + 1; ++k) {  // 1 m 网格
    cv::line(map, toPx({double(k), c.y - 2 * span}), toPx({double(k), c.y + 2 * span}), cv::Scalar(55, 55, 55), 1);
  }
  for (int k = static_cast<int>(std::floor(c.y - span)) - 1; k <= c.y + span + 1; ++k) {
    cv::line(map, toPx({c.x - 2 * span, double(k)}), toPx({c.x + 2 * span, double(k)}), cv::Scalar(55, 55, 55), 1);
  }
  for (size_t i = 1; i < track_.size(); ++i) cv::line(map, toPx(track_[i - 1]), toPx(track_[i]), cv::Scalar(0, 220, 0), 2);
  for (const auto& p : lost_) cv::drawMarker(map, toPx(p), cv::Scalar(0, 0, 255), cv::MARKER_TILTED_CROSS, 14, 2);
  cv::circle(map, toPx({0, 0}), 5, cv::Scalar(255, 255, 255), 2);  // 起点
  const cv::Point2d here(pos_flu[0], pos_flu[1]);
  const double yr = yaw_deg * CV_PI / 180.0;
  cv::arrowedLine(map, toPx(here), toPx(here + cv::Point2d(std::cos(yr), std::sin(yr)) * (0.15 * span)),
                  cv::Scalar(0, 200, 255), 2, cv::LINE_AA, 0, 0.3);
  char scale[48];
  std::snprintf(scale, sizeof(scale), "grid 1 m, view %.1f m", 2 * span);
  cv::putText(map, scale, cv::Point(8, kImgH - 10), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(160, 160, 160), 1);

  // 顶部状态栏（OpenCV putText 不支持中文，用英文）
  char bar[200];
  std::snprintf(bar, sizeof(bar),
                "%s  %.0f fps %.1f ms | inliers %d tracked %d | resets %d | fwd %+.2f left %+.2f up %+.2f m yaw %+.0f",
                r.ok ? "OK" : (r.rejected ? "REJECT" : "LOST"), fps, r.ms, r.inliers, r.tracked, r.reset_count, pos_flu[0],
                pos_flu[1], pos_flu[2], yaw_deg);
  cv::putText(canvas, bar, cv::Point(8, 26), cv::FONT_HERSHEY_SIMPLEX, 0.55,
              r.ok ? cv::Scalar(0, 230, 0) : cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

  return canvas;
}
