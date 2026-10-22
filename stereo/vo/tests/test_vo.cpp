// VO 单测：用纹理平面合成已知轨迹的双目图，检验三角化深度、轨迹精度和丢失恢复。
#include <cmath>
#include <set>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "calib.hpp"
#include "vo.hpp"

namespace {

constexpr double kF = 500.0, kCx = 320.0, kCy = 240.0, kBaseline = 0.086;
const cv::Size kSize(640, 480);

// 已校正、无畸变的理想双目（只测 VO 核心，校正交给 OpenCV）
StereoCalib idealCalib() {
  StereoCalib c;
  c.size = kSize;
  c.K1 = (cv::Mat_<double>(3, 3) << kF, 0, kCx, 0, kF, kCy, 0, 0, 1);
  c.K2 = c.K1.clone();
  c.D1 = cv::Mat::zeros(1, 5, CV_64F);
  c.D2 = cv::Mat::zeros(1, 5, CV_64F);
  c.R1 = cv::Mat::eye(3, 3, CV_64F);
  c.R2 = cv::Mat::eye(3, 3, CV_64F);
  c.P1 = (cv::Mat_<double>(3, 4) << kF, 0, kCx, 0, 0, kF, kCy, 0, 0, 0, 1, 0);
  c.P2 = (cv::Mat_<double>(3, 4) << kF, 0, kCx, -kF * kBaseline, 0, kF, kCy, 0, 0, 0, 1, 0);
  return c;
}

struct Plane {
  cv::Vec3d origin, e1, e2;  // 世界系：平面上 (u,v) 米 -> origin + u*e1 + v*e2
  double meters_per_texel;
  cv::Mat texture;
};

// 小随机图放大成平滑斑块纹理，角点丰富且不混叠
cv::Mat makeTexture(int w, int h, int blob, uint64 seed) {
  cv::RNG rng(seed);
  cv::Mat small(h / blob, w / blob, CV_8UC1);
  rng.fill(small, cv::RNG::UNIFORM, 0, 256);
  cv::Mat tex;
  cv::resize(small, tex, cv::Size(w, h), 0, 0, cv::INTER_CUBIC);
  return tex;
}

std::vector<Plane> makeScene() {
  // 远墙 z=6 m（12x8 m），近板 z=2.5 m（2x1.6 m，挡住画面一部分）。
  // 纹素投影到图像上必须 > 1 px：曾用 1 cm 纹素（6 m 处 0.83 px）缩小渲染出混叠，
  // 混叠纹理不随几何一致移动，让 KLT/双目带系统偏差，误判成 VO 漂移
  return {
      {{-6, -4, 6}, {1, 0, 0}, {0, 1, 0}, 0.02, makeTexture(600, 400, 6, 1)},     // 6 m 处 1.67 px/纹素
      {{-1.2, -0.6, 2.5}, {1, 0, 0}, {0, 1, 0}, 0.008, makeTexture(250, 200, 6, 2)},  // 2.5 m 处 1.6 px/纹素
  };
}

// 相机（光学系）位姿 T_w_c 下渲染一目图像；后画的平面遮挡先画的（近板在后）
cv::Mat render(const std::vector<Plane>& scene, const cv::Matx44d& T_w_c) {
  cv::Matx33d Rwc(T_w_c(0, 0), T_w_c(0, 1), T_w_c(0, 2), T_w_c(1, 0), T_w_c(1, 1), T_w_c(1, 2), T_w_c(2, 0),
                  T_w_c(2, 1), T_w_c(2, 2));
  cv::Vec3d twc(T_w_c(0, 3), T_w_c(1, 3), T_w_c(2, 3));
  cv::Matx33d Rcw = Rwc.t();
  cv::Vec3d tcw = -(Rcw * twc);
  cv::Matx33d K(kF, 0, kCx, 0, kF, kCy, 0, 0, 1);
  cv::Mat img(kSize, CV_8UC1, cv::Scalar(0));
  for (const auto& pl : scene) {
    cv::Vec3d c1 = Rcw * pl.e1 * pl.meters_per_texel, c2 = Rcw * pl.e2 * pl.meters_per_texel,
              c3 = Rcw * pl.origin + tcw;
    cv::Matx33d H = K * cv::Matx33d(c1[0], c2[0], c3[0], c1[1], c2[1], c3[1], c1[2], c2[2], c3[2]);
    cv::Mat warped, mask;
    cv::warpPerspective(pl.texture, warped, cv::Mat(H), kSize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    cv::warpPerspective(cv::Mat(pl.texture.size(), CV_8UC1, cv::Scalar(255)), mask, cv::Mat(H), kSize,
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    warped.copyTo(img, mask);
  }
  return img;
}

cv::Matx44d poseAt(int i) {
  // 每帧前进 1.2 cm、右移 1 cm、下移 0.2 cm，偏航 0.3°、俯仰 0.1°。
  // 必须两个轴都转：只绕一个轴时旋转可交换，「位姿乘反」这类错误测不出来（变异测试实测漏过）
  const double yaw = i * 0.3 * CV_PI / 180.0, pitch = i * 0.1 * CV_PI / 180.0;
  const cv::Matx33d Ry(std::cos(yaw), 0, std::sin(yaw), 0, 1, 0, -std::sin(yaw), 0, std::cos(yaw));
  const cv::Matx33d Rx(1, 0, 0, 0, std::cos(pitch), -std::sin(pitch), 0, std::sin(pitch), std::cos(pitch));
  const cv::Matx33d R = Ry * Rx;
  cv::Matx44d T = cv::Matx44d::eye();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) T(r, c) = R(r, c);
  }
  T(0, 3) = 0.010 * i;
  T(1, 3) = 0.002 * i;
  T(2, 3) = 0.012 * i;
  return T;
}

cv::Matx44d rightOf(const cv::Matx44d& T_w_l) {
  cv::Matx44d T_l_r = cv::Matx44d::eye();
  T_l_r(0, 3) = kBaseline;  // 右目在左目 +x 方向
  return T_w_l * T_l_r;
}

double rotationErrorDeg(const cv::Matx44d& A, const cv::Matx44d& B) {
  cv::Matx33d Ra(A(0, 0), A(0, 1), A(0, 2), A(1, 0), A(1, 1), A(1, 2), A(2, 0), A(2, 1), A(2, 2));
  cv::Matx33d Rb(B(0, 0), B(0, 1), B(0, 2), B(1, 0), B(1, 1), B(1, 2), B(2, 0), B(2, 1), B(2, 2));
  cv::Vec3d rv;
  cv::Rodrigues(Ra.t() * Rb, rv);
  return cv::norm(rv) * 180.0 / CV_PI;
}

}  // namespace

TEST(StereoVO, KeyframeDepthMatchesScene) {
  const auto scene = makeScene();
  StereoVO vo(idealCalib());
  const auto r = vo.process(render(scene, poseAt(0)), render(scene, rightOf(poseAt(0))));
  ASSERT_TRUE(r.ok);
  ASSERT_GE(r.keyframe_points, 100);
  int good = 0;
  for (const auto& p : vo.keyframePoints()) {
    if (std::abs(p.z - 6.0) < 0.01 * 6.0 || std::abs(p.z - 2.5) < 0.01 * 2.5) ++good;
  }
  // 遮挡边界点被残差过滤剔掉后应全部落在平面上（无过滤时约 12% 是错的，最大误差 33%）
  EXPECT_EQ(good, static_cast<int>(vo.keyframePoints().size()));
}

TEST(StereoVO, TrajectoryAccuracy) {
  const auto scene = makeScene();
  VoParams p;
  // 逼它频繁换关键帧：默认参数下 80 帧只换 1 次，关键帧衔接的错误（如位姿乘反）测不出来。
  // 特征数显式固定，免得默认值变了（200→300）点多到内点不再跌破 160，测试悄悄失效
  p.max_features = 200;
  p.kf_min_inliers = 160;
  StereoVO vo(idealCalib(), p);
  const int n = 80;
  const double step = std::sqrt(0.010 * 0.010 + 0.002 * 0.002 + 0.012 * 0.012);
  VoResult r;
  int keyframes = 0;
  double worst_ratio = 0, worst_rot = 0, last_err = 0;
  int worst_i = 0;
  for (int i = 0; i < n; ++i) {
    const auto gt = poseAt(i);
    r = vo.process(render(scene, gt), render(scene, rightOf(gt)));
    ASSERT_TRUE(r.ok) << "第 " << i << " 帧";
    keyframes += r.new_keyframe;
    if (i < 10) continue;  // 起步路程太短，比例没意义
    const double err =
        cv::norm(cv::Vec3d(r.T_w_c(0, 3) - gt(0, 3), r.T_w_c(1, 3) - gt(1, 3), r.T_w_c(2, 3) - gt(2, 3)));
    if (err / (i * step) > worst_ratio) {
      worst_ratio = err / (i * step);
      worst_i = i;
    }
    worst_rot = std::max(worst_rot, rotationErrorDeg(r.T_w_c, gt));
    last_err = err;
  }
  // 逐帧取最坏值，而不只看终点。实测最坏 0.79%；去掉遮挡点残差过滤时为 2.45%
  EXPECT_LT(worst_ratio, 0.015) << "最坏位置误差 / 已走路程";
  EXPECT_LT(worst_rot, 0.3) << "最坏姿态误差（度）";
  EXPECT_LT(last_err / ((n - 1) * step), 0.012) << "终点误差 / 路程";
  EXPECT_EQ(r.reset_count, 0);
  EXPECT_GE(keyframes, 4);
  std::printf("合成轨迹：关键帧 %d 个，最坏位置误差 %.2f%% 路程（第 %d 帧），终点 %.2f%%，最坏姿态误差 %.3f°\n", keyframes,
              100 * worst_ratio, worst_i, 100 * last_err / ((n - 1) * step), worst_rot);
}

TEST(StereoVO, StaticSparseSceneDoesNotDrift) {
  // 弱纹理：关键帧只有几十个点。静止时必须一直用同一个关键帧、位置不动
  const auto scene = makeScene();
  VoParams p;
  p.max_features = 70;  // 建出约 60 个点，低于旧版写死的 80，复现「每帧换关键帧」
  p.min_kf_points = 30;
  StereoVO vo(idealCalib(), p);
  const cv::Mat l = render(scene, poseAt(0)), r = render(scene, rightOf(poseAt(0)));
  VoResult res;
  int keyframes = 0;
  for (int i = 0; i < 50; ++i) {
    res = vo.process(l, r);
    ASSERT_TRUE(res.ok);
    keyframes += res.new_keyframe;
  }
  EXPECT_EQ(keyframes, 1);
  EXPECT_LT(cv::norm(cv::Vec3d(res.T_w_c(0, 3), res.T_w_c(1, 3), res.T_w_c(2, 3))), 1e-3);
}

TEST(PoseIsPlausible, RejectsBehindCameraAndJumps) {
  VoParams p;
  const std::vector<cv::Point3f> pts{{0.5f, 0.2f, 3.f}, {-1.f, 0.f, 6.f}, {0.f, -0.5f, 2.5f}};
  const cv::Matx44d I = cv::Matx44d::eye();
  EXPECT_TRUE(poseIsPlausible(I, I, pts, p));

  cv::Matx44d small = I;  // 前进 5 cm：合理
  small(2, 3) = -0.05;    // T_c_kf 平移为负 = 相机前移
  EXPECT_TRUE(poseIsPlausible(small, I, pts, p));

  cv::Matx44d behind = I;  // 相机冲到点后面：最近的点深度变负（实测错解的特征）
  behind(2, 3) = -2.6;
  EXPECT_FALSE(poseIsPlausible(behind, I, pts, p));

  cv::Matx44d far = I;  // 横跳 9 m，点仍在前方：靠运动上限拦
  far(0, 3) = 9.0;
  EXPECT_FALSE(poseIsPlausible(far, I, pts, p));

  // 绕 y 轴转 80°（实测错解转了 82°）
  const double a = 80 * CV_PI / 180;
  cv::Matx44d rot = I;
  rot(0, 0) = std::cos(a);
  rot(0, 2) = std::sin(a);
  rot(2, 0) = -std::sin(a);
  rot(2, 2) = std::cos(a);
  const std::vector<cv::Point3f> front{{0.f, 0.f, 0.1f}};
  EXPECT_FALSE(poseIsPlausible(rot, I, front, p));  // 点仍在前方（z = 0.1·cos80° > 0），只靠转角上限拦

  // 连续两次的增量才是判据：上一次已在 far，这次也在 far 附近，是合理的
  EXPECT_TRUE(poseIsPlausible(far, cv::Matx44d(far).inv(), pts, p));
}

TEST(StereoVO, LostThenRecovers) {
  const auto scene = makeScene();
  StereoVO vo(idealCalib());
  const cv::Mat blank(kSize, CV_8UC1, cv::Scalar(128));
  for (int i = 0; i < 3; ++i) ASSERT_TRUE(vo.process(render(scene, poseAt(i)), render(scene, rightOf(poseAt(i)))).ok);
  const auto last_ok = vo.process(render(scene, poseAt(3)), render(scene, rightOf(poseAt(3))));
  ASSERT_TRUE(last_ok.ok);

  const auto lost = vo.process(blank, blank);  // 镜头被挡
  EXPECT_FALSE(lost.ok);
  EXPECT_EQ(lost.reset_count, 1);
  EXPECT_EQ(lost.T_w_c, last_ok.T_w_c);  // 位姿停在最后有效值

  const auto again = vo.process(blank, blank);
  EXPECT_FALSE(again.ok);

  const auto back = vo.process(render(scene, poseAt(4)), render(scene, rightOf(poseAt(4))));
  EXPECT_TRUE(back.ok);  // 重新建关键帧
  EXPECT_TRUE(back.new_keyframe);
  const auto next = vo.process(render(scene, poseAt(5)), render(scene, rightOf(poseAt(5))));
  EXPECT_TRUE(next.ok);
  EXPECT_EQ(next.reset_count, 1);
}

namespace {

std::string writeCalib(const StereoCalib& c, const std::string& name) {
  const std::string path = ::testing::TempDir() + name;
  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  fs << "image_width" << c.size.width << "image_height" << c.size.height << "K1" << c.K1 << "D1" << c.D1 << "R1"
     << c.R1 << "P1" << c.P1 << "K2" << c.K2 << "D2" << c.D2 << "R2" << c.R2 << "P2" << c.P2;
  return path;
}

}  // namespace

TEST(StereoCalib, LoadRoundTrip) {
  const auto c = StereoCalib::load(writeCalib(idealCalib(), "ok.yml"));
  EXPECT_EQ(c.size, kSize);
  EXPECT_NEAR(c.baseline(), kBaseline, 1e-12);
  EXPECT_NEAR(c.fx(), kF, 1e-12);
}

TEST(StereoCalib, LoadRejectsSwappedLeftRight) {
  auto c = idealCalib();
  c.P2.at<double>(0, 3) = kF * kBaseline;  // Tx 为正 = 左右接反
  EXPECT_THROW(StereoCalib::load(writeCalib(c, "swapped.yml")), std::runtime_error);
}
