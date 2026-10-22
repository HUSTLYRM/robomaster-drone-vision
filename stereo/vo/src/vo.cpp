#include "vo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace {

cv::Matx44d fromRt(const cv::Mat& rvec, const cv::Mat& tvec) {
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  cv::Matx44d T = cv::Matx44d::eye();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) T(i, j) = R(i, j);
    T(i, 3) = tvec.at<double>(i);
  }
  return T;
}

void toRt(const cv::Matx44d& T, cv::Mat& rvec, cv::Mat& tvec) {
  cv::Matx33d R(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
  cv::Rodrigues(R, rvec);
  tvec = (cv::Mat_<double>(3, 1) << T(0, 3), T(1, 3), T(2, 3));
}

cv::Matx44d inverse(const cv::Matx44d& T) {
  cv::Matx44d Ti = cv::Matx44d::eye();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) Ti(i, j) = T(j, i);
  }
  for (int i = 0; i < 3; ++i) Ti(i, 3) = -(Ti(i, 0) * T(0, 3) + Ti(i, 1) * T(1, 3) + Ti(i, 2) * T(2, 3));
  return Ti;
}

cv::Point3d transformPoint(const cv::Matx44d& T, const cv::Point3f& P) {
  return {T(0, 0) * P.x + T(0, 1) * P.y + T(0, 2) * P.z + T(0, 3), T(1, 0) * P.x + T(1, 1) * P.y + T(1, 2) * P.z + T(1, 3),
          T(2, 0) * P.x + T(2, 1) * P.y + T(2, 2) * P.z + T(2, 3)};
}

// 把增量运动按比例缩放（旋转按轴角缩放），用于按真实时间间隔外推恒速运动
cv::Matx44d scaleMotion(const cv::Matx44d& T, double k) {
  cv::Mat rvec, tvec;
  toRt(T, rvec, tvec);
  return fromRt(rvec * k, tvec * k);
}

bool inside(const cv::Point2f& p, const cv::Size& s, float margin) {
  return p.x >= margin && p.y >= margin && p.x < s.width - margin && p.y < s.height - margin;
}

// 双向 KLT：from -> to 再回到 from，往返误差超限或出界的点 ok=false；err 为正向匹配残差（窗口平均灰度差）
void trackBidirectional(const cv::Mat& from, const cv::Mat& to, const std::vector<cv::Point2f>& pts,
                        std::vector<cv::Point2f>& out, std::vector<uchar>& ok, std::vector<float>& err,
                        const VoParams& p, bool use_initial) {
  const cv::Size win(p.klt_win, p.klt_win);
  const cv::TermCriteria crit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
  std::vector<uchar> st, st_back;
  std::vector<float> err_back;
  cv::calcOpticalFlowPyrLK(from, to, pts, out, st, err, win, p.klt_levels, crit,
                           use_initial ? cv::OPTFLOW_USE_INITIAL_FLOW : 0);
  std::vector<cv::Point2f> back = pts;
  cv::calcOpticalFlowPyrLK(to, from, out, back, st_back, err_back, win, p.klt_levels, crit,
                           cv::OPTFLOW_USE_INITIAL_FLOW);
  ok.assign(pts.size(), 0);
  for (size_t i = 0; i < pts.size(); ++i) {
    ok[i] = st[i] && st_back[i] && cv::norm(back[i] - pts[i]) < p.fb_max_err && inside(out[i], to.size(), 2.f);
  }
}

// 自适应残差阈值：max(下限, 倍数 × 通过点的残差中位数)
float residualThreshold(const std::vector<float>& err, const std::vector<uchar>& ok, const VoParams& p) {
  std::vector<float> v;
  for (size_t i = 0; i < err.size(); ++i) {
    if (ok[i]) v.push_back(err[i]);
  }
  if (v.empty()) return 0.f;
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return std::max(static_cast<float>(p.stereo_err_floor), static_cast<float>(p.stereo_err_ratio) * v[v.size() / 2]);
}

}  // namespace

bool poseIsPlausible(const cv::Matx44d& T_c_kf, const cv::Matx44d& T_kf_c_prev, const std::vector<cv::Point3f>& pts_kf,
                     const VoParams& p) {
  for (const auto& P : pts_kf) {
    const double z = T_c_kf(2, 0) * P.x + T_c_kf(2, 1) * P.y + T_c_kf(2, 2) * P.z + T_c_kf(2, 3);
    if (z <= 0) return false;
  }
  // 相对上一次有效位姿的增量：T_prev_new = T_c_kf_prev * T_kf_c_new
  const cv::Matx44d step = inverse(T_kf_c_prev) * inverse(T_c_kf);
  const double trans = std::sqrt(step(0, 3) * step(0, 3) + step(1, 3) * step(1, 3) + step(2, 3) * step(2, 3));
  const double cos_angle = std::clamp((step(0, 0) + step(1, 1) + step(2, 2) - 1.0) / 2.0, -1.0, 1.0);
  const double angle_deg = std::acos(cos_angle) * 180.0 / CV_PI;
  return trans <= p.max_step_translation && angle_deg <= p.max_step_rotation_deg;
}

StereoVO::StereoVO(const StereoCalib& calib, const VoParams& params)
    : calib_(calib), p_(params), K_(calib.rectK()), orb_(cv::ORB::create()) {}

bool StereoVO::makeKeyframe(const cv::Mat& left, const cv::Mat& right, const std::vector<cv::Point2f>& seeds) {
  std::vector<cv::Point2f> pts = seeds;
  const int need = p_.max_features - static_cast<int>(pts.size());
  if (need > 0) {
    // 已有点周围不再检测，避免扎堆
    cv::Mat mask(left.size(), CV_8UC1, cv::Scalar(255));
    for (const auto& s : seeds) cv::circle(mask, s, p_.min_distance, cv::Scalar(0), -1);
    std::vector<cv::Point2f> fresh;
    cv::goodFeaturesToTrack(left, fresh, need, p_.gftt_quality, p_.min_distance, mask, 3);
    pts.insert(pts.end(), fresh.begin(), fresh.end());
  }
  if (static_cast<int>(pts.size()) < p_.min_kf_points) return false;

  std::vector<cv::Point2f> lrect;
  calib_.rectifyLeft(pts, lrect);
  std::vector<cv::Point2f> rraw;
  calib_.guessRightRaw(lrect, p_.init_disparity, rraw);
  std::vector<uchar> ok;
  std::vector<float> err;
  trackBidirectional(left, right, pts, rraw, ok, err, p_, true);
  // 遮挡边界的点左右看到的不是同一处，前后向检查拦不住但残差大，深度可错 30%+（合成测试实测），剔掉
  const float err_thr = residualThreshold(err, ok, p_);
  for (size_t i = 0; i < ok.size(); ++i) ok[i] = ok[i] && err[i] <= err_thr;
  std::vector<cv::Point2f> rrect;
  calib_.rectifyRight(rraw, rrect);

  const double fx = calib_.fx(), fy = calib_.fy(), cx = calib_.cx(), cy = calib_.cy(), b = calib_.baseline();
  std::vector<cv::Point2f> kept;
  std::vector<cv::Point3f> pts3;
  for (size_t i = 0; i < pts.size(); ++i) {
    if (!ok[i]) continue;
    const double d = lrect[i].x - rrect[i].x;
    if (std::abs(lrect[i].y - rrect[i].y) > p_.max_row_diff || d < p_.min_disparity || d > p_.max_disparity) continue;
    const double z = fx * b / d;
    kept.push_back(pts[i]);
    pts3.emplace_back(static_cast<float>((lrect[i].x - cx) * z / fx), static_cast<float>((lrect[i].y - cy) * z / fy),
                      static_cast<float>(z));
  }
  if (static_cast<int>(kept.size()) < p_.min_kf_points) return false;

  pts_ = std::move(kept);
  pts3_ = std::move(pts3);
  kf_initial_points_ = static_cast<int>(pts_.size());
  prev_left_ = left.clone();
  kf_left_ = left.clone();  // 必须单独一份：prev_left_ 每帧 copyTo 原地覆盖，共享像素会被改写
  kf_pts_ = pts_;
  kf_pts3_ = pts3_;
  T_w_kf_ = T_w_c_;
  T_kf_c_ = cv::Matx44d::eye();
  has_kf_ = true;
  return true;
}

bool StereoVO::relocalize(const cv::Mat& left, double now) {
  if (!snap_.valid || now - snap_.t > p_.reloc_max_age || snap_.pts.size() < static_cast<size_t>(p_.reloc_min_inliers)) {
    return false;
  }
  // 两边都用「方向固定为 0」的 ORB 描述子：快照点是外部给的、ORB 不会替它算方向，
  // 若当前帧让 ORB 自己检测（带方向），两边旋转基准不同就匹配不上（合成测试实测）。
  // 短时丢失期间绕光轴转动小，不需要旋转不变
  std::vector<cv::KeyPoint> ks;
  for (size_t i = 0; i < snap_.pts.size(); ++i) ks.emplace_back(snap_.pts[i], 31.f, 0.f, 0.f, 0, static_cast<int>(i));
  cv::Mat ds;
  orb_->compute(snap_.image, ks, ds);  // 贴边的点会被删掉，靠 class_id 找回原序号
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(left, corners, 1000, p_.gftt_quality, 7);
  std::vector<cv::KeyPoint> kc;
  for (const auto& c : corners) kc.emplace_back(c, 31.f, 0.f);
  cv::Mat dc;
  orb_->compute(left, kc, dc);
  if (ds.empty() || dc.empty()) return false;
  std::vector<cv::DMatch> matches;
  cv::BFMatcher(cv::NORM_HAMMING, true).match(ds, dc, matches);
  std::vector<cv::Point3d> obj;
  std::vector<cv::Point2f> raw;
  for (const auto& m : matches) {
    if (m.distance > 64) continue;  // 汉明距离太大的不要
    obj.push_back(snap_.pts_w[ks[m.queryIdx].class_id]);
    raw.push_back(kc[m.trainIdx].pt);
  }
  if (static_cast<int>(obj.size()) < p_.reloc_min_inliers) return false;
  std::vector<cv::Point2f> img;
  calib_.rectifyLeft(raw, img);
  cv::Mat rvec, tvec;
  std::vector<int> inl;
  if (!cv::solvePnPRansac(obj, img, K_, cv::noArray(), rvec, tvec, false, 200, p_.reloc_reproj, 0.99, inl, cv::SOLVEPNP_EPNP) ||
      static_cast<int>(inl.size()) < p_.reloc_min_inliers) {
    return false;
  }
  std::vector<cv::Point3d> obj_in;
  std::vector<cv::Point2f> img_in;
  for (int i : inl) {
    obj_in.push_back(obj[i]);
    img_in.push_back(img[i]);
  }
  cv::solvePnPRefineLM(obj_in, img_in, K_, cv::noArray(), rvec, tvec);
  const cv::Matx44d T_c_w = fromRt(rvec, tvec);
  for (const auto& P : obj_in) {
    if (T_c_w(2, 0) * P.x + T_c_w(2, 1) * P.y + T_c_w(2, 2) * P.z + T_c_w(2, 3) <= 0) return false;
  }
  // 与丢失前位姿相比运动过大，多半是错配
  const cv::Matx44d step = inverse(snap_.T_w_c) * inverse(T_c_w);
  const double trans = std::sqrt(step(0, 3) * step(0, 3) + step(1, 3) * step(1, 3) + step(2, 3) * step(2, 3));
  const double ang = std::acos(std::clamp((step(0, 0) + step(1, 1) + step(2, 2) - 1.0) / 2.0, -1.0, 1.0)) * 180.0 / CV_PI;
  if (trans > p_.reloc_max_translation || ang > p_.reloc_max_rotation_deg) return false;
  T_w_c_ = inverse(T_c_w);
  return true;
}

void StereoVO::afterReinit(const cv::Mat& left, double now, VoResult& r) {
  if (!pending_reset_) return;
  pending_reset_ = false;
  if (p_.relocalize && relocalize(left, now)) {
    T_w_kf_ = T_w_c_;  // 刚建的关键帧就是当前帧
    r.relocalized = true;
  } else {
    ++reset_count_;
  }
  snap_.valid = false;
}

VoResult StereoVO::process(const cv::Mat& left, const cv::Mat& right, double t) {
  const auto t0 = std::chrono::steady_clock::now();
  const double now = t >= 0 ? t : static_cast<double>(frame_idx_);
  ++frame_idx_;
  VoResult r;
  auto finish = [&]() {
    r.reset_count = reset_count_;
    r.lost_count = lost_count_;
    r.keyframe_points = static_cast<int>(pts3_.size());
    r.T_w_c = T_w_c_;
    r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return r;
  };

  if (!has_kf_) {
    // 首帧位姿定义为原点；之后的建关键帧失败（看不到纹理）不改位姿
    r.ok = makeKeyframe(left, right, {});
    r.new_keyframe = r.ok;
    if (r.ok) {
      t_valid_ = now;
      afterReinit(left, now, r);
    }
    return finish();
  }

  // 运动预测：上一有效位姿 × 恒速增量（按时间比例缩放），作 KLT 与 PnP 初值
  cv::Matx44d T_kf_c_guess = T_kf_c_;
  std::vector<cv::Point2f> cur;
  std::vector<uchar> ok;
  std::vector<float> err;
  bool tracked_with_prior = false;
  if (p_.use_motion_prior && has_vel_ && vel_dt_ > 0) {
    T_kf_c_guess = T_kf_c_ * scaleMotion(vel_, std::clamp((now - t_valid_) / vel_dt_, 0.0, 5.0));
    const cv::Matx44d T_c_kf_guess = inverse(T_kf_c_guess);
    std::vector<cv::Point3d> pc;
    pc.reserve(pts3_.size());
    for (const auto& P : pts3_) pc.emplace_back(transformPoint(T_c_kf_guess, P));
    calib_.projectLeftRaw(pc, cur);
    for (size_t i = 0; i < cur.size(); ++i) {
      if (cur[i].x < 0) cur[i] = pts_[i];  // 预测到相机背后的点退回上一帧位置
    }
    trackBidirectional(prev_left_, left, pts_, cur, ok, err, p_, true);
    tracked_with_prior = true;
  }
  const auto alive = [](const std::vector<uchar>& v) { return static_cast<size_t>(std::count(v.begin(), v.end(), 1)); };
  if (!tracked_with_prior || alive(ok) < pts_.size() / 2) {
    // 没有预测，或预测不准（急停/急转向）存活不到一半：按零运动再跟一次，取存活多的
    std::vector<cv::Point2f> cur0;
    std::vector<uchar> ok0;
    std::vector<float> err0;
    trackBidirectional(prev_left_, left, pts_, cur0, ok0, err0, p_, false);
    if (!tracked_with_prior || alive(ok0) > alive(ok)) {
      cur.swap(cur0);
      ok.swap(ok0);
      T_kf_c_guess = T_kf_c_;
    }
  }
  std::vector<cv::Point2f> img;
  std::vector<cv::Point3f> obj;
  std::vector<cv::Point2f> raw_kept;
  for (size_t i = 0; i < pts_.size(); ++i) {
    if (!ok[i]) continue;
    raw_kept.push_back(cur[i]);
    obj.push_back(pts3_[i]);
  }
  r.tracked = static_cast<int>(raw_kept.size());

  bool pnp_ok = false;
  std::vector<int> inl;
  cv::Mat rvec, tvec;
  if (r.tracked >= p_.min_pnp_inliers) {
    calib_.rectifyLeft(raw_kept, img);
    toRt(inverse(T_kf_c_guess), rvec, tvec);  // solvePnP 求的是 T_c_kf
    pnp_ok = cv::solvePnPRansac(obj, img, K_, cv::noArray(), rvec, tvec, true, p_.ransac_iters, p_.ransac_reproj,
                                0.99, inl, cv::SOLVEPNP_ITERATIVE) &&
             static_cast<int>(inl.size()) >= p_.min_pnp_inliers;
  }

  auto lose = [&]() {
    // 丢失：位姿停在上一个有效值，存下最近关键帧作重定位快照，在当前帧重建关键帧；建不成就等下一帧继续试
    ++lost_count_;
    snap_.valid = !kf_left_.empty() && !kf_pts_.empty();
    if (snap_.valid) {
      snap_.image = kf_left_;
      snap_.pts = kf_pts_;
      snap_.pts_w.clear();
      for (const auto& P : kf_pts3_) snap_.pts_w.push_back(transformPoint(T_w_kf_, P));
      snap_.T_w_c = T_w_c_;
      snap_.t = now;
    }
    pending_reset_ = true;
    rejected_in_row_ = 0;
    has_kf_ = false;
    has_vel_ = false;  // 丢失前的速度不可信，重新积累
    pts_.clear();
    pts3_.clear();
    if (makeKeyframe(left, right, {})) {
      r.new_keyframe = true;
      t_valid_ = now;
      afterReinit(left, now, r);
    }
    return finish();
  };
  if (!pnp_ok) return lose();

  std::vector<cv::Point3f> obj_in;
  std::vector<cv::Point2f> img_in, raw_in;
  for (int i : inl) {
    obj_in.push_back(obj[i]);
    img_in.push_back(img[i]);
    raw_in.push_back(raw_kept[i]);
  }
  cv::solvePnPRefineLM(obj_in, img_in, K_, cv::noArray(), rvec, tvec);
  const cv::Matx44d T_c_kf = fromRt(rvec, tvec);
  if (!poseIsPlausible(T_c_kf, T_kf_c_, obj_in, p_)) {
    // 丢弃本帧，状态全不动（上一帧图像和点留着，下一帧照常从它跟踪）
    r.rejected = true;
    if (++rejected_in_row_ > p_.max_rejected) return lose();
    return finish();
  }
  rejected_in_row_ = 0;
  const cv::Matx44d T_kf_c_new = inverse(T_c_kf);
  if (now > t_valid_) {
    vel_ = inverse(T_kf_c_) * T_kf_c_new;  // 相机系增量 T_cprev_ccur（两者同一关键帧系）
    vel_dt_ = now - t_valid_;
    has_vel_ = true;
  }
  t_valid_ = now;
  T_kf_c_ = T_kf_c_new;
  T_w_c_ = T_w_kf_ * T_kf_c_;
  r.ok = true;
  r.inliers = static_cast<int>(inl.size());

  pts_ = std::move(raw_in);
  pts3_ = std::move(obj_in);
  left.copyTo(prev_left_);

  if (r.inliers < p_.kf_min_inliers || r.inliers < p_.kf_ratio * kf_initial_points_) {
    // 用仍在跟踪的点做种子换关键帧（重新三角化到新关键帧系）；makeKeyframe 失败不改状态，继续用旧关键帧
    r.new_keyframe = makeKeyframe(left, right, std::vector<cv::Point2f>(pts_));
  }
  return finish();
}
