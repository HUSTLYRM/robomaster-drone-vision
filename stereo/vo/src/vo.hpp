// 轻量双目视觉里程计：关键帧 + KLT 跟踪 + PnP，不建图、不做 BA，每帧计算量固定。
//
// 流程：
//   建关键帧：左目 GFTT 补足特征 -> 左右 KLT（带视差初值）-> 只校正特征点 -> 行差/视差筛选 -> 三角化出关键帧系 3D 点
//   每帧：  上一帧 -> 当前帧左目 KLT（前后向一致性检查）-> 关键帧 3D 点对当前帧 2D 点做 PnP RANSAC + LM 精化
//   内点少于阈值就以当前帧换关键帧；PnP 失败算丢失，在当前帧重建关键帧，位姿从上一个有效值接着走（reset_count+1）
//
// 坐标：世界系 = 第一个关键帧的「校正后左目」光学系（x 右 y 下 z 前）。转机体系由调用方处理。
#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "calib.hpp"

struct VoParams {
  int max_features = 300;          // 关键帧特征目标数（200→300 实测双目 3D 点 98→128；约一半点因右目看不到被双目检查剔掉）
  int min_distance = 15;           // 特征最小间距（像素），也用于保持分布均匀
  double gftt_quality = 0.01;
  int klt_win = 21;
  int klt_levels = 3;
  double fb_max_err = 1.0;         // KLT 前后向误差上限（像素）
  double init_disparity = 20.0;    // 左右 KLT 的视差初值（像素）
  double min_disparity = 2.0;      // 视差下限：太远的点深度不可信
  double max_disparity = 150.0;
  double max_row_diff = 1.0;       // 校正后左右行差上限（像素）
  double stereo_err_ratio = 4.0;   // 左右 KLT 残差 > 倍数 × 中位数 且 > 下限 的点剔除（遮挡边界点）
  double stereo_err_floor = 2.0;   // 残差下限（窗口平均灰度差），真机有噪声和左右亮度差，阈值不能写死
  // 关键帧至少要这么多 3D 点才算建成功。手持闭环实测 24 次丢失多数是「该换关键帧时凑不够 40 点、换不成、旧关键帧耗尽」，
  // 降到 30（PnP 至少 15 内点，仍留一倍余量）
  int min_kf_points = 30;
  double ransac_reproj = 1.5;      // PnP RANSAC 重投影阈值（像素）
  int ransac_iters = 100;
  int min_pnp_inliers = 15;        // 少于此值算丢失
  // PnP 结果合理性：RANSAC 只看重投影，会收敛到「部分内点在相机背后」的错解（实测 121 内点、位置跳 9 m、转 82°，下一帧又跳回）。
  // 不合理的帧丢弃（ok=false，不重置关键帧），连续 max_rejected 帧不合理才按丢失处理
  double max_step_translation = 0.5;  // 相邻两次有效位姿间平移上限（米）
  double max_step_rotation_deg = 45;  // 相邻两次有效位姿间转角上限（度）
  int max_rejected = 5;
  // 换关键帧：内点 < 建关键帧时点数 × kf_ratio，或 < kf_min_inliers。
  // 不能只用绝对阈值：弱纹理场景一个关键帧只有 ~50 点，曾用「< 80 就换」导致每帧换关键帧，静止 3 s 漂 15 cm（实测）
  double kf_ratio = 0.6;
  int kf_min_inliers = 30;
};

struct VoResult {
  bool ok = false;                 // 本帧位姿有效
  bool rejected = false;           // PnP 有解但不合理，本帧丢弃（不算丢失）
  bool new_keyframe = false;
  int reset_count = 0;             // 丢失重置累计次数：变化说明位姿出现了不连续
  int tracked = 0;                 // KLT 跟踪存活点数
  int inliers = 0;                 // PnP 内点数
  int keyframe_points = 0;         // 当前关键帧的 3D 点数
  double ms = 0.0;                 // 本帧处理耗时
  cv::Matx44d T_w_c = cv::Matx44d::eye();  // 当前帧（校正后左目）在世界系的位姿
};

// PnP 解是否合理：内点全在相机前方（T_c_kf 下 z > 0），且相对上一次有效位姿的运动不超限。单测直接调用
bool poseIsPlausible(const cv::Matx44d& T_c_kf, const cv::Matx44d& T_kf_c_prev, const std::vector<cv::Point3f>& pts_kf,
                     const VoParams& p);

class StereoVO {
 public:
  StereoVO(const StereoCalib& calib, const VoParams& params = VoParams());

  // left/right：原始（未校正）8 位灰度图
  VoResult process(const cv::Mat& left, const cv::Mat& right);

  // 单测用：当前关键帧的 3D 点（关键帧系）
  const std::vector<cv::Point3f>& keyframePoints() const { return pts3_; }

 private:
  bool makeKeyframe(const cv::Mat& left, const cv::Mat& right, const std::vector<cv::Point2f>& seeds);

  StereoCalib calib_;
  VoParams p_;
  cv::Matx33d K_;                  // 校正后左目内参

  bool has_kf_ = false;
  cv::Mat prev_left_;
  std::vector<cv::Point2f> pts_;   // 关键帧点在上一帧左目原始图上的位置
  std::vector<cv::Point3f> pts3_;  // 同序的关键帧系 3D 点
  int kf_initial_points_ = 0;
  cv::Matx44d T_w_kf_ = cv::Matx44d::eye();
  cv::Matx44d T_kf_c_ = cv::Matx44d::eye();  // 上一帧相对关键帧，用作 PnP 初值
  cv::Matx44d T_w_c_ = cv::Matx44d::eye();   // 最近一次有效位姿
  int reset_count_ = 0;
  int rejected_in_row_ = 0;
};
