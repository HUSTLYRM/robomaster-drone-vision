// 轻量双目视觉里程计：关键帧 + KLT 跟踪 + PnP，不建图、不做 BA，每帧计算量固定。
//
// 流程：
//   建关键帧：左目 GFTT 补足特征 -> 左右 KLT（带视差初值）-> 只校正特征点 -> 行差/视差筛选 -> 三角化出关键帧系 3D 点
//   每帧：  恒速运动预测当前位姿 -> 投影关键帧点作 KLT 初值 -> 上一帧到当前帧左目 KLT（前后向一致性检查）
//          -> 关键帧 3D 点对当前帧 2D 点做 PnP RANSAC + LM 精化 -> 合理性检查（深度为正、帧间运动不超限）
//   内点少于阈值就以当前帧换关键帧；PnP 失败算丢失：存下最近关键帧作快照，在当前帧重建关键帧，
//   再用 ORB 把当前帧和快照匹配做 PnP 重定位接回轨迹；接不回才从丢失前位姿重新起算（reset_count+1）
//
// 坐标：世界系 = 第一个关键帧的「校正后左目」光学系（x 右 y 下 z 前）。转机体系由调用方处理。
#pragma once

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

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
  // 恒速运动预测：用最近两个有效位姿外推当前帧，作 KLT 初值和 PnP 初值。快速转动时帧间位移超出 KLT 金字塔范围，
  // 不预测就跟丢（快速数据集 33 Hz 回放丢 33 次）；预测不准（急停）时退回零运动再跟一次，取存活多的
  bool use_motion_prior = true;
  // 丢失后重定位：重建关键帧时用 ORB 把当前帧与丢失前最后一个关键帧匹配，PnP 求回世界位姿，轨迹不断。
  // 晃动造成的丢失一般零点几秒、前后画面重叠大；不接回则丢失期间的运动全丢（快速数据集 12 次重置累计首尾 50～87 cm）
  bool relocalize = true;
  double reloc_max_age = 2.0;        // 快照最长有效期（秒，无时间戳时按帧数）
  // 快速段的关键帧在模糊画面上建成、只有 30～60 点，20 内点的门槛接不回任何一次；12 / 3 px 在快速数据集接回 6～8 / 12，
  // 慢速闭环不受影响（离线回放 2026-10-23）。错配靠深度为正 + 运动上限兜底
  int reloc_min_inliers = 12;
  double reloc_reproj = 3.0;         // 重定位 PnP RANSAC 重投影阈值（像素）
  double reloc_max_translation = 1.0;  // 相对丢失前位姿的平移上限（米），超了视为错配
  double reloc_max_rotation_deg = 90;
};

struct VoResult {
  bool ok = false;                 // 本帧位姿有效
  bool rejected = false;           // PnP 有解但不合理，本帧丢弃（不算丢失）
  bool new_keyframe = false;
  bool relocalized = false;        // 本帧丢失后重定位成功，位姿与丢失前连续
  int lost_count = 0;              // 跟丢累计次数（含已重定位接回的）
  int reset_count = 0;             // 位姿不连续累计次数（丢失且没能重定位）：变化时下游要按重置处理
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

  // left/right：原始（未校正）8 位灰度图；t：帧时间戳（秒），用于运动预测按真实间隔外推（丢帧时也对）；
  // 不给（< 0）时按帧序号当时间
  VoResult process(const cv::Mat& left, const cv::Mat& right, double t = -1.0);

  // 当前跟踪中的关键帧 3D 点（关键帧系），与 trackedPoints() 同序；单测和预览用
  const std::vector<cv::Point3f>& keyframePoints() const { return pts3_; }
  // 这些点在最近一帧左目原始图上的位置（预览画点用）
  const std::vector<cv::Point2f>& trackedPoints() const { return pts_; }

 private:
  bool makeKeyframe(const cv::Mat& left, const cv::Mat& right, const std::vector<cv::Point2f>& seeds);
  // 当前帧对丢失快照做 ORB 匹配 + PnP，成功则改写 T_w_c_
  bool relocalize(const cv::Mat& left, double now);
  // 丢失后重建关键帧成功时调用：先试重定位，失败才记一次重置
  void afterReinit(const cv::Mat& left, double now, VoResult& r);

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
  // 运动预测：vel_ = 最近两个有效帧间的相机增量 T_cprev_ccur，对应时长 vel_dt_
  bool has_vel_ = false;
  cv::Matx44d vel_ = cv::Matx44d::eye();
  double vel_dt_ = 0.0;
  double t_valid_ = -1.0;          // 最近一次有效位姿的时间
  long frame_idx_ = 0;
  // 关键帧快照（建关键帧时保存）与丢失快照（丢失时由关键帧快照转成世界坐标）
  cv::Mat kf_left_;
  std::vector<cv::Point2f> kf_pts_;
  std::vector<cv::Point3f> kf_pts3_;
  struct LostSnapshot {
    bool valid = false;
    cv::Mat image;
    std::vector<cv::Point2f> pts;
    std::vector<cv::Point3d> pts_w;
    cv::Matx44d T_w_c = cv::Matx44d::eye();
    double t = 0;
  } snap_;
  bool pending_reset_ = false;     // 已丢失、尚未重建关键帧
  int lost_count_ = 0;
  cv::Ptr<cv::Feature2D> orb_;
};
