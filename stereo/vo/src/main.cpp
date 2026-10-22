// stereo_vo：轻量双目 VO 主程序（无 ROS）。
//
//   实时：  stereo_vo --calib stereo/calib/1280x480/stereo.yml [--csv out.csv] [--seconds 30]
//   录制：  stereo_vo --record logs/stereo_seq/xxx --seconds 20        （只存图，不跑 VO，PGM + times.txt）
//   回放：  stereo_vo --replay logs/stereo_seq/xxx [--skip 3] [--csv out.csv]   （skip 3 = 100 fps 序列按 33 fps 处理）
//   其他：  --device /dev/video4  --swap-lr 0|1  --exposure N（0 自动）  --gain N（-1 不改）
//           --viz 1  预览窗口（左目+跟踪点按深度着色、俯视轨迹；回放时按录制时间实时播放；q 退出）
//           --viz-scale 0.8  预览窗口初始缩放（窗口可拖拽缩放、可最大化）
//
// 输出位姿换算到 FLU（x 前 y 左 z 上），世界系原点 = 第一个关键帧时的左目光心。Ctrl+C 结束并打印汇总。
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <linux/videodev2.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

#include "calib.hpp"
#include "v4l2_stereo.hpp"
#include "viz.hpp"
#include "vo.hpp"

namespace {

std::atomic<bool> g_stop{false};
void onSignal(int) { g_stop = true; }

struct Args {
  std::string calib = "stereo/calib/1280x480/stereo.yml";
  std::string device = "/dev/video4";
  std::string record, replay, csv;
  bool swap_lr = false, viz = false;
  int exposure = 0, gain = -1, skip = 1;
  double seconds = 0, viz_scale = 0.8;
};

Args parseArgs(int argc, char** argv) {
  Args a;
  std::map<std::string, std::string> kv;
  for (int i = 1; i + 1 < argc; i += 2) kv[argv[i]] = argv[i + 1];
  if (argc % 2 == 0) throw std::runtime_error("参数要成对出现：--名字 值");
  for (const auto& [k, v] : kv) {
    if (k == "--calib") a.calib = v;
    else if (k == "--device") a.device = v;
    else if (k == "--record") a.record = v;
    else if (k == "--replay") a.replay = v;
    else if (k == "--csv") a.csv = v;
    else if (k == "--swap-lr") a.swap_lr = v == "1";
    else if (k == "--exposure") a.exposure = std::stoi(v);
    else if (k == "--gain") a.gain = std::stoi(v);
    else if (k == "--skip") a.skip = std::max(1, std::stoi(v));
    else if (k == "--seconds") a.seconds = std::stod(v);
    else if (k == "--viz") a.viz = v == "1";
    else if (k == "--viz-scale") a.viz_scale = std::stod(v);
    else throw std::runtime_error("未知参数 " + k);
  }
  return a;
}

// 光学系（x 右 y 下 z 前）-> FLU（x 前 y 左 z 上）：p_flu = C p_opt
const cv::Matx33d kC(0, 0, 1, -1, 0, 0, 0, -1, 0);

struct PoseFlu {
  cv::Vec3d p;
  cv::Vec4d q;  // w x y z
  double yaw_deg;
};

PoseFlu toFlu(const cv::Matx44d& T) {
  cv::Matx33d R(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
  const cv::Matx33d Rf = kC * R * kC.t();
  PoseFlu out;
  out.p = kC * cv::Vec3d(T(0, 3), T(1, 3), T(2, 3));
  cv::Vec3d rv;
  cv::Rodrigues(Rf, rv);
  const double th = cv::norm(rv);
  const cv::Vec3d ax = th > 1e-12 ? rv / th : cv::Vec3d(1, 0, 0);
  out.q = cv::Vec4d(std::cos(th / 2), ax[0] * std::sin(th / 2), ax[1] * std::sin(th / 2), ax[2] * std::sin(th / 2));
  out.yaw_deg = std::atan2(Rf(1, 0), Rf(0, 0)) * 180.0 / CV_PI;
  return out;
}

struct Stats {
  int frames = 0, ok = 0, keyframes = 0, skipped = 0, resets = 0, lost = 0, relocalized = 0;
  std::vector<double> ms;
  std::vector<int> inliers;
  bool has_start = false;
  cv::Vec3d start, last;
  double path = 0;

  void add(const VoResult& r, const PoseFlu& f, int skip) {
    ++frames;
    skipped += skip;
    ms.push_back(r.ms);
    keyframes += r.new_keyframe;
    resets = r.reset_count;
    lost = r.lost_count;
    relocalized += r.relocalized;
    if (!r.ok) return;
    ++ok;
    inliers.push_back(r.inliers);
    if (!has_start) {
      start = last = f.p;
      has_start = true;
    }
    path += cv::norm(f.p - last);
    last = f.p;
  }
};

template <typename T>
double median(std::vector<T> v) {
  if (v.empty()) return 0;
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

void printSummary(const Stats& s, double wall_s) {
  const double closure = s.has_start ? cv::norm(s.last - s.start) : 0;
  std::printf("==== 汇总 ====\n");
  std::printf("处理 %d 帧（%.1f Hz），有效 %d，跟丢 %d 次（重定位接回 %d，位姿重置 %d），关键帧 %d，相机侧丢帧 %d\n", s.frames,
              wall_s > 0 ? s.frames / wall_s : 0.0, s.ok, s.lost, s.relocalized, s.resets, s.keyframes, s.skipped);
  std::printf("耗时中位 %.2f ms，最大 %.2f ms；内点中位 %.0f\n", median(s.ms),
              s.ms.empty() ? 0.0 : *std::max_element(s.ms.begin(), s.ms.end()), median(s.inliers));
  std::printf("路程 %.3f m，首尾距离 %.3f m（%.2f%% 路程）\n", s.path, closure,
              s.path > 0 ? 100 * closure / s.path : 0.0);
}

int record(const Args& a) {
  std::filesystem::create_directories(a.record);
  V4l2Stereo cam;
  std::string err;
  if (!cam.open(a.device, 1280, 480, 100, err)) throw std::runtime_error(err);
  if (a.exposure > 0) {
    cam.setControl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
    cam.setControl(V4L2_CID_EXPOSURE_ABSOLUTE, a.exposure);
  }
  if (a.gain >= 0) cam.setControl(V4L2_CID_GAIN, a.gain);
  std::ofstream times(a.record + "/times.txt");
  if (!times) throw std::runtime_error("写不了 " + a.record + "/times.txt");
  cv::Mat l, r;
  double t, t0 = -1;
  int skipped, n = 0, dropped = 0;
  while (!g_stop) {
    if (!cam.grab(l, r, t, skipped, a.swap_lr, err)) throw std::runtime_error(err);
    if (t0 < 0) t0 = t;
    if (a.seconds > 0 && t - t0 > a.seconds) break;
    char name[32];
    std::snprintf(name, sizeof(name), "/%06d", n);
    // imwrite 失败不抛异常，必须查返回值（曾因父目录不存在静默丢光所有帧）
    if (!cv::imwrite(a.record + name + "_L.pgm", l) || !cv::imwrite(a.record + name + "_R.pgm", r)) {
      throw std::runtime_error(std::string("写图失败 ") + a.record + name);
    }
    times << n << ' ' << std::fixed << t << '\n';
    dropped += skipped;
    ++n;
  }
  std::printf("录了 %d 对，写盘期间相机侧丢帧 %d，存到 %s\n", n, dropped, a.record.c_str());
  return 0;
}

int run(const Args& a) {
  const auto calib = StereoCalib::load(a.calib);
  StereoVO vo(calib);
  std::ofstream csv;
  if (!a.csv.empty()) {
    csv.open(a.csv);
    csv << "t,ok,x,y,z,qw,qx,qy,qz,inliers,tracked,new_kf,reset_count,ms,lost_count,relocalized\n";
  }
  Stats st;
  V4l2Stereo cam;
  std::vector<std::pair<int, double>> seq;  // 回放：序号, 时间
  size_t seq_i = 0;
  std::string err;
  if (a.replay.empty()) {
    if (!cam.open(a.device, calib.size.width * 2, calib.size.height, 100, err)) throw std::runtime_error(err);
    if (a.exposure > 0) {
      cam.setControl(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
      cam.setControl(V4L2_CID_EXPOSURE_ABSOLUTE, a.exposure);
    }
    if (a.gain >= 0) cam.setControl(V4L2_CID_GAIN, a.gain);
  } else {
    std::ifstream f(a.replay + "/times.txt");
    int i;
    double t;
    while (f >> i >> t) seq.emplace_back(i, t);
    if (seq.empty()) throw std::runtime_error("回放目录没有 times.txt 或为空：" + a.replay);
  }

  const auto wall0 = std::chrono::steady_clock::now();
  double t0 = -1, last_print = 0, last_viz = 0, viz_fps = 0;
  VoViz viz(a.viz_scale);
  cv::Mat l, r;
  while (!g_stop) {
    double t;
    int skipped = 0;
    if (a.replay.empty()) {
      if (!cam.grab(l, r, t, skipped, a.swap_lr, err)) throw std::runtime_error(err);
    } else {
      if (seq_i >= seq.size()) break;
      char name[32];
      std::snprintf(name, sizeof(name), "/%06d", seq[seq_i].first);
      l = cv::imread(a.replay + name + "_L.pgm", cv::IMREAD_GRAYSCALE);
      r = cv::imread(a.replay + name + "_R.pgm", cv::IMREAD_GRAYSCALE);
      t = seq[seq_i].second;
      seq_i += a.skip;
      if (l.empty() || r.empty()) throw std::runtime_error(std::string("读不到回放帧 ") + name);
    }
    if (l.size() != calib.size) throw std::runtime_error("图像尺寸与标定不符");
    if (t0 < 0) t0 = t;
    if (a.seconds > 0 && t - t0 > a.seconds) break;

    const auto res = vo.process(l, r, t);
    const auto f = toFlu(res.T_w_c);
    st.add(res, f, skipped);
    if (csv.is_open()) {
      csv << std::fixed << t << ',' << res.ok << ',' << f.p[0] << ',' << f.p[1] << ',' << f.p[2] << ',' << f.q[0] << ','
          << f.q[1] << ',' << f.q[2] << ',' << f.q[3] << ',' << res.inliers << ',' << res.tracked << ','
          << res.new_keyframe << ',' << res.reset_count << ',' << res.ms << ',' << res.lost_count << ',' << res.relocalized << '\n';
    }
    if (a.viz) {
      const double now = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
      if (!a.replay.empty()) {
        // 回放按录制时间戳实时播放，看起来和现场一致
        const double ahead = (t - t0) - now;
        if (ahead > 0) std::this_thread::sleep_for(std::chrono::duration<double>(ahead));
      }
      const double dt = now - last_viz;
      viz_fps = dt > 0 ? 0.9 * viz_fps + 0.1 / dt : viz_fps;
      last_viz = now;
      if (!viz.show(l, vo, res, f.p, f.yaw_deg, viz_fps)) break;
    }
    if (t - last_print > 2.0) {
      std::printf("[%5d 帧] %s 内点 %3d 跟踪 %3d %.1f ms | 位置 前%+.3f 左%+.3f 上%+.3f m 偏航 %+.1f° | 路程 %.2f m 重置 %d\n",
                  st.frames, res.ok ? "OK  " : "LOST", res.inliers, res.tracked, res.ms, f.p[0], f.p[1], f.p[2],
                  f.yaw_deg, st.path, res.reset_count);
      std::fflush(stdout);
      last_print = t;
    }
  }
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
  printSummary(st, wall);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  try {
    const Args a = parseArgs(argc, argv);
    return a.record.empty() ? run(a) : record(a);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "错误：%s\n", e.what());
    return 1;
  }
}
