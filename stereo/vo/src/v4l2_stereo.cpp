#include "v4l2_stereo.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

namespace {

int xioctl(int fd, unsigned long req, void* arg) {
  int r;
  do {
    r = ioctl(fd, req, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

std::string errnoStr(const char* what) { return std::string(what) + ": " + std::strerror(errno); }

constexpr int kNumBuffers = 4;

}  // namespace

V4l2Stereo::~V4l2Stereo() {
  if (fd_ < 0) return;
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(fd_, VIDIOC_STREAMOFF, &type);
  for (auto& b : buffers_) munmap(b.ptr, b.len);
  close(fd_);
}

bool V4l2Stereo::open(const std::string& dev, int width, int height, int fps, std::string& err) {
  fd_ = ::open(dev.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) {
    err = errnoStr(("打开 " + dev).c_str()) + "（被占用时用 fuser -v " + dev + " 查）";
    return false;
  }
  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
    err = errnoStr("VIDIOC_S_FMT");
    return false;
  }
  if (static_cast<int>(fmt.fmt.pix.width) != width || static_cast<int>(fmt.fmt.pix.height) != height ||
      fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
    err = "驱动没接受 " + std::to_string(width) + "x" + std::to_string(height) + " YUYV";
    return false;
  }
  width_ = width;
  height_ = height;
  bytes_per_line_ = static_cast<int>(fmt.fmt.pix.bytesperline);

  v4l2_streamparm parm{};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = fps;
  xioctl(fd_, VIDIOC_S_PARM, &parm);  // 拼接模式帧率固定，设不上不算错

  v4l2_requestbuffers req{};
  req.count = kNumBuffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
    err = errnoStr("VIDIOC_REQBUFS");
    return false;
  }
  buffers_.resize(req.count);
  for (unsigned i = 0; i < req.count; ++i) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      err = errnoStr("VIDIOC_QUERYBUF");
      return false;
    }
    buffers_[i].len = buf.length;
    buffers_[i].ptr = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
    if (buffers_[i].ptr == MAP_FAILED) {
      err = errnoStr("mmap");
      return false;
    }
    if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      err = errnoStr("VIDIOC_QBUF");
      return false;
    }
  }
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    err = errnoStr("VIDIOC_STREAMON");
    return false;
  }
  return true;
}

bool V4l2Stereo::setControl(uint32_t id, int32_t value) {
  v4l2_control c{};
  c.id = id;
  c.value = value;
  return xioctl(fd_, VIDIOC_S_CTRL, &c) == 0;
}

bool V4l2Stereo::grab(cv::Mat& left, cv::Mat& right, double& t_mono, int& skipped, bool swap,
                      std::string& err) {
  skipped = 0;
  v4l2_buffer latest{};
  bool have = false;
  while (!have) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    timeval tv{2, 0};
    const int r = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (r < 0 && errno == EINTR) continue;
    if (r <= 0) {
      err = r == 0 ? "等帧超时 2 s（设备拔出？）" : errnoStr("select");
      return false;
    }
    // 排空队列：旧帧立即还给驱动，只留最新一帧
    for (;;) {
      v4l2_buffer buf{};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) break;
        err = errnoStr("VIDIOC_DQBUF");
        return false;
      }
      if (have) {
        ++skipped;
        xioctl(fd_, VIDIOC_QBUF, &latest);
      }
      latest = buf;
      have = true;
    }
  }
  if (last_seq_ >= 0 && latest.sequence > last_seq_ + 1) {
    skipped = static_cast<int>(latest.sequence - last_seq_ - 1);
  }
  last_seq_ = latest.sequence;
  t_mono = latest.timestamp.tv_sec + latest.timestamp.tv_usec * 1e-6;

  const int half = width_ / 2;
  left.create(height_, half, CV_8UC1);
  right.create(height_, half, CV_8UC1);
  cv::Mat& first = swap ? right : left;
  cv::Mat& second = swap ? left : right;
  const auto* data = static_cast<const uint8_t*>(buffers_[latest.index].ptr);
  for (int y = 0; y < height_; ++y) {
    const uint8_t* row = data + static_cast<size_t>(y) * bytes_per_line_;
    uint8_t* a = first.ptr<uint8_t>(y);
    uint8_t* b = second.ptr<uint8_t>(y);
    // YUYV：Y 在偶数字节
    for (int x = 0; x < half; ++x) {
      a[x] = row[2 * x];
      b[x] = row[2 * (x + half)];
    }
  }
  xioctl(fd_, VIDIOC_QBUF, &latest);
  return true;
}
