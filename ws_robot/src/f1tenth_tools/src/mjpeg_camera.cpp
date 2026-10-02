// V4L2 camera driver that publishes the camera's own MJPEG frames as sensor_msgs/CompressedImage.
// No decoding or re-encoding on the car, and a compressed stream is light enough for a remote UI.
// Reconnects if the camera is unplugged.
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

class MjpegCamera : public rclcpp::Node
{
public:
  MjpegCamera() : Node("camera")
  {
    device_ = declare_parameter<std::string>("device", "/dev/video0");
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 720);
    fps_ = declare_parameter<int>("fps", 30);
    max_rate_ = declare_parameter<double>("max_publish_rate", 0.0);  // Hz, 0 = every frame
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_optical_1_link");
    // For a camera mounted on its side: how far the UI turns the picture anticlockwise (0, 90, 180 or 270
    // degrees). The stream itself is not rotated, which would mean decoding and re-encoding every frame here.
    declare_parameter<int>("rotation", 0);
    param_cb_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        if (p.get_name() == "rotation" && (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
          p.as_int() % 90 != 0 || p.as_int() < 0 || p.as_int() >= 360))
        {
          result.successful = false;
          result.reason = "rotation must be 0, 90, 180 or 270";
        }
      }
      return result;
    });

    pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera/image_raw/compressed", rclcpp::SensorDataQoS());
    thread_ = std::thread([this]() {run();});
  }

  ~MjpegCamera() override
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    close_device();
  }

private:
  struct Buffer
  {
    void * start = nullptr;
    size_t length = 0;
  };

  static int xioctl(int fd, unsigned long request, void * arg)
  {
    int r;
    do {
      r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
  }

  bool open_device()
  {
    fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
      return fail("open " + device_);
    }
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
      return fail("set MJPEG format");
    }
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
      return fail("camera does not support MJPEG");
    }
    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps_;
    xioctl(fd_, VIDIOC_S_PARM, &parm);  // best effort

    v4l2_requestbuffers req{};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
      return fail("request buffers");
    }
    buffers_.resize(req.count);
    for (unsigned i = 0; i < req.count; ++i) {
      v4l2_buffer buf{};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
        return fail("query buffer");
      }
      buffers_[i].length = buf.length;
      buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
      if (buffers_[i].start == MAP_FAILED) {
        buffers_[i].start = nullptr;
        return fail("mmap");
      }
      if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        return fail("queue buffer");
      }
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
      return fail("start streaming");
    }
    RCLCPP_INFO(get_logger(), "Streaming %s: MJPEG %ux%u @ %d fps", device_.c_str(),
      fmt.fmt.pix.width, fmt.fmt.pix.height, fps_);
    return true;
  }

  bool fail(const std::string & what)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Camera: cannot %s: %s", what.c_str(), std::strerror(errno));
    close_device();
    return false;
  }

  void close_device()
  {
    if (fd_ >= 0) {
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(fd_, VIDIOC_STREAMOFF, &type);
    }
    for (auto & b : buffers_) {
      if (b.start) {
        munmap(b.start, b.length);
      }
    }
    buffers_.clear();
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  void run()
  {
    auto last_publish = std::chrono::steady_clock::now() - std::chrono::hours(1);
    while (running_ && rclcpp::ok()) {
      if (fd_ < 0 && !open_device()) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }
      pollfd pfd{fd_, POLLIN, 0};
      int r = poll(&pfd, 1, 1000);
      if (r <= 0) {
        if (r < 0 && errno != EINTR) {
          fail("poll");
        }
        continue;
      }
      v4l2_buffer buf{};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        if (errno != EAGAIN) {
          fail("read frame");  // e.g. unplugged; reopen on the next loop
        }
        continue;
      }
      auto t = std::chrono::steady_clock::now();
      bool publish = max_rate_ <= 0.0 ||
        std::chrono::duration<double>(t - last_publish).count() >= 1.0 / max_rate_;
      if (publish && buf.bytesused > 0) {
        last_publish = t;
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = now();
        msg->header.frame_id = frame_id_;
        msg->format = "jpeg";
        auto * data = static_cast<const uint8_t *>(buffers_[buf.index].start);
        msg->data.assign(data, data + buf.bytesused);
        pub_->publish(std::move(msg));
      }
      xioctl(fd_, VIDIOC_QBUF, &buf);
    }
  }

  std::string device_, frame_id_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  int width_, height_, fps_;
  double max_rate_;
  int fd_ = -1;
  std::vector<Buffer> buffers_;
  std::atomic<bool> running_{true};
  std::thread thread_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MjpegCamera>());
  rclcpp::shutdown();
  return 0;
}
