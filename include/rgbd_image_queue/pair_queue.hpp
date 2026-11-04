#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <sensor_msgs/msg/image.hpp>

namespace rgbd_image_queue
{
using Image = sensor_msgs::msg::Image;

struct Config
{
  int64_t queue_length{10};
  int64_t save_every_n_frames{5};
  double max_image_age_sec{5.0};
};

// 仅包含用户批准的三个参数范围检查。
inline void validate_config(const Config & config)
{
  if (config.queue_length < 0 || config.save_every_n_frames < 1 ||
      !(config.max_image_age_sec >= 0.0))
  {
    throw std::invalid_argument(
      "queue_length >= 0, save_every_n_frames >= 1, max_image_age_sec >= 0");
  }
}

inline int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
}

struct Pair
{
  uint64_t id;
  Image::ConstSharedPtr color;
  Image::ConstSharedPtr depth;
  int64_t received_steady_ns;
  uint64_t bytes() const {return color->data.size() + depth->data.size();}
};

struct View
{
  std::shared_ptr<const Pair> pair;
  double image_age_sec{-1.0};
  bool exceeded_max_age{false};
  uint32_t queue_size{0};
  bool latest_available{false};
  double latest_age_sec{-1.0};
  uint64_t paired_count{0};
  uint64_t enqueued_count{0};
  uint64_t expired_count{0};
  uint64_t capacity_evicted_count{0};
  uint64_t retained_image_bytes{0};
};

enum class Selection {None, Latest, Queue, ClearQueue};

class PairQueue
{
public:
  explicit PairQueue(Config config = {}) : config_(config) {validate_config(config);}

  Config config() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
  }

  void configure(const Config & config, int64_t ros_ns, int64_t steady_ns)
  {
    validate_config(config);
    std::lock_guard<std::mutex> lock(mutex_);
    if (config.save_every_n_frames != config_.save_every_n_frames) {sample_phase_ = 0;}
    config_ = config;
    prune_locked(ros_ns, steady_ns);
  }

  void push(
    Image::ConstSharedPtr color, Image::ConstSharedPtr depth,
    int64_t ros_ns, int64_t steady_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prune_locked(ros_ns, steady_ns);
    latest_ = std::make_shared<Pair>(
      Pair{++paired_count_, std::move(color), std::move(depth), steady_ns});
    if (config_.queue_length > 0 && sample_phase_ == 0) {
      history_.push_back(latest_);
      history_bytes_ += latest_->bytes();
      ++enqueued_count_;
    }
    sample_phase_ = (sample_phase_ + 1) % config_.save_every_n_frames;
    prune_locked(ros_ns, steady_ns);
  }

  // 返回共享引用后即释放锁；服务的图像拷贝/序列化在锁外执行。
  View read(Selection selection, int64_t ros_ns, int64_t steady_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prune_locked(ros_ns, steady_ns);
    if (selection == Selection::ClearQueue) {history_.clear(); history_bytes_ = 0;}
    View view;
    if (selection == Selection::Latest) {view.pair = latest_;}
    if (selection == Selection::Queue && !history_.empty()) {view.pair = history_.back();}
    if (view.pair) {
      view.image_age_sec = age(*view.pair, ros_ns, steady_ns);
      view.exceeded_max_age = config_.max_image_age_sec > 0 &&
        view.image_age_sec > config_.max_image_age_sec;
    }
    view.queue_size = static_cast<uint32_t>(history_.size());
    view.latest_available = static_cast<bool>(latest_);
    view.latest_age_sec = latest_ ? age(*latest_, ros_ns, steady_ns) : -1.0;
    view.paired_count = paired_count_;
    view.enqueued_count = enqueued_count_;
    view.expired_count = expired_count_;
    view.capacity_evicted_count = capacity_evicted_count_;
    view.retained_image_bytes = history_bytes_;
    // latest 若恰好也是队列尾部，仅计一次图像数据。
    if (latest_ && (history_.empty() || history_.back() != latest_)) {
      view.retained_image_bytes += latest_->bytes();
    }
    return view;
  }

  void prune(int64_t ros_ns, int64_t steady_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prune_locked(ros_ns, steady_ns);
  }

private:
  static double age(const Pair & pair, int64_t ros_ns, int64_t steady_ns)
  {
    const auto captured = std::min(stamp_ns(pair.color->header.stamp),
                                  stamp_ns(pair.depth->header.stamp));
    return std::max({0.0, (ros_ns - captured) / 1e9,
                     (steady_ns - pair.received_steady_ns) / 1e9});
  }

  void pop_front()
  {
    history_bytes_ -= history_.front()->bytes();
    history_.pop_front();
  }

  void prune_locked(int64_t ros_ns, int64_t steady_ns)
  {
    while (!history_.empty() && config_.max_image_age_sec > 0 &&
      age(*history_.front(), ros_ns, steady_ns) > config_.max_image_age_sec)
    {
      pop_front();
      ++expired_count_;
    }
    while (history_.size() > static_cast<size_t>(config_.queue_length)) {
      pop_front();
      ++capacity_evicted_count_;
    }
  }

  mutable std::mutex mutex_;
  Config config_;
  std::deque<std::shared_ptr<const Pair>> history_;
  std::shared_ptr<const Pair> latest_;
  int64_t sample_phase_{0};
  uint64_t history_bytes_{0};
  uint64_t paired_count_{0}, enqueued_count_{0}, expired_count_{0}, capacity_evicted_count_{0};
};
}  // namespace rgbd_image_queue
