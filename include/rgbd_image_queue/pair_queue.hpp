#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace rgbd_image_queue
{
using CompressedImage = sensor_msgs::msg::CompressedImage;

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
  uint64_t history_seq;  ///< 入历史队列时递增；未入队为零。
  CompressedImage::ConstSharedPtr color;
  CompressedImage::ConstSharedPtr depth;
  int64_t received_steady_ns;
  uint64_t bytes() const {return color->data.size() + depth->data.size();}
};

struct View
{
  std::shared_ptr<const Pair> pair;
  double image_age_sec{-1.0};
  bool exceeded_max_age{false};
  int64_t queue_index{-1};
  double age_error_sec{0.0};
  uint32_t queue_size{0};
  bool latest_available{false};
  double latest_age_sec{-1.0};
  uint64_t paired_count{0};
  uint64_t enqueued_count{0};
  uint64_t expired_count{0};
  uint64_t capacity_evicted_count{0};
  uint64_t retained_image_bytes{0};
};

struct Batch
{
  std::vector<std::shared_ptr<const Pair>> pairs;
  uint64_t snapshot_seq{0};
  uint64_t next_seq{0};
  uint64_t missing_count{0};
  bool cursor_reset{false};
  bool history_enabled{false};
  bool has_more{false};
  int32_t latest_index{-1};
  double max_image_age_sec{0.0};
};

enum class Selection {None, Latest, Queue, QueueAtAge, ClearQueue};

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
    CompressedImage::ConstSharedPtr color, CompressedImage::ConstSharedPtr depth,
    int64_t ros_ns, int64_t steady_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prune_locked(ros_ns, steady_ns);
    const bool enqueue = config_.queue_length > 0 && sample_phase_ == 0;
    const auto seq = enqueue ? ++enqueued_count_ : 0;
    latest_ = std::make_shared<Pair>(
      Pair{++paired_count_, seq, std::move(color), std::move(depth), steady_ns});
    if (enqueue) {
      history_.push_back(latest_);
      history_bytes_ += latest_->bytes();
    }
    sample_phase_ = (sample_phase_ + 1) % config_.save_every_n_frames;
    prune_locked(ros_ns, steady_ns);
  }

  // 返回共享引用后即释放锁；服务的图像拷贝/序列化在锁外执行。
  View read(Selection selection, int64_t ros_ns, int64_t steady_ns, uint32_t detail = 0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prune_locked(ros_ns, steady_ns);
    if (selection == Selection::ClearQueue) {history_.clear(); history_bytes_ = 0;}
    View view;
    if (selection == Selection::Latest) {view.pair = latest_;}
    if (selection == Selection::Queue && detail < history_.size()) {
      view.queue_index = detail;
      view.pair = history_[history_.size() - 1 - detail];
    }
    if (selection == Selection::QueueAtAge) {
      const double requested_age = detail / 1000.0;
      double best_distance = 0.0;
      // 从新到旧扫描；相同误差保留较新的配对。
      for (size_t index = 0; index < history_.size(); ++index) {
        const auto & pair = history_[history_.size() - 1 - index];
        const double error = age(*pair, ros_ns, steady_ns) - requested_age;
        if (!view.pair || std::abs(error) < best_distance) {
          view.pair = pair;
          view.queue_index = static_cast<int64_t>(index);
          view.age_error_sec = error;
          best_distance = std::abs(error);
        }
      }
    }
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

  /** @brief 读取共享引用快照；预留最新对的位置，游标仅跨过已读或已明确丢失的历史。 */
  Batch read_batch(uint64_t after_seq, uint32_t max_pairs, bool start_at_head,
                   int64_t ros_ns, int64_t steady_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prune_locked(ros_ns, steady_ns);
    Batch batch;
    batch.history_enabled = config_.queue_length > 0;
    batch.max_image_age_sec = config_.max_image_age_sec;
    batch.snapshot_seq = enqueued_count_;
    batch.cursor_reset = after_seq > batch.snapshot_seq;
    batch.next_seq = start_at_head || batch.cursor_reset ? batch.snapshot_seq : after_seq;
    const auto removed_through = history_.empty() ? enqueued_count_ : history_.front()->history_seq - 1;
    if (removed_through > batch.next_seq) {
      batch.missing_count = removed_through - batch.next_seq;
      batch.next_seq = removed_through;
    }

    const std::size_t limit = max_pairs == 0 ? 8 : std::min(max_pairs, uint32_t{32});
    const auto unread = batch.snapshot_seq - batch.next_seq;
    const bool latest_fits_in_history = unread > 0 && unread <= limit && history_.back() == latest_;
    const auto history_limit = limit - (latest_ && !latest_fits_in_history ? 1 : 0);
    const auto count = std::min<uint64_t>(unread, history_limit);
    const auto first = history_.size() - unread;
    batch.pairs.reserve(static_cast<std::size_t>(count) + (latest_ ? 1 : 0));
    for (uint64_t i = 0; i < count; ++i) {
      batch.pairs.push_back(history_[first + i]);
    }
    batch.next_seq += count;
    batch.has_more = batch.next_seq < batch.snapshot_seq;
    if (latest_) {
      if (batch.pairs.empty() || batch.pairs.back() != latest_) {batch.pairs.push_back(latest_);}
      batch.latest_index = static_cast<int32_t>(batch.pairs.size() - 1);
    }
    return batch;
  }

  void prune(int64_t ros_ns, int64_t steady_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prune_locked(ros_ns, steady_ns);
  }

  static double age(const Pair & pair, int64_t ros_ns, int64_t steady_ns)
  {
    const auto captured = std::min(stamp_ns(pair.color->header.stamp),
                                  stamp_ns(pair.depth->header.stamp));
    return std::max({0.0, (ros_ns - captured) / 1e9,
                     (steady_ns - pair.received_steady_ns) / 1e9});
  }

private:
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
