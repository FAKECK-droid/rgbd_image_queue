#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rgbd_image_queue/pair_queue.hpp"
#include "rgbd_image_queue/srv/get_image.hpp"

namespace rgbd_image_queue
{
using GetImage = srv::GetImage;
using Policy = message_filters::sync_policies::ApproximateTime<CompressedImage, CompressedImage>;
using Synchronizer = message_filters::Synchronizer<Policy>;

int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

class QueueNode : public rclcpp::Node
{
public:
  QueueNode() : Node("rgbd_image_queue")
  {
    Config config;
    config.queue_length = declare_parameter<int64_t>("queue_length", 10);
    config.save_every_n_frames = declare_parameter<int64_t>("save_every_n_frames", 5);
    config.max_image_age_sec = declare_parameter<double>("max_image_age_sec", 5.0);
    queue_ = std::make_unique<PairQueue>(config);
    const auto color_topic = fixed<std::string>("color_topic", "/camera/color/image_raw/compressed");
    const auto depth_topic = fixed<std::string>("depth_topic", "/camera/depth/image_raw/compressedDepth");
    const auto service_name = fixed<std::string>("service_name", "~/get_image");
    const auto tolerance = fixed<double>("sync_tolerance_sec", 0.04);
    const auto sync_length = fixed<int64_t>("sync_queue_size", 4);
    const auto qos_depth = fixed<int64_t>("qos_depth", 2);
    const auto purge_period = fixed<double>("purge_period_sec", 0.2);
    image_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = image_group_;
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(static_cast<size_t>(qos_depth));
    // 直接订阅完整话题，消息头、format 和压缩字节均不修改。
    color_sub_.subscribe(this, color_topic, qos.get_rmw_qos_profile(), options);
    depth_sub_.subscribe(this, depth_topic, qos.get_rmw_qos_profile(), options);
    sync_ = std::make_shared<Synchronizer>(
      Policy(static_cast<uint32_t>(sync_length)), color_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(tolerance));
    sync_->registerCallback(
      std::bind(&QueueNode::on_pair, this, std::placeholders::_1, std::placeholders::_2));
    service_ = create_service<GetImage>(
      service_name,
      std::bind(&QueueNode::on_request, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_group_);
    purge_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(purge_period)),
      [this]() {queue_->prune(now().nanoseconds(), steady_ns());}, image_group_);
    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(&QueueNode::on_parameters, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
      "RGB-D queue: %s + %s; service=%s; length=%ld, every=%ld, lifetime=%.3fs, skew<=%.3fs",
      color_topic.c_str(), depth_topic.c_str(), service_->get_service_name(),
      static_cast<long>(config.queue_length), static_cast<long>(config.save_every_n_frames),
      config.max_image_age_sec, tolerance);
  }

private:
  template<typename T>
  T fixed(const std::string & name, T value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, std::move(value), descriptor);
  }

  void on_pair(
    const CompressedImage::ConstSharedPtr & color, const CompressedImage::ConstSharedPtr & depth)
  {
    // 同步器按消息时间戳一对一配对；共享原始压缩消息，不读取图像内容。
    queue_->push(color, depth, now().nanoseconds(), steady_ns());
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    auto config = queue_->config();
    bool changed = false;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "queue_length") {
        config.queue_length = parameter.as_int();
        changed = true;
      } else if (parameter.get_name() == "save_every_n_frames") {
        config.save_every_n_frames = parameter.as_int();
        changed = true;
      } else if (parameter.get_name() == "max_image_age_sec") {
        config.max_image_age_sec = parameter.as_double();
        changed = true;
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    // 仅执行已获批准的参数范围检查；失败时本次更新不改变队列配置。
    if (changed) {
      try {
        queue_->configure(config, now().nanoseconds(), steady_ns());
      } catch (const std::invalid_argument & error) {
        result.successful = false;
        result.reason = error.what();
      }
    }
    return result;
  }

  void on_request(
    const GetImage::Request::SharedPtr request, GetImage::Response::SharedPtr response)
  {
    using Request = GetImage::Request;
    response->queue_index = -1;
    Selection selection = Selection::None;
    bool color = false, depth = false, fetch = false;
    switch (request->operation) {
      case Request::LATEST_COLOR: selection = Selection::Latest; color = fetch = true; break;
      case Request::LATEST_DEPTH: selection = Selection::Latest; depth = fetch = true; break;
      case Request::LATEST_PAIR:
        selection = Selection::Latest; color = depth = fetch = true; break;
      case Request::QUEUE_COLOR: selection = Selection::Queue; color = fetch = true; break;
      case Request::QUEUE_DEPTH: selection = Selection::Queue; depth = fetch = true; break;
      case Request::QUEUE_PAIR:
        selection = Selection::Queue; color = depth = fetch = true; break;
      case Request::QUEUE_COLOR_AT_AGE:
        selection = Selection::QueueAtAge; color = fetch = true; break;
      case Request::QUEUE_DEPTH_AT_AGE:
        selection = Selection::QueueAtAge; depth = fetch = true; break;
      case Request::QUEUE_PAIR_AT_AGE:
        selection = Selection::QueueAtAge; color = depth = fetch = true; break;
      case Request::STATUS: break;
      case Request::CLEAR_QUEUE: selection = Selection::ClearQueue; break;
      default: response->message = "unknown operation"; return;
    }
    auto view = queue_->read(selection, now().nanoseconds(), steady_ns(), request->detail);
    response->queue_index = view.queue_index;
    response->age_error_sec = view.age_error_sec;
    response->queue_size = view.queue_size;
    response->latest_available = view.latest_available;
    response->latest_age_sec = view.latest_age_sec;
    response->paired_count = view.paired_count;
    response->enqueued_count = view.enqueued_count;
    response->expired_count = view.expired_count;
    response->capacity_evicted_count = view.capacity_evicted_count;
    response->retained_image_bytes = view.retained_image_bytes;
    response->success = !fetch || static_cast<bool>(view.pair);
    response->message = response->success ? "ok" :
      (selection == Selection::Queue ? "queue index out of range" : "selected cache is empty");
    response->image_age_sec = view.image_age_sec;
    response->exceeded_max_age = view.exceeded_max_age;
    if (view.pair) {
      response->pair_id = view.pair->id;
      response->color_stamp = view.pair->color->header.stamp;
      response->depth_stamp = view.pair->depth->header.stamp;
      response->stamp_delta_sec = std::abs(
        stamp_ns(response->color_stamp) - stamp_ns(response->depth_stamp)) / 1e9;
      response->has_color = color;
      response->has_depth = depth;
      // 大图拷贝在缓存锁外；服务组与图像订阅组可以并行运行。
      if (color) {response->color = *view.pair->color;}
      if (depth) {response->depth = *view.pair->depth;}
    }
  }

  std::unique_ptr<PairQueue> queue_;
  rclcpp::CallbackGroup::SharedPtr image_group_, service_group_;
  message_filters::Subscriber<CompressedImage> color_sub_, depth_sub_;
  std::shared_ptr<Synchronizer> sync_;
  rclcpp::Service<GetImage>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr purge_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}  // namespace rgbd_image_queue

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rgbd_image_queue::QueueNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::invalid_argument & error) {
    RCLCPP_ERROR(rclcpp::get_logger("rgbd_image_queue"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
