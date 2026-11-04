#include <gtest/gtest.h>
#include "rgbd_image_queue/pair_queue.hpp"
using namespace rgbd_image_queue;
constexpr int64_t S = 1000000000LL;

CompressedImage::SharedPtr image(uint8_t value, int64_t stamp = S, size_t bytes = 3)
{
  auto msg = std::make_shared<CompressedImage>();
  msg->header.stamp.sec = static_cast<int32_t>(stamp / S);
  msg->header.stamp.nanosec = static_cast<uint32_t>(stamp % S);
  msg->header.frame_id = "camera_color_optical_frame";
  msg->format = bytes == 3 ? "rgb8; jpeg compressed bgr8" : "16UC1; compressedDepth png";
  msg->data.assign(bytes, value);
  return msg;
}
void push(PairQueue & q, uint8_t id, int64_t now = S)
{
  q.push(image(id, now), image(id, now, 2), now, now);
}

TEST(PairQueue, EmptyIsNormal)
{
  PairQueue q;
  auto v = q.read(Selection::Queue, S, S);
  EXPECT_FALSE(v.pair);
  EXPECT_FALSE(v.latest_available);
  EXPECT_EQ(v.queue_size, 0u);
  EXPECT_EQ(v.latest_age_sec, -1);
}
TEST(PairQueue, LatestUpdatesEveryPairHistorySamplesFirstAndEveryN)
{
  PairQueue q({10, 3, 5});
  push(q, 1); push(q, 2); push(q, 3);
  auto latest = q.read(Selection::Latest, S, S);
  auto history = q.read(Selection::Queue, S, S);
  EXPECT_EQ(latest.pair->id, 3u);
  EXPECT_EQ(history.pair->id, 1u);
  EXPECT_EQ(latest.pair->color->data[0], 3);
  push(q, 4);
  EXPECT_EQ(q.read(Selection::Queue, S, S).pair->id, 4u);
}
TEST(PairQueue, CapacityEvictsOldest)
{
  PairQueue q({2, 1, 0});
  for (uint8_t i=1; i<=7; ++i) {push(q, i);}
  auto v = q.read(Selection::Queue, S, S);
  EXPECT_EQ(v.queue_size, 2u);
  EXPECT_EQ(v.pair->id, 7u);
  EXPECT_EQ(v.capacity_evicted_count, 5u);
  EXPECT_EQ(v.retained_image_bytes, 10u);
}
TEST(PairQueue, ExpiryKeepsIndependentLatest)
{
  PairQueue q({10, 1, .5});
  push(q, 1);
  auto v = q.read(Selection::Queue, 2*S, 2*S);
  EXPECT_FALSE(v.pair);
  EXPECT_EQ(v.queue_size, 0u);
  EXPECT_EQ(v.expired_count, 1u);
  v = q.read(Selection::Latest, 2*S, 2*S);
  ASSERT_TRUE(v.pair);
  EXPECT_TRUE(v.exceeded_max_age);
  EXPECT_DOUBLE_EQ(v.image_age_sec, 1);
}
TEST(PairQueue, ExpiryUsesSteadyTimeWhenRosClockPaused)
{
  PairQueue q({10, 1, .5});
  push(q, 1);
  EXPECT_EQ(q.read(Selection::Queue, S, 2*S).queue_size, 0u);
  EXPECT_EQ(q.read(Selection::Latest, S, 2*S).latest_age_sec, 1);
}
TEST(PairQueue, LifetimeZeroDisablesExpiry)
{
  PairQueue q({2, 1, 0});
  push(q, 1);
  auto v=q.read(Selection::Queue, 100*S, 100*S);
  ASSERT_TRUE(v.pair);
  EXPECT_FALSE(v.exceeded_max_age);
}
TEST(PairQueue, ZeroCapacityRetainsOnlyLatest)
{
  PairQueue q({0, 2, 5});
  push(q, 1); push(q, 2);
  auto v=q.read(Selection::Latest, S, S);
  ASSERT_TRUE(v.pair);
  EXPECT_EQ(v.pair->id, 2u);
  EXPECT_EQ(v.queue_size, 0u);
  EXPECT_EQ(v.enqueued_count, 0u);
  EXPECT_EQ(v.retained_image_bytes, 5u);
}
TEST(PairQueue, ClearHistoryLeavesLatest)
{
  PairQueue q({2, 1, 5});
  push(q, 1); push(q, 2);
  auto v=q.read(Selection::ClearQueue, S, S);
  EXPECT_EQ(v.queue_size, 0u);
  EXPECT_TRUE(v.latest_available);
  EXPECT_EQ(q.read(Selection::Latest, S, S).pair->id, 2u);
}
TEST(PairQueue, DynamicShrinkAndStrideChange)
{
  PairQueue q({4, 1, 5});
  push(q, 1); push(q, 2); push(q, 3);
  q.configure({1, 4, 5}, S, S);
  EXPECT_EQ(q.read(Selection::Queue, S, S).queue_size, 1u);
  push(q, 4); push(q, 5);
  EXPECT_EQ(q.read(Selection::Queue, S, S).pair->id, 4u);
  EXPECT_EQ(q.read(Selection::Latest, S, S).pair->id, 5u);
}
TEST(PairQueue, SharedImagesArePreservedAndBytesNotDoubleCounted)
{
  PairQueue q({4, 2, 5});
  auto color=image(9), depth=image(42, S+3000000, 2);
  q.push(color, depth, S+10000000, S);
  auto v=q.read(Selection::Latest, S+10000000, S);
  EXPECT_EQ(v.pair->color.get(), color.get());
  EXPECT_EQ(v.pair->depth.get(), depth.get());
  EXPECT_EQ(v.pair->depth->data, depth->data);
  EXPECT_EQ(v.pair->color->format, color->format);
  EXPECT_EQ(v.pair->depth->format, depth->format);
  EXPECT_EQ(v.pair->depth->header.frame_id, depth->header.frame_id);
  EXPECT_EQ(v.retained_image_bytes, 5u);
  push(q, 10);
  EXPECT_EQ(q.read(Selection::Latest, S, S).retained_image_bytes, 10u);
}
TEST(PairQueue, ApprovedRangeChecksDoNotChangeConfigOnFailure)
{
  PairQueue q({2, 3, 5});
  EXPECT_THROW(q.configure({-1, 3, 5}, S, S), std::invalid_argument);
  EXPECT_THROW(q.configure({2, 0, 5}, S, S), std::invalid_argument);
  EXPECT_THROW(q.configure({2, 3, -1}, S, S), std::invalid_argument);
  EXPECT_EQ(q.config().queue_length, 2);
  EXPECT_EQ(q.config().save_every_n_frames, 3);
  EXPECT_EQ(q.config().max_image_age_sec, 5);
}


TEST(PairQueue, QueueIndexIsNewestFirstAndNeverFallsBack)
{
  PairQueue q({3, 1, 0});
  for (uint8_t i=1; i<=4; ++i) {push(q, i, i*S);}
  for (uint32_t index=0; index<3; ++index) {
    const auto v=q.read(Selection::Queue, 5*S, 5*S, index);
    ASSERT_TRUE(v.pair);
    EXPECT_EQ(v.pair->id, 4u-index);
    EXPECT_EQ(v.queue_index, index);
  }
  const auto missing=q.read(Selection::Queue, 5*S, 5*S, 3);
  EXPECT_FALSE(missing.pair);
  EXPECT_EQ(missing.queue_index, -1);
  EXPECT_EQ(q.read(Selection::None, 5*S, 5*S).queue_size, 3u);
}

TEST(PairQueue, AgeSearchReturnsClosestAndSignedError)
{
  PairQueue q({4, 1, 0});
  push(q, 1, S); push(q, 2, 2*S); push(q, 3, 3*S);
  // 当前 4s，三对年龄分别为 3s、2s、1s，请求 2.2s 前。
  auto v=q.read(Selection::QueueAtAge, 4*S, 4*S, 2200);
  ASSERT_TRUE(v.pair);
  EXPECT_EQ(v.pair->id, 2u);
  EXPECT_EQ(v.queue_index, 1);
  EXPECT_DOUBLE_EQ(v.image_age_sec, 2);
  EXPECT_NEAR(v.age_error_sec, -.2, 1e-12);
  v=q.read(Selection::QueueAtAge, 4*S, 4*S, 1800);
  EXPECT_NEAR(v.age_error_sec, .2, 1e-12);
}

TEST(PairQueue, AgeSearchTiesPreferNewerAndRangeUsesNearestEndpoint)
{
  PairQueue q({4, 1, 0});
  push(q, 1, S); push(q, 2, 2*S); push(q, 3, 3*S);
  EXPECT_EQ(q.read(Selection::QueueAtAge, 4*S, 4*S, 1500).pair->id, 3u);
  EXPECT_EQ(q.read(Selection::QueueAtAge, 4*S, 4*S, 0).pair->id, 3u);
  auto v=q.read(Selection::QueueAtAge, 4*S, 4*S, UINT32_MAX);
  EXPECT_EQ(v.pair->id, 1u);
  EXPECT_EQ(v.queue_index, 2);
}

TEST(PairQueue, AgeSearchUsesCaptureTimeAndOlderStampOfPair)
{
  PairQueue q({4, 1, 0});
  // 在 5s 才接收到采集于 1s/1.1s 的消息，年龄应为 4s，而不是 0s。
  q.push(image(1,S),image(1,S+S/10,2),5*S,5*S);
  q.push(image(2,3*S),image(2,3*S+S/10,2),5*S,5*S);
  auto v=q.read(Selection::QueueAtAge,5*S,5*S,3900);
  EXPECT_EQ(v.pair->id,1u);
  EXPECT_DOUBLE_EQ(v.image_age_sec,4);
  EXPECT_NEAR(v.age_error_sec,.1,1e-12);
}

TEST(PairQueue, HistoricalQueriesExcludeUnsampledLatestAndExpiredImages)
{
  PairQueue q({4, 3, 2});
  push(q, 1,S); push(q, 2,2*S);
  auto v=q.read(Selection::QueueAtAge,2*S,2*S,0);
  EXPECT_EQ(v.pair->id,1u);
  EXPECT_EQ(q.read(Selection::Latest,2*S,2*S).pair->id,2u);
  v=q.read(Selection::QueueAtAge,4*S,4*S,3000);
  EXPECT_FALSE(v.pair);
  EXPECT_EQ(v.queue_index,-1);
  EXPECT_TRUE(v.latest_available);
}

TEST(PairQueue, AgeSearchOnEmptyOrClearedHistoryDoesNotUseLatest)
{
  PairQueue q({4,1,0});
  EXPECT_FALSE(q.read(Selection::QueueAtAge,S,S,0).pair);
  push(q,1);
  q.read(Selection::ClearQueue,S,S);
  EXPECT_FALSE(q.read(Selection::QueueAtAge,S,S,0).pair);
  EXPECT_EQ(q.read(Selection::Latest,S,S,123).queue_index,-1);
}
