# 压缩 RGB-D 配对；history_seq=0 表示没有采样进历史队列。
uint64 pair_id
uint64 history_seq
float64 image_age_sec
bool exceeded_max_age
sensor_msgs/CompressedImage color
sensor_msgs/CompressedImage depth
