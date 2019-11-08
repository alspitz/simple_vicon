#pragma once

#include <array>
#include <atomic>
#include <functional>
#include <string>
#include <vector>

#include <ViconDataStreamSDK_CPP/DataStreamClient.h>

namespace ViconSDK = ViconDataStreamSDK::CPP;

typedef struct vicon_driver_params {
  std::string server_ip;
  // ClientPull, ClientPullPreFetch, or ServerPush
  ViconSDK::StreamMode::Enum stream_mode;
} vicon_driver_params_t;

typedef struct vicon_pose {
  std::string subject;
  bool occluded;
  std::array<double, 3> pos;
  std::array<double, 4> quat;

  vicon_pose(const std::string& subject, bool occluded, std::array<double, 3>&& pos, std::array<double, 4>&& quat) :
    subject(subject),
    occluded(occluded),
    pos(pos),
    quat(quat) {}
} vicon_pose_t;

typedef struct vicon_result {
  double time{0.0};
  std::vector<vicon_pose_t> data;
} vicon_result_t;

class ViconDriver {
  typedef std::function<void(vicon_result_t)> callback_type;

  public:
    bool init(const vicon_driver_params_t& params, callback_type callback);
    void run_loop();
    void stop();

  private:
    ViconSDK::Client client_;
    std::atomic<bool> running_;
    callback_type callback_;
};
