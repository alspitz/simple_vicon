#include <simple_vicon/ViconDriver.h>

#include <iostream>

bool ViconDriver::init(const vicon_driver_params_t& params, callback_type callback) {
  callback_ = callback;

  client_.Connect(params.server_ip);
  if (!client_.IsConnected().Connected) {
    std::cerr << "Failed to connect to Vicon server at " << params.server_ip << std::endl;
    return false;
  }

  client_.SetStreamMode(params.stream_mode);
  client_.SetAxisMapping(ViconSDK::Direction::Forward,
                         ViconSDK::Direction::Left,
                         ViconSDK::Direction::Up);

  return true;
}

void ViconDriver::run_loop() {
  client_.EnableSegmentData();

  running_.store(true);
  while (running_.load()) {
    auto result = client_.GetFrame().Result;

    if (result != ViconSDK::Result::Success) {
      std::cerr << "ViconSDK GetFrame did not return success: " << result << std::endl;
      continue;
    }

    vicon_result_t res;
    // TODO: Fill this with the actual time.
    // Proposed strategy: subtract latency from timecode
    res.time = client_.GetLatencyTotal().Total;

    int n_subjects = client_.GetSubjectCount().SubjectCount;
    for (int i = 0; i < n_subjects; i++) {
      std::string subject_name = client_.GetSubjectName(i).SubjectName;

      std::string segment_name = client_.GetSegmentName(subject_name, 0).SegmentName;
      auto trans = client_.GetSegmentGlobalTranslation(subject_name, segment_name);
      auto rot = client_.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);

      if (trans.Result != ViconSDK::Result::Success) {
        std::cerr << "Translation get failed" << std::endl;
      }
      if (rot.Result != ViconSDK::Result::Success) {
        std::cerr << "Rotation get failed" << std::endl;
      }

      res.data.emplace_back(
        subject_name,
        trans.Occluded or rot.Occluded,
        std::array<double, 3> {
          trans.Translation[0] / 1000.0,
          trans.Translation[1] / 1000.0,
          trans.Translation[2] / 1000.0
        },
        // Quaternion is returned in W last format; we store it W first.
        std::array<double, 4> {
          rot.Rotation[3],
          rot.Rotation[0],
          rot.Rotation[1],
          rot.Rotation[2]
        }
      );
    }

    if (res.data.size()) {
      callback_(std::move(res));
    }
  }
}

void ViconDriver::stop() {
  running_.store(false);
}
