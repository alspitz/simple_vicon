#include <simple_vicon/ViconDriver.h>

#include <chrono>
#include <iostream>

bool ViconDriver::init(const std::string& host, const std::string& target_subject, callback_type callback) {
  target_subject_ = target_subject;
  callback_ = callback;

  client_.Connect(host);
  if (!client_.IsConnected().Connected) {
    std::cerr << "Failed to connect to Vicon server at " << host << std::endl;
    return false;
  }

  client_.SetStreamMode(ViconSDK::StreamMode::ServerPush);
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

    double now = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

    if (result != ViconSDK::Result::Success) {
      std::cerr << "ViconSDK GetFrame did not return success: " << result << std::endl;
      continue;
    }

    int n_subjects = client_.GetSubjectCount().SubjectCount;
    for (int i = 0; i < n_subjects; i++) {
      std::string subject_name = client_.GetSubjectName(i).SubjectName;
      if (subject_name != target_subject_) {
        continue;
      }

      double lat = client_.GetLatencyTotal().Total;

      std::string segment_name = client_.GetSegmentName(subject_name, 0).SegmentName;
      auto trans = client_.GetSegmentGlobalTranslation(subject_name, segment_name);
      auto rot = client_.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);

      if (trans.Result != ViconSDK::Result::Success) {
        std::cerr << "Translation get failed" << std::endl;
      }
      if (rot.Result != ViconSDK::Result::Success) {
        std::cerr << "Rotation get failed" << std::endl;
      }

      std::array<double, 3> pos {
        trans.Translation[0] / 1000.0,
        trans.Translation[1] / 1000.0,
        trans.Translation[2] / 1000.0
      };

      // Quaternion is returned in W last format; we store it W first.
      std::array<double, 4> quat {
        rot.Rotation[3],
        rot.Rotation[0],
        rot.Rotation[1],
        rot.Rotation[2],
      };

      callback_(now - lat, pos, quat);
    }
  }
}

void ViconDriver::stop() {
  running_.store(false);
}
