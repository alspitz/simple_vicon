#include <simple_vicon/ViconDriverROS.h>

#include <simple_vicon/Subject.h>

bool ViconDriverROS::initialize(const ros::NodeHandle& n) {
  nh_ = n;

  vicon_driver_params_t params;
  if (!ros::param::get("vicon/server_ip", params.server_ip)) {
    ROS_ERROR("[ViconDriverROS] Could not get parameter vicon/server_ip");
    return false;
  }

  std::string stream_mode;
  if (!ros::param::get("vicon/stream_mode", stream_mode)) {
    ROS_ERROR("[ViconDriverROS] Could not get parameter vicon/stream_mode");
    return false;
  }

  if (stream_mode == "ClientPull") params.stream_mode = ViconSDK::StreamMode::ClientPull;
  else if (stream_mode == "ClientPullPreFetch") params.stream_mode = ViconSDK::StreamMode::ClientPullPreFetch;
  else if (stream_mode == "ServerPush") params.stream_mode = ViconSDK::StreamMode::ServerPush;
  else {
    ROS_ERROR("[ViconDriverROS] Invalid Vicon stream mode %s. Choose from ClientPull, ClientPullPreFetch, or ServerPush", stream_mode.c_str());
    return false;
  }

  if (!driver_.init(params, std::bind(&ViconDriverROS::viconCallback, this, std::placeholders::_1))) {
    return false;
  }

  vicon_thread_ = std::thread(&ViconDriver::run_loop, &driver_);

  return true;
}

void ViconDriverROS::finalize() {
  driver_.stop();
  vicon_thread_.join();
}

void ViconDriverROS::viconCallback(vicon_result_t vicon_result) {
  for (const auto& vicon_pose : vicon_result.data) {
    auto it = std::find_if(pubs_.begin(), pubs_.end(), [&vicon_pose] (const auto &p) { return p.first == vicon_pose.subject; });

    ros::Publisher *pub;

    if (it == pubs_.end()) {
      ROS_INFO("[ViconDriverROS] Adding new publisher for %s", vicon_pose.subject.c_str());
      pubs_.emplace_back(vicon_pose.subject, nh_.advertise<simple_vicon::Subject>(vicon_pose.subject, 1));
      pub = &pubs_[pubs_.size() - 1].second;
    }
    else {
      pub = &it->second;
    }

    simple_vicon::Subject sub;
    sub.header.stamp = ros::Time(vicon_result.time);
    sub.occluded = vicon_pose.occluded;
    sub.pose.position.x = vicon_pose.pos[0];
    sub.pose.position.y = vicon_pose.pos[1];
    sub.pose.position.z = vicon_pose.pos[2];
    sub.pose.orientation.w = vicon_pose.quat[0];
    sub.pose.orientation.x = vicon_pose.quat[1];
    sub.pose.orientation.y = vicon_pose.quat[2];
    sub.pose.orientation.z = vicon_pose.quat[3];

    pub->publish(sub);
  }
}
