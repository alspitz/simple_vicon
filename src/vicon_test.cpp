#include <array>
#include <csignal>
#include <iostream>
#include <thread>

#include <simple_vicon/ViconDriver.h>

ViconDriver driver;

void signal_handler(int signal) {
  driver.stop();
}

void vicon_callback(vicon_result_t res) {
  std::cout << "Latency is " << res.latency << std::endl;
  for (const auto& vicon_pose : res.data) {
    std::cout << "Subject " << vicon_pose.subject << std::endl;
    std::cout << "\tPos is " << vicon_pose.pos[0] << " " << vicon_pose.pos[1] << " " << vicon_pose.pos[2] << std::endl;
    std::cout << "\tQuat is " << vicon_pose.quat[0] << " " << vicon_pose.quat[1] << " " << vicon_pose.quat[2] << " " << vicon_pose.quat[3] << std::endl;
  }
}

int main(int argc, char **argv) {
  if (argc <= 2) {
    std::cerr << "Pass server IP address and object name" << std::endl;
    return 1;
  }

  vicon_driver_params_t params;
  params.server_ip = argv[1];
  params.stream_mode = ViconSDK::StreamMode::ServerPush;
  params.target_subjects.push_back(argv[2]);

  if (!driver.init(params, vicon_callback)) {
    std::cerr << "Failed to init driver" << std::endl;
    return 1;
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  std::signal(SIGHUP, signal_handler);
  std::thread vicon_thread(&ViconDriver::run_loop, &driver);

  vicon_thread.join();

  return 0;
}
