#include <array>
#include <csignal>
#include <iostream>
#include <thread>

#include <simple_vicon/ViconDriver.h>

ViconDriver driver;

void signal_handler(int signal) {
  driver.stop();
}

void vicon_callback(double time, std::array<double, 3> pos, std::array<double, 4> quat) {
  std::cout << "Time is " << time << std::endl;
  std::cout << "Pos is " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
  std::cout << "Quat is " << quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3] << std::endl;
}

int main(int argc, char **argv) {
  if (argc <= 2) {
    std::cerr << "Pass server IP address and object name" << std::endl;
    return 1;
  }

  if (!driver.init(argv[1], argv[2], vicon_callback)) {
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
