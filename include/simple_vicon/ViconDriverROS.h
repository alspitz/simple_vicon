#include <simple_vicon/ViconDriver.h>

#include <thread>
#include <utility>
#include <vector>

#include <ros/ros.h>

class ViconDriverROS{
  public:
    bool initialize(const ros::NodeHandle& n);
    void finalize();

  private:
    void viconCallback(vicon_result_t vicon_result);
    ViconDriver driver_;
    std::thread vicon_thread_;

    std::vector<std::pair<std::string, ros::Publisher>> pubs_;
    ros::NodeHandle nh_;
};
