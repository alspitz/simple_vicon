#include <simple_vicon/ViconDriver.h>

#include <thread>
#include <utility>
#include <vector>

#include <ros/ros.h>

class ViconDriverROS {
  public:
    bool initialize(const ros::NodeHandle& n);
    void finalize();

  private:
    void viconCallback(vicon_result_t vicon_result);
    ViconDriver driver_;
    std::thread vicon_thread_;

    typedef struct vicon_publishers {
      ros::Publisher subject_pub;
      ros::Publisher pose_pub;
    } vicon_publishers_t;

    // List of subjects. A subject is a name, publishers pair.
    std::vector<std::pair<std::string, vicon_publishers_t>> pubs_;
    ros::NodeHandle nh_;

    bool publish_subject_;
    bool publish_pose_;
};
