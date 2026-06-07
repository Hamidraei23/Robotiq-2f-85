/**
 * sinwave_gripper.cpp
 *
 * Replicates the 2f_sinwave.py logic in C++ using non-blocking goal streaming
 * (Option A). A ros::Timer fires at `rate` Hz and calls goToPosition(..., wait=false)
 * each tick. Each new call preempts the in-flight goal on the action server, so
 * the gripper continuously chases the updated setpoint without blocking.
 *
 * Note: RobotiqActionClient uses private inheritance, so the public interface is:
 *   goToPosition(pos, speed, force, wait=false)  — non-blocking stream
 *   goToPosition(pos, speed, force, wait=true)   — blocking (do NOT use in timer)
 *   close() / open()
 *
 * Parameters (all private, prefix with ~):
 *   ~min_gap   [m]   default 0.02
 *   ~max_gap   [m]   default 0.04
 *   ~frequency [Hz]  default 0.2   (oscillation frequency)
 *   ~rate      [Hz]  default 20.0  (goal publish rate)
 *   ~speed     [m/s] default 0.05  (keep low for smoothness; min=0.013)
 *   ~force     [%]   default 20.0
 */

#include <cmath>
#include <ros/ros.h>
#include <robotiq_2f_gripper_control/robotiq_gripper_client.h>

class SinwaveGripper
{
public:
    SinwaveGripper(ros::NodeHandle& nh)
        : client_("command_robotiq_action")
    {
        min_gap_   = nh.param("min_gap",   0.02);
        max_gap_   = nh.param("max_gap",   0.04);
        frequency_ = nh.param("frequency", 0.2);
        rate_hz_   = nh.param("rate",      20.0);
        speed_     = static_cast<float>(nh.param("speed", 0.05));
        force_     = static_cast<float>(nh.param("force", 20.0));

        center_    = 0.5 * (min_gap_ + max_gap_);
        amplitude_ = 0.5 * (max_gap_ - min_gap_);

        ROS_INFO("Connected to Robotiq action server. Starting sinwave streaming.");

        start_ = ros::Time::now();
        timer_ = nh.createTimer(ros::Duration(1.0 / rate_hz_),
                                &SinwaveGripper::timerCb, this);
    }

private:
    void timerCb(const ros::TimerEvent&)
    {
        double t   = (ros::Time::now() - start_).toSec();
        double gap = center_ + amplitude_ * std::sin(2.0 * M_PI * frequency_ * t);

        // Non-blocking: preempts the previous in-flight goal each tick.
        // The gripper's internal generator continuously chases the new setpoint.
        client_.goToPosition(static_cast<float>(gap), speed_, force_, /*wait=*/false);

        ROS_INFO_THROTTLE(0.5, "Commanded gap: %.3f m (%.1f cm)", gap, gap * 100.0);
    }

    robotiq_2f_gripper_control::RobotiqActionClient client_;
    ros::Timer timer_;
    ros::Time  start_;

    double min_gap_, max_gap_, frequency_, rate_hz_;
    double center_, amplitude_;
    float  speed_, force_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sinwave_gripper_cpp");
    ros::NodeHandle nh("~");

    SinwaveGripper node(nh);
    ros::spin();
    return 0;
}
