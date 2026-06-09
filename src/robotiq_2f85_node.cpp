/**
 * robotiq_2f85_node.cpp
 *
 * ROS node that drives the Robotiq 2F-85 directly over Modbus RTU (libmodbus).
 * No Python, no action server, no extra topic hops.
 *
 * Parameters (all private, prefix with ~):
 *   port          (string)  Serial device          default: /dev/ttyUSB0
 *   baudrate      (int)     Baud rate              default: 115200
 *   slave_id      (int)     Modbus slave address   default: 9
 *   status_rate   (double)  Status publish rate Hz default: 50.0
 *   auto_activate (bool)    Activate on startup    default: true
 *
 * Services (all under the node's private namespace):
 *   ~/activate   (std_srvs/Trigger)              — reset + activate sequence
 *   ~/open       (std_srvs/Trigger)              — fully open, max speed/force
 *   ~/close      (std_srvs/Trigger)              — fully close, max speed/force
 *   ~/stop       (std_srvs/Trigger)              — halt current motion
 *   ~/move       (robotiq_2f85_driver/GripperMove) — move to pos/vel/force
 *
 * Published topic:
 *   ~/status     (robotiq_2f85_driver/GripperStatus) — at status_rate Hz
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "robotiq_2f85_driver/robotiq_2f85_driver.h"
#include "robotiq_2f85_driver/GripperMove.h"
#include "robotiq_2f85_driver/GripperStatus.h"

using namespace robotiq_2f85;

// ─────────────────────────────────────────────────────────────────────────────
// Helper: convert internal GripperStatus → ROS message
// ─────────────────────────────────────────────────────────────────────────────
static robotiq_2f85_driver::GripperStatus toMsg(const GripperStatus& s)
{
  robotiq_2f85_driver::GripperStatus msg;
  msg.activated        = s.activated;
  msg.moving           = s.moving;
  msg.object_detected  = s.object_detected;
  msg.at_position      = s.at_position;
  msg.fault            = s.fault;
  msg.pos_raw          = s.pos_raw;
  msg.current_raw      = s.current_raw;
  msg.position_m       = s.position_m;
  msg.current_ma       = s.current_ma;
  return msg;
}

// ─────────────────────────────────────────────────────────────────────────────
// Node class
// ─────────────────────────────────────────────────────────────────────────────
class GripperNode
{
public:
  GripperNode(ros::NodeHandle& nh,
              const std::string& port,
              int baudrate,
              int slave_id,
              double status_rate,
              bool auto_activate)
    : driver_(port, baudrate, slave_id)
    , nh_(nh)
  {
    if (!driver_.connect())
    {
      throw std::runtime_error("Cannot connect to gripper on " + port);
    }

    if (auto_activate)
    {
      ROS_INFO("[GripperNode] auto_activate=true — running activation sequence");
      if (!driver_.activate(8000))
      {
        ROS_WARN("[GripperNode] Activation did not complete — gripper may already "
                 "be active, or call ~/activate manually");
      }
    }

    // Status publisher
    status_pub_ = nh_.advertise<robotiq_2f85_driver::GripperStatus>("status", 1);

    // Services
    srv_activate_ = nh_.advertiseService("activate", &GripperNode::srvActivate, this);
    srv_move_     = nh_.advertiseService("move",     &GripperNode::srvMove,     this);
    srv_open_     = nh_.advertiseService("open",     &GripperNode::srvOpen,     this);
    srv_close_    = nh_.advertiseService("close",    &GripperNode::srvClose,    this);
    srv_stop_     = nh_.advertiseService("stop",     &GripperNode::srvStop,     this);

    // Status timer — uses ROS timer (serialised by AsyncSpinner below)
    status_timer_ = nh_.createTimer(
        ros::Duration(1.0 / status_rate),
        &GripperNode::statusTimerCb, this);

    ROS_INFO("[GripperNode] Ready.  Services: activate | move | open | close | stop");
    ROS_INFO("[GripperNode] Status:  ~/status at %.0f Hz", status_rate);
  }

private:
  // ── Service handlers ───────────────────────────────────────────────────────

  bool srvActivate(std_srvs::Trigger::Request&,
                   std_srvs::Trigger::Response& res)
  {
    const bool ok = driver_.activate(8000);
    res.success = ok;
    res.message = ok ? "Gripper activated" : "Activation failed — check fault code";
    return true;
  }

  bool srvMove(robotiq_2f85_driver::GripperMove::Request& req,
               robotiq_2f85_driver::GripperMove::Response& res)
  {
    const bool ok = driver_.move(req.position, req.velocity, req.force);
    res.success = ok;
    res.message = ok ? "Move command dispatched" : "Modbus write failed";
    return true;
  }

  bool srvOpen(std_srvs::Trigger::Request&,
               std_srvs::Trigger::Response& res)
  {
    const bool ok = driver_.open();
    res.success = ok;
    res.message = ok ? "Open command dispatched" : "Modbus write failed";
    return true;
  }

  bool srvClose(std_srvs::Trigger::Request&,
                std_srvs::Trigger::Response& res)
  {
    const bool ok = driver_.close();
    res.success = ok;
    res.message = ok ? "Close command dispatched" : "Modbus write failed";
    return true;
  }

  bool srvStop(std_srvs::Trigger::Request&,
               std_srvs::Trigger::Response& res)
  {
    const bool ok = driver_.stop();
    res.success = ok;
    res.message = ok ? "Stop command dispatched" : "Modbus write failed";
    return true;
  }

  // ── Timer callback (status polling) ───────────────────────────────────────

  void statusTimerCb(const ros::TimerEvent&)
  {
    const GripperStatus s = driver_.readStatus();

    if (s.fault)
    {
      ROS_WARN_THROTTLE(1.0, "[GripperNode] Gripper fault code: 0x%02X", s.fault);
    }

    status_pub_.publish(toMsg(s));
  }

  // ── Members ────────────────────────────────────────────────────────────────
  Robotiq2F85Driver driver_;
  ros::NodeHandle&  nh_;
  ros::Publisher    status_pub_;
  ros::ServiceServer srv_activate_, srv_move_, srv_open_, srv_close_, srv_stop_;
  ros::Timer        status_timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotiq_2f85_node");

  // Private node handle — all topics/services are under ~/<param>
  ros::NodeHandle nh("~");

  std::string port;
  int    baudrate, slave_id;
  double status_rate;
  bool   auto_activate;

  nh.param<std::string>("port",          port,          "/dev/ttyUSB0");
  nh.param<int>        ("baudrate",      baudrate,      115200);
  nh.param<int>        ("slave_id",      slave_id,      9);
  nh.param<double>     ("status_rate",   status_rate,   50.0);
  nh.param<bool>       ("auto_activate", auto_activate, true);

  // Use 2 threads so service calls are not blocked by the status timer and
  // vice-versa.  The driver's internal mutex handles concurrent Modbus access.
  ros::AsyncSpinner spinner(2);
  spinner.start();

  try
  {
    GripperNode node(nh, port, baudrate, slave_id, status_rate, auto_activate);
    ros::waitForShutdown();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("[GripperNode] Fatal error: %s", e.what());
    return 1;
  }

  return 0;
}
