#!/usr/bin/env python3
"""
gripper_pulse.py

Waits for Enter, then fires a rapid open→close pulse:
  - Sends open goal  (1.45 cm) non-blocking
  - Waits half the pulse window
  - Sends close goal (1.30 cm) non-blocking
  Total command-send window: ~50 ms (configurable via ~pulse_ms)

Because both goals are non-blocking (send_goal without wait), they are
dispatched as fast as Python allows. The second goal preempts the first
on the action server — the gripper will attempt to reach 1.45 cm and
immediately be redirected to 1.30 cm.

Parameters (private, prefix ~):
    ~open_pos   [m]  default 0.0145  (1.45 cm)
    ~close_pos  [m]  default 0.0130  (1.30 cm)
    ~speed      [m/s] default 0.1    (max speed for tightest pulse)
    ~force      [%]  default 20.0
    ~pulse_ms   [ms] default 50      (gap between open and close sends)
"""

import time
import rospy
import actionlib

from robotiq_2f_gripper_msgs.msg import (
    CommandRobotiqGripperAction,
    CommandRobotiqGripperGoal,
)


def make_goal(position, speed, force):
    g = CommandRobotiqGripperGoal()
    g.emergency_release     = False
    g.emergency_release_dir = 0
    g.stop                  = False
    g.position              = position
    g.speed                 = speed
    g.force                 = force
    return g


def main():
    rospy.init_node("gripper_pulse", disable_signals=True)

    open_pos  = rospy.get_param("~open_pos",  0.0145)   # m
    close_pos = rospy.get_param("~close_pos", 0.0130)   # m
    speed     = rospy.get_param("~speed",     0.1)      # m/s  (max = fastest)
    force     = rospy.get_param("~force",     20.0)     # %
    pulse_ms  = rospy.get_param("~pulse_ms",  50.0)     # ms

    pulse_sec = pulse_ms / 1000.0

    client = actionlib.SimpleActionClient(
        "command_robotiq_action",
        CommandRobotiqGripperAction,
    )
    rospy.loginfo("Waiting for Robotiq action server...")
    client.wait_for_server()
    rospy.loginfo("Ready. Press Enter to pulse (Ctrl-C to quit).")

    open_goal  = make_goal(open_pos,  speed, force)
    close_goal = make_goal(close_pos, speed, force)

    while not rospy.is_shutdown():
        try:
            input("")   # blocks until Enter
        except (EOFError, KeyboardInterrupt):
            break

        # ── Fire open → close within pulse_ms window ──────────────────────
        t0 = time.monotonic()

        client.send_goal(open_goal)                         # non-blocking
        rospy.loginfo("OPEN  sent  (%.4f cm)", open_pos * 100.0)

        # busy-wait for the remainder of the half-window
        half = pulse_sec / 2.0
        while (time.monotonic() - t0) < half:
            pass

        t1 = time.monotonic()
        client.send_goal(close_goal)                        # non-blocking, preempts open
        rospy.loginfo("CLOSE sent  (%.4f cm)  |  inter-send gap: %.1f ms",
                      close_pos * 100.0,
                      (t1 - t0) * 1000.0)

        rospy.loginfo("Pulse dispatched. Press Enter for next pulse.")

    rospy.loginfo("Shutting down.")


if __name__ == "__main__":
    main()
