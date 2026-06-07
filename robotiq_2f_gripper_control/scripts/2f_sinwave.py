#!/usr/bin/env python3

import math
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from robotiq_2f_gripper_msgs.msg import (
    CommandRobotiqGripperAction,
    CommandRobotiqGripperGoal,
    CommandRobotiqGripperFeedback,
)


def main():
    rospy.init_node("sin_gripper_gap_commander")

    # Sin wave parameters
    min_gap = rospy.get_param("~min_gap", 0.02)      # 2 cm
    max_gap = rospy.get_param("~max_gap", 0.04)      # 3 cm
    frequency = rospy.get_param("~frequency", 0.2)   # Hz
    publish_rate = rospy.get_param("~rate", 0.8)    # Hz

    speed = rospy.get_param("~speed", 0.05)          # m/s
    force = rospy.get_param("~force", 10.0)          # percent

    center = 0.5 * (min_gap + max_gap)               # 0.03 m
    amplitude = 0.5 * (max_gap - min_gap)            # 0.01 m

    client = actionlib.SimpleActionClient(
        "command_robotiq_action",
        CommandRobotiqGripperAction
    )

    rospy.loginfo("Waiting for Robotiq action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to Robotiq action server.")

    # ── Current-spike latency tracker ─────────────────────────────────────────
    # T1 is recorded just before send_goal().
    # T2 is recorded in the feedback callback the first time current exceeds
    # `current_threshold` after a new goal is sent.  This detects motor
    # energisation before any position change is visible.
    current_threshold = rospy.get_param("~current_threshold", 0.5)  # [A]

    # Shared mutable state between main loop and feedback callback.
    # Using a one-element list so the nested function can rebind the value.
    t1_holder      = [None]   # timestamp of the last send_goal()
    spike_armed    = [False]  # True while we are waiting for the spike
    spike_latencies = []

    def feedback_cb(fb):
        """Called by actionlib every time the action server publishes feedback."""
        if not spike_armed[0] or t1_holder[0] is None:
            return
        if fb.current > current_threshold:
            t2 = rospy.Time.now().to_sec()
            latency = t2 - t1_holder[0]
            spike_armed[0] = False          # disarm until next send_goal()
            spike_latencies.append(latency)
            avg = sum(spike_latencies) / len(spike_latencies)
            rospy.loginfo(
                "[current-spike latency] %.4f s | avg=%.4f s | min=%.4f s | max=%.4f s | n=%d",
                latency, avg,
                min(spike_latencies), max(spike_latencies),
                len(spike_latencies),
            )

    # ── Goal-reached latency tracking (existing) ──────────────────────────────
    latencies = []

    def make_done_cb(t_sent, target_gap):
        """
        Returns a per-goal done callback. The default-argument trick (t0=t_sent)
        captures this goal's send time by value, independent of later iterations.
        Only SUCCEEDED goals (position reached or object detected) are counted;
        PREEMPTED goals are discarded because a newer goal replaced them.
        """
        def done_cb(state, result, t0=t_sent, pos=target_gap):
            if state == GoalStatus.SUCCEEDED:
                latency = rospy.Time.now().to_sec() - t0
                latencies.append(latency)
                avg = sum(latencies) / len(latencies)
                rospy.loginfo(
                    "[latency] pos=%.3f m | %.3f s | avg=%.3f s | min=%.3f s | max=%.3f s | n=%d",
                    pos, latency, avg, min(latencies), max(latencies), len(latencies)
                )
        return done_cb

    # ── Main loop ─────────────────────────────────────────────────────────────
    rate = rospy.Rate(publish_rate)
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time

        # gap oscillates between min_gap and max_gap
        gap = center + amplitude * math.sin(2.0 * math.pi * frequency * t)

        goal = CommandRobotiqGripperGoal()
        goal.emergency_release = False
        goal.emergency_release_dir = 0
        goal.stop = False
        goal.position = gap
        goal.speed = speed
        goal.force = force

        # T1: arm the spike detector before sending so no feedback sample is missed
        t1_holder[0]   = rospy.Time.now().to_sec()
        spike_armed[0] = True
        t_sent = t1_holder[0]
        client.send_goal(goal,
                         done_cb=make_done_cb(t_sent, gap),
                         feedback_cb=feedback_cb)

        rospy.loginfo_throttle(
            0.5,
            "Commanded gripper gap: %.3f m = %.1f cm",
            gap,
            gap * 100.0
        )

        rate.sleep()

    # ── Final summaries on Ctrl-C ────────────────────────────────────────────
    if spike_latencies:
        rospy.loginfo(
            "=== Current-spike latency: %d samples | avg=%.4f s | min=%.4f s | max=%.4f s ===",
            len(spike_latencies),
            sum(spike_latencies) / len(spike_latencies),
            min(spike_latencies),
            max(spike_latencies),
        )
    else:
        rospy.logwarn("No current spike exceeded threshold=%.1f A — check ~current_threshold",
                      current_threshold)

    if latencies:
        rospy.loginfo(
            "=== Latency summary: %d goals | avg=%.3f s | min=%.3f s | max=%.3f s ===",
            len(latencies),
            sum(latencies) / len(latencies),
            min(latencies),
            max(latencies),
        )
    else:
        rospy.logwarn(
            "No goals reached SUCCEEDED — all were preempted. "
            "Lower ~rate or ~frequency so goals have time to complete."
        )


if __name__ == "__main__":
    main()
