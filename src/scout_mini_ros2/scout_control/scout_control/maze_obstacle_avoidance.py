#!/usr/bin/env python3
"""
maze_obstacle_avoidance.py
==========================
Scout Mini - Class 05: Obstacle Avoidance (maze)

Reactive goal-directed navigation – NO fixed waypoints.
Single goal defined relative to the starting position (captured at runtime).

Key fixes vs previous version:
  - FRONT_AVOID restored to 1.50 m (2.20 was too wide for narrow corridors)
  - critical_locked: avoid_dir is computed ONCE when entering CRITICAL and held
    until the robot leaves the critical zone, preventing L/R oscillation in
    symmetric tight spaces
  - RECOVER uses pure rotation (V=0), not backward motion – going backward in
    a corridor just re-triggers CRITICAL from behind
  - RECOVERY_CYCLES=18 gives enough time to rotate clear of the corner

Requirements covered:
    - Subscribe /scan (LaserScan)
    - Subscribe /odom (pose tracking)
    - Publish   /cmd_vel (Twist)
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ObstacleAvoidanceNode(Node):

    # ── Single relative goal (top-right corner of the maze) ───────────────
    # Positive X  = robot forward direction at startup.
    # Positive Y  = left of robot at startup.
    # Adjust only these two values if the robot stops at the wrong place.
    GOAL_REL_X = 15.0   # metres
    GOAL_REL_Y = 13.2   # metres
    GOAL_TOL   =  0.45  # acceptance radius (m)

    # ── Safety distances ──────────────────────────────────────────────────
    FRONT_CRITICAL = 0.65   # stop & rotate
    FRONT_AVOID    = 1.50   # slow & turn
    SIDE_CRITICAL  = 0.45   # lateral wall too close
    WALL_SOFT      = 0.70   # soft repulsion zone

    # ── Velocities / gains ───────────────────────────────────────────────
    V_CRUISE    =  0.28
    V_AVOID_MAX =  0.18
    V_AVOID_MIN =  0.06
    W_CRITICAL  =  1.10
    W_AVOID_BASE=  0.80
    W_RECOVER   =  1.20   # pure-rotation angular speed
    W_AVOID_TURN=  1.20   # dedicated in-place turn speed for AVOID 90°
    K_HEADING   =  1.10
    AVOID_TURN_ANGLE = math.pi / 2.0
    AVOID_YAW_TOL    = 0.01
    AVOID_STOP_CYCLES = 0
    V_POST_TURN = 0.16
    POST_TURN_FORWARD_CYCLES = 10
    POST_AVOID_HEADING_HOLD = 12

    # ── Anti-stuck ────────────────────────────────────────────────────────
    STUCK_LIMIT     =  8   # CRITICAL cycles before forcing RECOVER
    RECOVERY_CYCLES = 18   # pure rotation cycles

    # ─────────────────────────────────────────────────────────────────────
    def __init__(self) -> None:
        super().__init__('maze_obstacle_avoidance')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        self.x:    float = 0.0
        self.y:    float = 0.0
        self.yaw:  float = 0.0
        self.start_x: float | None = None
        self.start_y: float | None = None

        self.goal_reached:    bool  = False
        self.avoid_dir:       float = 1.0
        self.critical_locked: bool  = False  # locks avoid_dir during CRITICAL
        self.stuck_count:     int   = 0
        self.recovery_left:   int   = 0
        self.avoid_zone_active: bool  = False
        self.avoid_turning:     bool  = False
        self.avoid_turn_done:   bool  = False
        self.avoid_stop_left:   int   = 0
        self.avoid_target_yaw:  float = 0.0
        self.avoid_turn_prev_yaw: float = 0.0
        self.avoid_turn_accum:    float = 0.0
        self.post_turn_left:      int   = 0
        self.heading_hold_left:   int   = 0

        self.get_logger().info(
            'ObstacleAvoidanceNode ready | single-goal reactive mode'
        )

    # ── Odometry callback ────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y
            self.get_logger().info(
                f'Origin locked at ({self.x:.2f}, {self.y:.2f})'
            )

    # ── LaserScan callback ───────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan) -> None:
        if self.goal_reached:
            self._publish(0.0, 0.0)
            return
        if self.start_x is None:
            return

        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        c = int(round(-msg.angle_min / msg.angle_increment))
        c = max(1, min(n - 1, c))

        # Convert desired angles to index offsets (robust to any LiDAR resolution)
        def deg2idx(d: float) -> int:
            return max(1, int(round(math.radians(d) / msg.angle_increment)))

        front       = self._sector_min(ranges, c - deg2idx(15),  c + deg2idx(15))
        front_left  = self._sector_min(ranges, c + deg2idx(5),   c + deg2idx(45))
        front_right = self._sector_min(ranges, c - deg2idx(45),  c - deg2idx(5))
        left        = self._sector_min(ranges, c + deg2idx(45),  c + deg2idx(120))
        right       = self._sector_min(ranges, c - deg2idx(120), c - deg2idx(45))

        goal_x = self.start_x + self.GOAL_REL_X
        goal_y = self.start_y + self.GOAL_REL_Y
        dist   = math.hypot(goal_x - self.x, goal_y - self.y)

        if dist < self.GOAL_TOL:
            self._publish(0.0, 0.0)
            self.goal_reached = True
            self.get_logger().info(
                f'*** GOAL REACHED at ({self.x:.2f}, {self.y:.2f}) ***'
            )
            return

        lin, ang, state = self._control(
            front, front_left, front_right, left, right, goal_x, goal_y
        )

        # Soft lateral repulsion only in FREE mode.
        # In AVOID/CRITICAL/RECOVER this may corrupt intentional turn maneuvers.
        if state == 'FREE':
            if left < self.WALL_SOFT:
                ang -= 0.40 * (self.WALL_SOFT - left) / self.WALL_SOFT
            if right < self.WALL_SOFT:
                ang += 0.40 * (self.WALL_SOFT - right) / self.WALL_SOFT

        ang = self._clamp(ang, -1.3, 1.3)
        self._publish(lin, ang)

        turn_info = f' ta={self.avoid_turn_accum:.2f}' if self.avoid_turning else ''
        self.get_logger().info(
            f'{state:<8} dist={dist:.2f} f={front:.2f} '
            f'fl={front_left:.2f} fr={front_right:.2f} '
            f'l={left:.2f} r={right:.2f} '
            f'v={lin:.2f} w={ang:.2f}{turn_info}',
            throttle_duration_sec=0.45,
        )

    # ── Main reactive controller ─────────────────────────────────────────
    def _control(
        self,
        front:       float,
        front_left:  float,
        front_right: float,
        left:        float,
        right:       float,
        goal_x:      float,
        goal_y:      float,
    ) -> tuple:

        # ── AVOID maneuver has top priority once armed ──────────────────
        # Phase 1: hard stop, Phase 2: rotate 90°, then release.
        if self.avoid_zone_active and not self.avoid_turn_done:
            if self.avoid_stop_left > 0:
                self.avoid_stop_left -= 1
                return 0.0, 0.0, 'AVOID'

            if not self.avoid_turning:
                d = self._choose_dir(front_left, front_right, left, right)
                self.avoid_dir = d
                self.avoid_target_yaw = self._norm(self.yaw + d * self.AVOID_TURN_ANGLE)
                self.avoid_turning = True
                self.avoid_turn_prev_yaw = self.yaw
                self.avoid_turn_accum = 0.0
                dir_txt = 'LEFT' if d > 0.0 else 'RIGHT'
                self.get_logger().info(
                    f'AVOID turn start: dir={dir_txt} fl={front_left:.2f} fr={front_right:.2f} l={left:.2f} r={right:.2f}'
                )

            dyaw = self._norm(self.yaw - self.avoid_turn_prev_yaw)
            progress = dyaw * self.avoid_dir
            if progress > 0.0:
                self.avoid_turn_accum += progress
            self.avoid_turn_prev_yaw = self.yaw

            if self.avoid_turn_accum >= (self.AVOID_TURN_ANGLE - self.AVOID_YAW_TOL):
                self.avoid_turning = False
                self.avoid_turn_done = True
                self.post_turn_left = self.POST_TURN_FORWARD_CYCLES
                self.heading_hold_left = self.POST_AVOID_HEADING_HOLD
                self.get_logger().info(
                    f'AVOID turn done (90 deg, accum={self.avoid_turn_accum:.2f} rad)'
                )
                return self.V_POST_TURN, 0.0, 'AVOID'

            return 0.0, self.avoid_dir * self.W_AVOID_TURN, 'AVOID'

        # ── RECOVER: pure rotation until counter expires ─────────────────
        if self.recovery_left > 0:
            self.recovery_left -= 1
            return 0.0, self.avoid_dir * self.W_RECOVER, 'RECOVER'

        # ── CRITICAL: too close to obstacle/wall ─────────────────────────
        if front < self.FRONT_CRITICAL or min(front_left, front_right) < self.SIDE_CRITICAL:
            # Compute direction ONLY on first entry; hold it until zone is clear.
            if not self.critical_locked:
                self.avoid_dir = self._choose_dir(
                    front_left, front_right, left, right)
                self.critical_locked = True

            self.stuck_count += 1
            if self.stuck_count >= self.STUCK_LIMIT:
                self.recovery_left = self.RECOVERY_CYCLES
                self.stuck_count   = 0
                return 0.0, self.avoid_dir * self.W_RECOVER, 'RECOVER'

            return 0.0, self.avoid_dir * self.W_CRITICAL, 'CRITICAL'

        # ── Left critical zone → release lock ────────────────────────────
        self.critical_locked = False

        # ── AVOID: obstacle within avoidance range ───────────────────────
        # Stop completely and turn toward the side with more free space (l vs r).
        if front < self.FRONT_AVOID:
            self.stuck_count = 0
            if not self.avoid_zone_active:
                self.avoid_zone_active = True
                self.avoid_turn_done = False
                self.avoid_turning = True
                self.avoid_stop_left = self.AVOID_STOP_CYCLES
                d = self._choose_dir(front_left, front_right, left, right)
                self.avoid_dir = d
                self.avoid_target_yaw = self._norm(self.yaw + d * self.AVOID_TURN_ANGLE)
                self.avoid_turn_prev_yaw = self.yaw
                self.avoid_turn_accum = 0.0
                dir_txt = 'LEFT' if d > 0.0 else 'RIGHT'
                self.get_logger().info(
                    f'AVOID turn start: dir={dir_txt} fl={front_left:.2f} fr={front_right:.2f} l={left:.2f} r={right:.2f}'
                )
                return 0.0, d * self.W_AVOID_TURN, 'AVOID'

            if not self.avoid_turn_done:
                return 0.0, 0.0, 'AVOID'

            # After completing 90°, commit to a short straight segment to
            # clear the corner before any extra turning.
            if self.post_turn_left > 0:
                self.post_turn_left -= 1
                return self.V_POST_TURN, 0.0, 'AVOID'

            return self.V_AVOID_MIN, self.avoid_dir * 0.20, 'AVOID'

        # Left AVOID zone → arm next 90-degree maneuver.
        self.avoid_zone_active = False
        self.avoid_turning = False
        self.avoid_turn_done = False
        self.avoid_stop_left = 0
        self.avoid_turn_accum = 0.0

        # ── FREE: clear path, steer toward goal ──────────────────────────
        self.stuck_count = 0
        if self.heading_hold_left > 0:
            self.heading_hold_left -= 1
            return self.V_CRUISE, 0.0, 'FREE'

        heading = math.atan2(goal_y - self.y, goal_x - self.x)
        err = self._norm(heading - self.yaw)
        ang = self._clamp(self.K_HEADING * err, -0.65, 0.65)
        return self.V_CRUISE, ang, 'FREE'

    # ── Direction chooser ────────────────────────────────────────────────
    @staticmethod
    def _choose_dir(
        front_left:  float,
        front_right: float,
        left:        float,
        right:       float,
        goal_x:      float = 0.0,
        goal_y:      float = 0.0,
    ) -> float:
        # Choose purely by sensor score – more open side wins.
        # Goal bias was removed: it corrupted correct physical decisions
        # (e.g. biasing left when the open passage is on the right).
        # FREE mode already steers toward the goal via K_HEADING.
        left_score  = front_left  + 0.60 * left
        right_score = front_right + 0.60 * right
        return 1.0 if left_score >= right_score else -1.0

    # ── Helpers ──────────────────────────────────────────────────────────
    def _publish(self, linear: float, angular: float) -> None:
        cmd = Twist()
        cmd.linear.x  = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    @staticmethod
    def _sector_min(ranges: list, i0: int, i1: int) -> float:
        n = len(ranges)
        i0, i1 = max(0, min(i0, n)), max(0, min(i1, n))
        if i0 >= i1:
            return 10.0
        valid = [r for r in ranges[i0:i1] if math.isfinite(r) and 0.05 < r < 10.0]
        return min(valid) if valid else 10.0

    @staticmethod
    def _norm(a: float) -> float:
        while a >  math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
