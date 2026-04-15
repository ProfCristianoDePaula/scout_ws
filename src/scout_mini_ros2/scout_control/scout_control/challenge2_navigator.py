#!/usr/bin/env python3
"""
challenge2_navigator.py
=======================
Scout Mini – Challenge 2: navegação autônoma no corredor DC com desvio de caixas.

Estratégia:
  - Estado LIVRE: avança em direção ao goal com correção proporcional de heading
  - Estado EVITANDO: gira em direção comprometida por tempo mínimo (MIN_AVOID_TIME)
    e só sai quando frente E lado original do obstáculo estiverem livres
  - Detecta e estima pose das 3 caixas via clusterização euclidiana do LaserScan

Authors: AMR Course UFSCar 2026
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ObstacleAvoidanceNode(Node):

    def __init__(self) -> None:
        super().__init__('challenge2_navigator')

        # ── I/O ────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # ── Robot state ─────────────────────────────────────────────────
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0
        self.goal_reached: bool = False

        # ── Goal ────────────────────────────────────────────────────────
        # odom inicializa em yaw=0 (ignora spawn yaw=pi) -> frente = odom +x
        self.GOAL_X: float = 13.2
        self.GOAL_Y: float = 0.0
        self.GOAL_TOL: float = 0.60   # metros

        # ── Velocidades ─────────────────────────────────────────────────
        self.LINEAR_SPEED: float = 0.30    # m/s cruzeiro
        self.HEADING_KP: float = 1.20      # ganho proporcional de heading

        # ── Limiares de obstáculo ───────────────────────────────────────
        self.FRONT_STOP: float = 0.70      # muito perto – para e gira
        self.FRONT_SLOW: float = 1.50      # obstáculo – desacelera e desvia
        self.SIDE_DANGER: float = 0.50     # lado perigosamente perto
        self.WALL_MIN: float = 0.60        # distância mínima das paredes

        # ── Recuperação de encurralamento ──────────────────────────────
        self._stuck_count: int = 0         # ciclos consecutivos em CRITICO
        self.STUCK_LIMIT: int = 8          # ciclos para ativar ré

        # ── Detecção de caixas ──────────────────────────────────────────
        self._box_poses: list = []         # (cx, cy) sensor frame
        self._logged_boxes: bool = False

        self.get_logger().info(
            f'Iniciado | Goal=({self.GOAL_X}, {self.GOAL_Y}) '
            f'tol={self.GOAL_TOL}m front_slow={self.FRONT_SLOW}m'
        )

    # ── Odom callback ───────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    # ── Scan callback (lógica principal) ────────────────────────────────

    def _scan_cb(self, msg: LaserScan) -> None:
        if self.goal_reached:
            return

        laser = msg.ranges
        N = len(laser)
        if N == 0:
            return

        # Centro do scan (frente do robô)
        center = round(-msg.angle_min / msg.angle_increment)
        center = max(30, min(N - 30, center))

        # Log único com info do sensor
        if not hasattr(self, '_sensor_logged'):
            self._sensor_logged = True
            self.get_logger().info(
                f'[SENSOR] N={N} '
                f'angle_min={math.degrees(msg.angle_min):.1f}deg '
                f'increment={math.degrees(msg.angle_increment):.2f}deg/ray '
                f'-> frente=idx[{center}]'
            )

        # Goal check
        dist_goal = math.hypot(self.x - self.GOAL_X, self.y - self.GOAL_Y)
        if dist_goal < self.GOAL_TOL:
            self._publish(0.0, 0.0)
            self.goal_reached = True
            self.get_logger().info(
                f'OBJETIVO ATINGIDO em ({self.x:.2f}, {self.y:.2f})'
            )
            self._log_boxes()
            return

        # Distâncias por setor (cone frontal estreito para reagir melhor)
        front = self._sector_min(laser, center - 15, center + 15)
        left  = self._sector_min(laser, center + 15, center + 60)
        right = self._sector_min(laser, center - 60, center - 15)

        # Estimativa de caixas
        self._detect_boxes(msg)

        # ── Controle reativo puro (sem máquina de estados) ─────────────
        lin = 0.0
        ang = 0.0
        side_close = min(left, right)

        if front < self.FRONT_STOP or side_close < self.SIDE_DANGER:
            self._stuck_count += 1
            if self._stuck_count > self.STUCK_LIMIT:
                # Encurralado: ré + gira para criar espaço
                lin = -0.20
                ang = 0.8 if left > right else -0.8
                self.get_logger().info(
                    f'RE      | frente={front:.2f}m esq={left:.2f}m dir={right:.2f}m',
                    throttle_duration_sec=0.5)
            else:
                # Muito perto: para e gira forte
                lin = 0.0
                ang = 1.0 if left > right else -1.0
                self.get_logger().info(
                    f'CRITICO | frente={front:.2f}m esq={left:.2f}m dir={right:.2f}m',
                    throttle_duration_sec=0.5)

        elif front < self.FRONT_SLOW:
            self._stuck_count = 0
            # Obstáculo à frente: avança devagar e desvia com força
            lin = 0.10
            ang = 0.8 if left > right else -0.8
            self.get_logger().info(
                f'DESVIO  | frente={front:.2f}m esq={left:.2f}m dir={right:.2f}m',
                throttle_duration_sec=0.5)

        else:
            self._stuck_count = 0
            # Caminho livre: cruzeiro em direção ao goal
            angle_to_goal = math.atan2(
                self.GOAL_Y - self.y, self.GOAL_X - self.x)
            heading_err = self._norm_angle(angle_to_goal - self.yaw)
            lin = self.LINEAR_SPEED
            ang = self._clamp(self.HEADING_KP * heading_err, -0.8, 0.8)
            self.get_logger().info(
                f'LIVRE   | goal={dist_goal:.1f}m frente={front:.2f}m '
                f'heading={math.degrees(heading_err):.1f}deg',
                throttle_duration_sec=1.0)

        # Correção de parede (sempre ativa)
        if left < self.WALL_MIN:
            ang = self._clamp(ang - 0.5, -1.2, 1.2)
        if right < self.WALL_MIN:
            ang = self._clamp(ang + 0.5, -1.2, 1.2)

        self._publish(lin, ang)

    # ── Detecção de caixas ──────────────────────────────────────────────

    def _detect_boxes(self, msg: LaserScan) -> None:
        """Clusterização euclidiana para detectar caixas no scan."""
        clusters: list = []
        current: list = []

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r > 5.0:
                if current:
                    clusters.append(current)
                    current = []
                continue
            a = msg.angle_min + i * msg.angle_increment
            pt = (r * math.cos(a), r * math.sin(a))
            if not current:
                current.append(pt)
            elif math.hypot(pt[0] - current[-1][0], pt[1] - current[-1][1]) < 0.25:
                current.append(pt)
            else:
                clusters.append(current)
                current = [pt]
        if current:
            clusters.append(current)

        poses = []
        for c in clusters:
            if len(c) >= 5:
                cx = sum(p[0] for p in c) / len(c)
                cy = sum(p[1] for p in c) / len(c)
                poses.append((cx, cy))

        if poses:
            self._box_poses = poses

    def _log_boxes(self) -> None:
        if self._logged_boxes:
            return
        self._logged_boxes = True
        self.get_logger().info(
            f'=== RELATORIO DE CAIXAS: {len(self._box_poses)} detectadas ==='
        )
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        for i, (sx, sy) in enumerate(self._box_poses, 1):
            # Transforma sensor frame -> odom frame
            ox = self.x + cos_y * sx - sin_y * sy
            oy = self.y + sin_y * sx + cos_y * sy
            self.get_logger().info(
                f'  Caixa {i}: odom ({ox:.2f}, {oy:.2f}) '
                f'| sensor ({sx:.2f}, {sy:.2f})'
            )

    # ── Utilitários ─────────────────────────────────────────────────────

    def _publish(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    @staticmethod
    def _sector_min(laser: list, i0: int, i1: int) -> float:
        N = len(laser)
        i0, i1 = max(0, i0), min(N, i1)
        valid = [r for r in laser[i0:i1] if math.isfinite(r) and 0.1 < r < 7.0]
        return min(valid) if valid else 10.0

    @staticmethod
    def _norm_angle(a: float) -> float:
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
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
