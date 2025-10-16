#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import math
import sys

class FollowDynamicLanesFromMarkerArrayWithOdom(Node):
    def __init__(self):
        super().__init__('follow_dynamic_lanes_from_marker_array_with_odom')

        # Cliente de ação para NavigateThroughPoses
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self._goal_handle = None

        self.lane_state_sub = self.create_subscription(
            Bool,
            '/binary_state',
            self.lane_state_callback,
            10
        )

        self.marker_array_sub = self.create_subscription(
            MarkerArray,
            'visualization_marker_array',
            self.marker_array_callback,
            10
        )

        # Subscreve a odometria global para obter posição do robô em map
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.odom_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_waypoints = []  # Lista de waypoints (x,y,yaw)
        self.navigation_enabled = True
        self.robot_map_pose = None  # (x,y,yaw) do robô no frame map, obtida via /odometry/global

        self.get_logger().info("Aguardando marker_array e pose do robô via odometria global...")

    def odom_callback(self, msg: Odometry):
        # Extrai pose do robô a partir da odometria
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Calcula yaw do quaternion
        siny_cosp = 2.0*(qw*qz+qx*qy)
        cosy_cosp = 1.0-2.0*(qy*qy+qz*qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.robot_map_pose = (x,y,yaw)
        self.get_logger().info(f"Posição atual do robô via odometria global: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.2f}°")

    def marker_array_callback(self, msg: MarkerArray):
        # Extrai waypoints do MarkerArray
        new_waypoints = []
        for marker in msg.markers:
            wp = {
                'x': marker.pose.position.x,
                'y': marker.pose.position.y,
                'yaw': 0.0  # se quiser, calcule yaw entre pontos consecutivos
            }
            new_waypoints.append(wp)

        if new_waypoints:
            filtered_wps = self.filter_waypoints(new_waypoints)
            self.current_waypoints = filtered_wps
            self.get_logger().info(f"Waypoints atualizados do marker_array: {len(self.current_waypoints)} após filtro.")

            if self.navigation_enabled and not self._goal_handle:
                self.send_goal(self.current_waypoints)

    def filter_waypoints(self, waypoints, min_dist=2.0):
        # Se não temos pose do robô, retorne todos
        if self.robot_map_pose is None:
            self.get_logger().warn("Pose do robô desconhecida (odom não disponível ainda?), sem filtrar waypoints.")
            return waypoints

        robot_x, robot_y, robot_yaw = self.robot_map_pose

        # Critério simples: remover waypoints muito próximos (dist<2m) ou atrás do robô (se considerarmos robô avançando no eixo X)
        # Caso não haja direcionalidade, pode simplesmente descartar waypoints com dist<2m.
        # Aqui, assumiremos que o robô se move principalmente no sentido crescente de X.
        filtered = []
        for wp in waypoints:
            dx = wp['x'] - robot_x
            dy = wp['y'] - robot_y
            dist = math.sqrt(dx*dx+dy*dy)
            # Mantém se dist>min_dist e wp.x>robot_x (robô anda aumentando X)
            if dist>min_dist and wp['x']>robot_x:
                filtered.append(wp)

        return filtered

    def lane_state_callback(self, msg: Bool):
        if not msg.data and self.navigation_enabled:
            self.get_logger().warn("Faixa não navegável! Cancelando objetivo e reenviando rota filtrada.")
            self.cancel_all_goals()

    def cancel_all_goals(self):
        if self._goal_handle:
            self.get_logger().info("Tentando cancelar todos os objetivos de navegação...")
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_all_goals_callback)
        else:
            self.get_logger().info("Nenhum objetivo ativo para cancelar. Reenviando rota filtrada.")
            self.resend_waypoints()

    def cancel_all_goals_callback(self, future):
        resp = future.result()
        if resp and len(resp.goals_canceling)>0:
            self.get_logger().info("Objetivos cancelados. Parando o robô...")
            self.stop_robot()
            # Filtra novamente considerando pose atual do robô
            filtered_wps = self.filter_waypoints(self.current_waypoints)
            self.current_waypoints = filtered_wps
            self.resend_waypoints()
        else:
            self.get_logger().warn("Falha ao cancelar objetivos.")
        self._goal_handle=None

    def resend_waypoints(self):
        if self.current_waypoints:
            self.get_logger().info(f"Reenviando {len(self.current_waypoints)} waypoints após cancelamento.")
            self.send_goal(self.current_waypoints)
        else:
            self.get_logger().warn("Sem waypoints disponíveis para reenviar.")

    def send_goal(self, waypoints):
        if not waypoints:
            self.get_logger().warn("Nenhum waypoint para enviar ao NavigateThroughPoses.")
            return

        self.get_logger().info("Enviando objetivo para NavigateThroughPoses (filtrados):")
        for i,wp in enumerate(waypoints):
            self.get_logger().info(f"Wp {i}: x={wp['x']:.3f}, y={wp['y']:.3f}, yaw={math.degrees(wp['yaw']):.2f}°")

        goal_msg = NavigateThroughPoses.Goal(poses=self.convert_to_poses(waypoints))
        self._action_client.wait_for_server()
        self.get_logger().info("Chamando NavigateThroughPoses...")
        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def convert_to_poses(self, wps):
        from geometry_msgs.msg import PoseStamped
        poses=[]
        for wp in wps:
            ps = PoseStamped()
            ps.header.frame_id='map'
            ps.header.stamp=self.get_clock().now().to_msg()
            ps.pose.position.x=wp['x']
            ps.pose.position.y=wp['y']
            yaw=wp['yaw']
            qz=math.sin(yaw/2)
            qw=math.cos(yaw/2)
            ps.pose.orientation.z=qz
            ps.pose.orientation.w=qw
            poses.append(ps)
        return poses

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().info("Objetivo rejeitado.")
            self._goal_handle=None
            return
        self.get_logger().info("Objetivo aceito. Aguardando resultado...")
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Resultado da navegação: {result.error_code}")
        self._goal_handle = None

    def stop_robot(self):
        twist=Twist()
        twist.linear.x=0.0
        twist.angular.z=0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Robô parado.")

def main(args=None):
    rclpy.init(args=args)
    node = FollowDynamicLanesFromMarkerArrayWithOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
