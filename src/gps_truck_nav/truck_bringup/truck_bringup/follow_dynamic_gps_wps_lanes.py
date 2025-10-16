#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from math import sqrt


class LaneChangeController(Node):
    def __init__(self):
        super().__init__('lane_change_controller')

        # Navegador para enviar objetivos
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions
        self.lane_change_sub = self.create_subscription(
            Bool,
            '/lane_change_request',
            self.lane_change_callback,
            10
        )
        self.marker_array_sub = self.create_subscription(
            MarkerArray,
            'visualization_marker_array',
            self.marker_array_callback,
            10
        )

        # Variáveis internas
        self.waypoints = []
        self.current_waypoint_index = 0
        self.current_goal = None
        self.lane_change_requested = False

        self.get_logger().info("Lane Change Controller Initialized. Waiting for lane requests and waypoints...")

    def lane_change_callback(self, msg):
        if msg.data:
            self.lane_change_requested = True
            self.get_logger().info("Lane change request received. Cancelling current goal...")
            self.cancel_current_goal()

    def cancel_current_goal(self):
        """
        Cancela o objetivo atual usando o ActionClient.
        """
        self.action_client.wait_for_server()
        cancel_future = self.action_client.cancel_all_goals()
        rclpy.spin_until_future_complete(self, cancel_future)

        cancel_response = cancel_future.result()
        if cancel_response.goals_canceling:
            self.get_logger().info("Current goal successfully canceled.")
        else:
            self.get_logger().warn("Failed to cancel the current goal.")

        # Após cancelar, recalcular o caminho
        self.recalculate_route()

    def marker_array_callback(self, msg):
        """
        Atualiza os waypoints com base nos markers recebidos.
        """
        self.get_logger().info("Received updated waypoints.")
        self.waypoints = [
            PoseStamped(
                header=marker.header,
                pose=marker.pose
            ) for marker in msg.markers
        ]
        self.get_logger().info(f"{len(self.waypoints)} waypoints loaded.")

        # Iniciar navegação, se necessário
        if not self.lane_change_requested:
            self.navigate_to_next_waypoint()

    def navigate_to_next_waypoint(self):
        """
        Navega para o próximo waypoint na lista.
        """
        if self.current_waypoint_index < len(self.waypoints):
            self.current_goal = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index}...")

            # Envia o objetivo
            goal_future = self.action_client.send_goal_async(NavigateToPose.Goal(pose=self.current_goal))
            rclpy.spin_until_future_complete(self, goal_future)

            goal_handle = goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f"Goal {self.current_waypoint_index} was not accepted.")
                return

            self.get_logger().info(f"Goal {self.current_waypoint_index} accepted. Waiting for result...")
            self.current_waypoint_index += 1
        else:
            self.get_logger().info("All waypoints completed.")

    def recalculate_route(self):
        """
        Recalcula a rota para novos waypoints após mudança de faixa.
        """
        if self.waypoints and self.current_waypoint_index < len(self.waypoints):
            # Filtra waypoints ainda não visitados
            remaining_waypoints = self.waypoints[self.current_waypoint_index:]
            self.get_logger().info(f"{len(remaining_waypoints)} remaining waypoints after lane change.")

            # Atualiza a lista de waypoints
            self.waypoints = remaining_waypoints
            self.current_waypoint_index = 0
            self.lane_change_requested = False

            # Inicia navegação com os waypoints restantes
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().warn("No remaining waypoints to navigate after lane change.")


def main(args=None):
    rclpy.init(args=args)
    node = LaneChangeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
