#!/usr/bin/env python3

#あらかじめ実行すること
#(1) ~/RaspberryPiMouse/utils/build_install.bash
#(2) ros2 launch raspimouse raspimouse.launch.py

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState
from std_srvs.srv import SetBool

from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
import datetime

class Drone(Node):
    def __init__(self):
        super().__init__('receiver')

        self.data = Twist()
        self.data.linear.x = 0.0
        self.data.angular.z = 0.0
        self.previous_control_time = datetime.datetime.now()
        self.subscription = self.create_subscription(String, 'controller', self.set_twist, 1)
        self.subscription

        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run)

    def motor_start(self):
        self._client_get_state = self.create_client(GetState, 'raspimouse/get_state')
        while not self._client_get_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(self._client_get_state.srv_name + ' service not available')

        self._client_change_state = self.create_client(ChangeState, 'raspimouse/change_state')
        while not self._client_change_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(self._client_change_state.srv_name + ' service not available')
        self._activate_raspimouse()

        self._client_motor_power = self.create_client(SetBool, 'motor_power')
        while not self._client_motor_power.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(self._client_motor_power.srv_name + ' service not available')
        self._motor_on()

    def motor_stop(self):
        self._motor_off()
        self._set_mouse_lifecycle_state(Transition.TRANSITION_DEACTIVATE)

    def _activate_raspimouse(self):
        self._set_mouse_lifecycle_state(Transition.TRANSITION_CONFIGURE)
        self._set_mouse_lifecycle_state(Transition.TRANSITION_ACTIVATE)
        self.get_logger().info('Mouse state is '
                               + self._get_mouse_lifecycle_state())
    def _set_mouse_lifecycle_state(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self._client_change_state.call_async(request)
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        rclpy.spin_until_future_complete(self, future, executor=executor)
        return future.result().success

    def _get_mouse_lifecycle_state(self):
        future = self._client_get_state.call_async(GetState.Request())
        executor = rclpy.executors.SingleThreadedExecutor(context=self.context)
        rclpy.spin_until_future_complete(self, future, executor=executor)
        return future.result().current_state.label

    def _motor_request(self, request_data=False):
        request = SetBool.Request()
        request.data = request_data
        self._client_motor_power.call_async(request)

    def _motor_on(self):
        self._motor_request(True)

    def _motor_off(self):
        self._motor_request(False)

    def set_twist(self, data):
        rev = data.data.split(',')
        self.data.linear.x = float(rev[0])
        self.data.angular.z = float(rev[1])
        self.previous_control_time = datetime.datetime.now()

    def run(self, reset=False):
        if datetime.datetime.now() > self.previous_control_time + datetime.timedelta(seconds=0.3):
            self.data.linear.x = 0.0
            self.data.angular.z = 0.0
        if reset:
            self.data.linear.x = 0.0
            self.data.angular.z = 0.0           
        self.get_logger().info(f"x:{self.data.linear.x}, z:{self.data.angular.z}")
        try:
            self.pub.publish(self.data)
        except:
            self.data.linear.x = 0.0
            self.data.angular.z = 0.0
            self.pub.publish(self.data)
 
def main(args=None):
    rclpy.init(args=args)
    drone = Drone()
    drone.motor_start()
    try:
        rclpy.spin(drone)
    except KeyboardInterrupt:
        pass
    drone.motor_stop()
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
