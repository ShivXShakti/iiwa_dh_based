import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class MyGenerator(Node):

    def __init__(self):
        super().__init__("my_generator")

        self.joint_subscriber_ = self.create_subscription(
            JointState, '/joint_states', self.cb_joint_sub, 10)

        self.joint_publisher_ = self.create_publisher(
            Float64MultiArray, '/forward_command_controller/commands', 10)

        # initializing parameters for initial angles

        self.joint_index = {}
        self.joint_index_initializer = False
        self.initial_angles = None
        self.initial_angle_recieved = False

        # initializing trajectory parameters

        self.desired_angles = np.array([0.0, 0.0, 90.0, 0.0, 0.0, 90.0, 0.0])
        self.final_angles = np.deg2rad(self.desired_angles)
        self.t = 0.0
        self.dt = 0.01
        self.Vmax = 5.0
        self.Amax = 1.0

        self.timer_ = self.create_timer(self.dt, self.cb_joint_pub)

    def cb_joint_sub(self, msg):

        if not self.joint_index_initializer:
            for i, name in enumerate(msg.name):
                self.joint_index[name] = i
            self.joint_index_initializer = True

        if not self.initial_angle_recieved:
            self.initial_angles = np.array([
                msg.position[self.joint_index[name]] for name in msg.name
            ])
            self.initial_angle_recieved = True

    def trajectory_generator(self):

        distances = self.final_angles - self.initial_angles
        max_distance = np.max(np.abs(distances))

        ratios = distances/max_distance

        t_acc = self.Vmax/self.Amax
        dist_acc_dec = self.Vmax * t_acc

        if max_distance < dist_acc_dec:
            t_acc = np.sqrt(max_distance/self.Amax)
            t_cruise = 0.0
        else:
            t_cruise = (max_distance-dist_acc_dec)/self.Vmax

        self.total_time = 2*t_acc + t_cruise

        if self.t > self.total_time:
            return None

        if self.t < t_acc:
            S = 0.5*self.Amax*self.t*self.t

        elif self.t < t_cruise+t_acc:
            S = 0.5*self.Amax*t_acc**2 + self.Vmax*(self.t - t_acc)

        else:
            t_dec = self.t - (t_acc + t_cruise)

            S = 0.5*self.Amax*t_acc**2 + self.Vmax*t_cruise + \
                self.Vmax*t_dec - 0.5*self.Amax*t_dec**2

        q_cmd = self.initial_angles + S*ratios
        self.t += self.dt

        return q_cmd

    def cb_joint_pub(self):

        if not self.initial_angle_recieved:
            return

        q_cmd = self.trajectory_generator()

        if q_cmd is None:
            self.get_logger().info(
                f"Trajectory Completed in {self.total_time}")
            self.timer_.cancel()
            return

        msg = Float64MultiArray()
        msg.data = q_cmd.tolist()
        self.joint_publisher_.publish(msg=msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyGenerator()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
