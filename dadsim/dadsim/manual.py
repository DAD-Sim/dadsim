import rclpy
from rclpy.node import Node
from rclpy.time import Time, Duration
from dadsim_agents_py.car_agent import CarNode
from dadsim_mapd_interfaces.msg import RoadCoord
from std_msgs.msg import ColorRGBA
from ackermann_msgs.msg import AckermannDrive
import argparse
import sys


class Manual(CarNode):
    def __init__(
        self,
        name: str,
        init_position: RoadCoord,
        init_heading: float = 0.0,
        weelbase: float = 2.8,
        width: float = 1.8,
        length: float = 4.0,
        height: float = 1.6,
        color: ColorRGBA = ColorRGBA(r=0.9, g=0.6, b=0.7, a=0.6),
    ):
        super().__init__(name, init_position, init_heading, weelbase, width, length, height, color)
        self.drive = AckermannDrive()
        self.create_subscription(
            AckermannDrive, "/ackermann_drive", self.drive_msg_callback, 10
        )

    def get_drive(self, time: Time):
        return self.drive

    def drive_msg_callback(self, msg):
        self.drive = msg


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("road", type=str)
    parser.add_argument("s", type=float)
    parser.add_argument("d", type=float)
    argv = parser.parse_args(
        (
            sys.argv[1 : sys.argv.index("--ros-args")]
            + sys.argv[sys.argv.index("--") + 1 :]
            if "--" in sys.argv
            else sys.argv[1 : sys.argv.index("--ros-args")]
        )
        if "--ros-args" in sys.argv
        else sys.argv[1:]
    )
    agent_node = Manual(
        "manual",
        RoadCoord(road_id=argv.road, s=argv.s, d=argv.d, h=0.0),
        color=ColorRGBA(r=0.1, g=1.0, b=0.05, a=0.6),
    )
    rclpy.spin(agent_node)
    agent_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
