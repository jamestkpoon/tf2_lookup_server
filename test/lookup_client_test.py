import json
import sys
from os import getpid

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration, Time
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_msgs.action import LookupTransform


class TFLookupTestClient(Node):
    def __init__(self):
        super().__init__("tf_lookup_test_client_" + str(getpid()))
        self.client_ = ActionClient(self, LookupTransform, "tf_lookup")

        self.send_goal_future_ = None
        self.get_result_future_ = None

    def query(self, goal: LookupTransform.Goal):
        self.client_.wait_for_server()
        self.send_goal_future_ = self.client_.send_goal_async(goal)
        self.send_goal_future_.add_done_callback(self._goal_response_callback)

        self.get_logger().info("sent")

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self.get_result_future_ = goal_handle.get_result_async()
        self.get_result_future_.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        else:
            self.get_logger().error("Goal failed with status: {0}".format(status))

        print(str(result))

        rclpy.shutdown()


def main(args=None):
    req_cfg = json.load(open(sys.argv[1], "rb"))
    req_cfg["source_time"] = Time(**req_cfg["source_time"])
    req_cfg["timeout"] = Duration(**req_cfg["timeout"])

    rclpy.init(args=args)
    client = TFLookupTestClient()
    client.query(LookupTransform.Goal(**req_cfg))
    rclpy.spin(client)


if __name__ == "__main__":
    main()
