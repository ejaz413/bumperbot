
import rclpy
import rclpy.executors
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String
import time

class SimpleLifecycleNode(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

    def on_configure(self, state:State) -> TransitionCallbackReturn:
        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, 10)
        self.get_logger().info("Lifecycle node on_configure() called")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state:State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.sub_)
        self.get_logger().info("Lifecycle node on_shutdown() called")
        return TransitionCallbackReturn.SUCCESS
    def on_cleanup(self, state:State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.sub_)
        self.get_logger().info("Lifecycle node on_cleanup() called")
        return TransitionCallbackReturn.SUCCESS
    def on_activate(self, state:State) -> TransitionCallbackReturn:
        
        self.get_logger().info("Lifecycle node on_activate() called")
        time.sleep(2)
        return super().on_activate(state)
    def on_deactivate(self, state:State) -> TransitionCallbackReturn:
        
        self.get_logger().info("Lifecycle node on_deactivate() called")
        time.sleep(2)
        return super().on_deactivate(state)
    
    def msgCallback(self, msg):
       current_state =  self._state_machine.current_state
       if current_state[1] == "active":
           self.get_logger.info("I heard: " %msg.data)


def main():
    rclpy.init()
    executer = rclpy.executors.SingleThreadedExecutor()
    simple_lifecycle_node = SimpleLifecycleNode("simple_life_cycle_node")
    executer.add_node(simple_lifecycle_node)
    try:
        executer.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        simple_lifecycle_node.destroy_node()
    
if __name__ == "__main__":
    main()