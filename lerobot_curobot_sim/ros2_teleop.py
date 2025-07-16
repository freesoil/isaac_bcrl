import carb
import rclpy
from rclpy.node import Node

# Hangi mesaj tipini kullanacağımıza karar ver
USE_SENSOR_JOINT_STATE = False  # gerekirse True yap

try:
    from isaac_ros2_bridge_msgs.msg import IsaacJointCommand
    HAVE_IJC = True
except ImportError:
    HAVE_IJC = False

from sensor_msgs.msg import JointState

# >>> KULLANICI AYARLARI <<<
TOPIC_NAME = "/isaac_joint_commands"   # Robostack'taki yayın
ART_ROOT   = "/SOARM100"   # <-- kendi root yolunu yaz
JOINT_ORDER = [
    "Rotation",
    "Pitch",
    "Elbow",
    "Wirst_Roll",
    "Wirst_Pitch",
    "Jaw",
]  # Stage'den gerçek eklem adların
SCALE = 1.0

# Isaac API
from omni.isaac.core import SimulationContext
from omni.isaac.core.articulations import ArticulationView

class ArmCmdSub(Node):
    def __init__(self, art):
        super().__init__("arm_cmd_sub")
        self.art = art
        self.q = [0.0] * len(JOINT_ORDER)

        if HAVE_IJC and not USE_SENSOR_JOINT_STATE:
            self.get_logger().info(f"Subscribing to IsaacJointCommand on {TOPIC_NAME}")
            self.create_subscription(IsaacJointCommand, TOPIC_NAME, self.cb_ijc, 10)
        else:
            self.get_logger().info(f"Subscribing to sensor_msgs/JointState on {TOPIC_NAME}")
            self.create_subscription(JointState, TOPIC_NAME, self.cb_js, 10)

    def cb_ijc(self, msg):
        self._map_and_apply(msg.joint_names, msg.positions)

    def cb_js(self, msg):
        self._map_and_apply(msg.name, msg.position)

    def _map_and_apply(self, names, positions):
        # Map incoming names to our JOINT_ORDER
        for i,jn in enumerate(JOINT_ORDER):
            try:
                idx = names.index(jn)
                self.q[i] = positions[idx] * SCALE
            except ValueError:
                # joint not in incoming msg; leave old value
                pass
        self.art.set_joint_positions(self.q)

def run():
    sim = SimulationContext()
    art = ArticulationView(prim_paths_expr=ART_ROOT)
    art.initialize()

    rclpy.init()
    node = ArmCmdSub(art)

    # ana döngü
    while sim.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        sim.step(render=True)

if __name__ == "__main__":
    run()
