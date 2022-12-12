import rospy
import math

from typing import Tuple, List

from std_msgs.msg import String
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Pose


def compute_distance(x: Pose, y: Pose) -> float:
    return math.sqrt(math.pow(x.x - y.x, 2) + math.pow(x.y - y.y, 2) + math.pow(x.z - y.z, 2))


def get_quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert an Euler angle to a quaternion.

    Args:
        roll: The roll (rotation around x-axis) angle in radians.
        pitch: The pitch (rotation around y-axis) angle in radians.
        yaw: The yaw (rotation around z-axis) angle in radians.

    Return:
        qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    sin_roll = math.sin(roll/2)
    cos_roll = math.cos(roll/2)

    sin_pitch = math.sin(pitch/2)
    cos_pitch = math.cos(pitch/2)

    sin_yaw = math.sin(yaw/2)
    cos_yaw = math.cos(yaw/2)


    qx = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw
    qy = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw
    qz = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw
    qw = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw

    return (qx, qy, qz, qw)


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Convert a quaternion to Euler angle.

    Args:
        x: quaternion x
        y: quaternion y
        z: quaternion z
        w: quaternion w

    Return:
        roll, pitch, yaw in radians.
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return (roll, pitch, yaw)


class ControlMessage:
    """Send control message via MavROS"""

    def __init__(self) -> None:
        self.control_pub = rospy.Publisher('/rc_demo', String, queue_size=10)

    def send_control(self, roll: float, pitch: float, yaw: float, throttle: float, ctrl_mode: bool) -> None:
        """Send control message
        Header: c / w
        Args:
            roll: roll
            pitch: pitch
            yaw: yaw
            throttle: throttle
        """
        if ctrl_mode:
            mode = "c"
        else:
            mode = "w"
        msg = String()
        msg.data = "{mode},{roll},{pitch},{yaw},{throttle}".format(mode=mode,roll=roll,pitch=pitch,yaw=yaw,throttle=throttle)
        self.control_pub.publish(msg)

    def send_delete_path_cmd(self) -> None:
        """Send delete path message
        Header: d
        """
        msg = String()
        msg.data = "d"
        self.control_pub.publish(msg)

    def send_control_assist(self, roll: float, pitch: float, yaw: float, throttle: float) -> None:
        """Send control message
        Header: a
        Args:
            roll: roll
            pitch: pitch
            yaw: yaw
            throttle: throttle
        """
        msg = String()
        msg.data = "a,{roll},{pitch},{yaw},{throttle}".format(roll=roll,pitch=pitch,yaw=yaw,throttle=throttle)
        self.control_pub.publish(msg)

    def send_plan_start(self) -> None:
        """Send delete path message
        Header: p
        """
        msg = String()
        msg.data = "p"
        self.control_pub.publish(msg)


class MAVROSCommander:
    def __init__(self):
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

    def set_mode(self, mode):
        offb_set_mode = SetMode()
        offb_set_mode.custom_mode = mode
        try:
            resp1 = self.set_mode_client(0, offb_set_mode.custom_mode)
            return resp1.mode_sent
        except:
            return False

    def set_arm(self, value):
        arm_cmd = CommandBool()
        arm_cmd.value = value
        try:
            arm_client_1 = self.arming_client(arm_cmd.value)
            return arm_client_1.success
        except:
            return False


class MAVStateReciever:
    def __init__(self):
        self.state_subscriber = rospy.Subscriber('/mavros/state', State, self.callback_state)
        self.armed = False
        self.mode = ''

    def callback_state(self, msg: State):
        self.armed = msg.armed
        self.mode = msg.mode
