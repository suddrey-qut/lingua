import rospy
from sensor_msgs.msg import JointState

def gripper_open(args):
  if args[0] == '?':
    return ['g1'] if gripper_open('g1') else []
  
  msg = rospy.wait_for_message('/franka_gripper/joint_states', JointState)
  return msg.position[0] > 0.035 and msg.position[1] > 0.035