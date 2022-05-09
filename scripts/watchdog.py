from py_trees.composites import Sequence

from ros_trees.leaves_ros import ServiceLeaf

from rv_leaves.branches.generic.task import ResetTask

from rv_leaves.leaves.generic.noop import Wait
from rv_leaves.leaves.manipulation.status import IsErrored, SetCartesianImpedance
from rv_leaves.leaves.manipulation.grasping import ActuateGripper
from rv_leaves.leaves.manipulation.motion import MoveToNamedGripperPose

class Watchdog(Sequence):
  def __init__(self):
    super(Watchdog, self).__init__('Watchdog', children=[
      IsErrored(timeout=2),
      SetCartesianImpedance(load_value=[3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0]),
      ServiceLeaf('Recover', '/arm/recover', save=False),
      ActuateGripper(),
      MoveToNamedGripperPose(load_value='ready')
    ])