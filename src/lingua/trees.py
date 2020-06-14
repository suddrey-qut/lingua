import rospy

from py_trees.composites import Sequence, Composite
from py_trees.common import Status

from lingua_world.srv import FindObjects

class Root(Composite):
  def __init__(self, name='Root', children=None, *args, **kwargs):
    super(Root, self).__init__(name=name, children=children if children else [], *args, **kwargs)
    self.current_child = None

  def tick(self):
      """
      Run the tick behaviour for this root.

      Yields:
          :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
      """
      self.logger.debug("%s.tick()" % self.__class__.__name__)
      
      if self.status != Status.RUNNING:
          self.initialise()

      self.update()

      if not self.children:
        self.status = Status.SUCCESS
        yield self
        return

      previous = self.current_child
      for child in self.children:
          for node in child.tick():
              yield node
              if node is child:
                  if node.status == Status.RUNNING or node.status == Status.SUCCESS:
                      self.current_child = child
                      self.status = node.status
                      if previous is None or previous != self.current_child:
                          # we interrupted, invalidate everything at a lower priority
                          passed = False
                          for child in self.children:
                              if passed:
                                  if child.status != Status.INVALID:
                                      child.stop(Status.INVALID)
                              passed = True if child == self.current_child else passed
                      
                      if child.status == Status.SUCCESS:
                        self.remove_child(child)

                      yield self
                      return
      
      try:
          self.current_child = self.children[-1]
      except IndexError:
          self.current_child = None
      yield self

  def stop(self, new_status=Status.INVALID):
      """
      Stopping a root behaviour requires setting the current child to none. Note that it
      is important to implement this here instead of terminate, so users are free
      to subclass this easily with their own terminate and not have to remember
      that they need to call this function manually.

      Args:
          new_status (:class:`~py_trees.common.Status`): the composite is transitioning to this new status
      """
      # retain information about the last running child if the new status is
      # SUCCESS or FAILURE
      if new_status == Status.INVALID:
          self.current_child = None
      Composite.stop(self, new_status)

  def __repr__(self):
      """
      Simple string representation of the object.

      Returns:
          :obj:`str`: string representation
      """
      s = "Name       : %s\n" % self.name
      s += "  Status  : %s\n" % self.status
      s += "  Current : %s\n" % (self.current_child.name if self.current_child is not None else "none")
      s += "  Children: %s\n" % [child.name for child in self.children]
      return s


class Subtree(Sequence):
  def __init__(self, name, method_name, arguments, mapping=None, *args, **kwargs):
    super(Subtree, self).__init__(name, children=[], *args, **kwargs)
    self.method_name = method_name
    self.mapping = mapping if mapping else {}
    self.arguments = arguments

    self.search = rospy.ServiceProxy('/lingua/world/query', FindObjects)

  def setup(self, timeout):
    super(Subtree, self).setup(timeout)
    self.method = Method.methods[self.method_name]
    return True
  
  def initialise(self):
    self.remove_all_children()
    
    args = {}

    for type_name, key in self.method.get_arguments():
      if key not in self.mapping:
        continue
      args[key] = self.arguments[self.mapping[key]]

      if isinstance(self.arguments[self.mapping[key]], Groundable) and not args[key].is_grounded():
        args[key].ground(self.search)
    
    self.add_child(self.method.instantiate(args))
    super(Subtree, self).initialise()
    
  def terminate(self, new_status=Status.INVALID):
    super(Subtree, self).terminate(new_status)
    self.remove_all_children()  

from .method import Method
from .types import Groundable