import rospy
import copy
import re
import json
import itertools
import uuid
import py_trees
import importlib
import rv_trees.data_management as dm

from collections import Iterable

from py_trees import Status

from rv_trees.trees import BehaviourTree
from rv_trees.leaves_ros import ActionLeaf, SubscriberLeaf, ServiceLeaf, PublisherLeaf
from py_trees.composites import Sequence, Selector, Parallel, Composite

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


class GetObjectPose(ServiceLeaf):
  def __init__(self, name=None, *args, **kwargs):
    super(GetObjectPose, self).__init__(
      name=name if name else 'Get Object Pose',
      service_name='/lingua/world/objects/get_pose',
      load_fn=self.load_fn,
      *args,
      **kwargs
    )

  def load_fn(self):
    value = self._default_load_fn(False)

    if isinstance(value, Groundable):
      value = value.get_id()

    if isinstance(value, Iterable):
      return value[0] if value else None
      
    return value

class GroundObjects(ServiceLeaf):
  def __init__(self, name=None, *args, **kwargs):
    super(FindObjects, self).__init__(
      name=name if name else 'Get Object Pose',
      service_name='/lingua/world/objects/search',
      load_fn=self.load_fn,
      *args,
      **kwargs
    )

  def load_fn(self):
    value = self._default_load_fn(False)
    
    if isinstance(value, Groundable):
      return json.dumps(value.toJSON())

    return value

class Assert(ServiceLeaf):
  def __init__(self, name=None, *args, **kwargs):
    super(Assert, self).__init__(
      name=name if name else 'Get Object Pose',
      service_name='/lingua/world/objects/assert',
      load_fn=self.load_fn,
      *args,
      **kwargs
    )

  def load_fn(self):
    value = self._default_load_fn(False)
    
    if isinstance(value, Groundable):
      return json.dumps(value.toJSON())

    return value

class Subtree(py_trees.composites.Sequence):
  def __init__(self, name, method_name, arguments, mapping=None, *args, **kwargs):
    super(Subtree, self).__init__(name, children=[], *args, **kwargs)
    self.method_name = method_name
    self.mapping = mapping if mapping else {}
    self.arguments = arguments

    self.search = rospy.ServiceProxy('/lingua/world/objects/search', FindObjects)

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
        args[key].set_id(self.search(json.dumps(self.arguments[self.mapping[key]].toJSON())).ids)
        
    self.add_child(self.method.instantiate(args))
    super(Subtree, self).initialise()
    
  def terminate(self, new_status=Status.INVALID):
    super(Subtree, self).terminate(new_status)
    self.remove_all_children()

class Method:
  methods = {}

  def __init__(self, name, preconditions=None, postconditions=None, root=None):
    self.name = name

    self.root = root if root else {}

    self.preconditions = preconditions if preconditions else []
    self.postconditions = postconditions if postconditions else []

  def instantiate(self, arguments={}):
    is_iterable = bool([key for key in arguments if isinstance(arguments[key], Conjunction)])

    if not is_iterable:
      subtree = self.generate_tree(arguments, setup=True)
      return subtree
      
    children = []

    for args in self.zip_arguments(arguments):
      branch = self.generate_tree(args, setup=True)
      children.append(branch)
    
    return Sequence(name='sequence:{}'.format(self.name), children=children)
  
  def get_name(self):
    return self.name

  def get_arguments(self):
    return [tuple(arg.split()) for arg in re.findall('\(([^\)]+)', self.name)[0].split(', ')]

  def get_cost(self):
    if self.is_operator():
      return 1
    return sum((methods[subtask.get_name()].get_cost() for subtask in self.root))

  def product_dict(self, **kwargs):
    keys = kwargs.keys()
    vals = kwargs.values()
    for instance in itertools.product(*vals):
      yield dict(zip(keys, instance))

  def zip_arguments(self, arguments):
    objects = {}
    # for key in arguments:

    #   objects[key] = []

    #   if not Parser.is_iterable(arguments[key].get_id()):
    #     objects[key].append(argument[key])
    #     continue
      
    #   object_ids = Parser.logical_split(arguments[key].get_id())[1:]
      
    #   for object_id in object_ids:
    #     objects[key].append(DummyObject(arguments[key].get_type_name(), object_id, arguments[key].descriptor))
    
    return list(self.product_dict(**arguments))
    
  def __str__(self):
    output = str(self.name)
    output += '\n\tPreconditions: '
    for precondition in self.preconditions:
      output += '\n\t\t' + str(precondition)
    output += '\n\tEffects: '
    for effect in self.effects:
      output += '\n\t\t' + str(effect)
    if self.root:
      output += '\n\tTree:'
      output += '\n\t\t' + str(self.root)

    return output

  def generate_tree(self, arguments={}, setup=False):
    root = self.generate_branch(self.root, arguments)
    root.setup(0)
    return root

  def generate_branch(self, branch, arguments):
    if branch['type'] == 'sequence':
      children = list([self.generate_branch(child, arguments) for child in branch['children']])
      return Sequence(branch['name']if 'name' in branch else None if 'name' in branch else 'sequence', children=children)

    if branch['type'] == 'selector':
      children = list([self.generate_branch(child, arguments) for child in branch['children']])
      return Selector(branch['name']if 'name' in branch else None if 'name' in branch else 'selector', children=children)

    if branch['type'] == 'precondition':
      return KBConditionLeaf(name=branch['name']if 'name' in branch else None, arguments=arguments, **branch['args'] if 'args' in branch else {})
    
    if branch['type'] == 'postcondition':
      return KBConditionLeaf(name=branch['name']if 'name' in branch else None, arguments=arguments, **branch['args'] if 'args' in branch else {})
    
    if branch['type'] == 'class':
      data = copy.deepcopy(branch)

      if 'args' in data and 'load_value' in data['args']:
        if 'method:' in data['args']['load_value']:
          data['args']['load_value'] = arguments[data['args']['load_value'].split(':')[1]]

      module = importlib.import_module(data['package'])
      class_type = module.__getattribute__(data['class_name'])
      return class_type(name=data['name']if 'name' in data else None, **data['args'] if 'args' in data else {})

    if branch['type'] == 'behaviour':
      print(branch)
      return Subtree(name=branch['name'] if 'name' in branch else branch['method_name'], method_name=branch['method_name'], arguments=arguments, **branch['args'] if 'args' in branch else {})

from .types import Conjunction, Groundable