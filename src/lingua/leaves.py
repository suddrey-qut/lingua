import json

from collections import Iterable
from py_trees import Status
from rv_trees.leaves import Leaf
from rv_trees.leaves_ros import ActionLeaf, PublisherLeaf, ServiceLeaf

from std_msgs.msg import String

class Planner(ActionLeaf):
  def __init__(self, name='Planner', conditions=None, target=None, *args, **kwargs):
    super(Planner, self).__init__(
      name=name, 
      action_namespace='/resolver',
      load_fn=self.load_fn,
      result_fn=self.result_fn,
      *args,
      **kwargs
    )
    self.conditions = conditions
    self.target = target

  def load_fn(self):
    goal = self._default_load_fn()
    goal.predicates = self.conditions
    goal.path = Method.get_path()

    return goal

  def result_fn(self):
    result = self._default_result_fn()
    target = self.target if self.target is not None else self.parent

    plan = json.loads(result.solution)
    children = []

    for step in plan:
      arguments = {}
      for arg_id in step['args']:
        arguments[arg_id] = DummyObject(idx=step['args'][arg_id])

      print(arguments)
      subtree = Method.methods[step['name']] \
        .instantiate(arguments) \
        .to_tree()

      subtree.setup(0)
      children.append(subtree)
    
    target.add_children(children)
    return children


class GetObjectPose(ServiceLeaf):
  def __init__(self, name=None, *args, **kwargs):
    super(GetObjectPose, self).__init__(
      name=name if name else 'Get Object Pose',
      service_name='/lingua/world/get_pose',
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

class Say(PublisherLeaf):
  def __init__(self, name='Say', topic_name='/speech/out', topic_class=String, *args, **kwargs):
    super(Say, self).__init__(name=name, topic_name=topic_name, topic_class=topic_class, *args, **kwargs)


class PollInput(Leaf):
  def __init__(self, name='Poll Input', *args, **kwargs):
    super(PollInput, self).__init__(
      name=name,
      save=True,
      result_fn=self.result_fn,
      *args,
      **kwargs
    )
    
    self.input = None
    
  def _extra_initialise(self):
    self.get_root().set_listener(self)
    self.input = None
    
  def _extra_terminate(self, new_status):
    self.get_root().set_listener(None)

  def _is_leaf_done(self):
    return True if self.input is not None else False

  def result_fn(self):
    print('Result:', self.input)
    return self.input
    
  def set_input(self, resp):
    self.input = resp

  def get_root(self):
    parent = self.parent
    while not isinstance(parent, Root):
      parent = parent.parent
    return parent

class GroundObjects(ServiceLeaf):
  def __init__(self, name=None, *args, **kwargs):
    super(GroundObjects, self).__init__(
      name=name if name else 'Ground Objects',
      service_name='/kb/query',
      load_fn=self.load_fn,
      result_fn=self.result_fn,
      *args,
      **kwargs
    )

  def load_fn(self):
    value = self._default_load_fn(False)

    if not isinstance(value, Groundable):
      raise Exception('Expected input value to be groundable')

    return value

  def result_fn(self):
    obj = self.loaded_data
    
    if obj.is_grounded():
      return obj.get_id()

    result = self._default_result_fn(obj.to_query())

    if not result or len(result.ids) == 0:
      return

    try:
      obj.set_id(result.ids)
    except:
      return False
    
    return result.ids

class Assert(GroundObjects):
  def __init__(self, name=None, *args, **kwargs):
    super(Assert, self).__init__(
      name=name if name else 'Assert Object'
      *args,
      **kwargs
    )

  def result_fn(self):
    item, attribute = self.loaded_data

    if not item.is_grounded():
      item.ground(self._service_proxy)

    if not attribute.is_grounded():
      attribute.ground(self._service_proxy)

    intersection = set(item.get_id()).intersection(attribute.get_id())
    
    return len(intersection) > 0
    
  def load_fn(self):
    value = self._default_load_fn(False)
    
    if isinstance(value, Groundable) and not value.is_grounded():
      return value.to_query()

    return value

from .types import Groundable, DummyObject
from .trees import Root
from .method import Method