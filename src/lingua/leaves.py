from collections import Iterable
from py_trees import Status
from rv_trees.leaves_ros import ServiceLeaf

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

class GroundObjects(ServiceLeaf):
  def __init__(self, name=None, *args, **kwargs):
    super(GroundObjects, self).__init__(
      name=name if name else 'Ground Objects',
      service_name='/lingua/world/query',
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

    obj.set_id(result.ids)
    
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

from .types import Groundable