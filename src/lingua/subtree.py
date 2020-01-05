import copy
import re
import itertools
import uuid
import py_trees
import rv_trees.data_management as dm
from rv_trees.trees import BehaviourTree
from rv_trees.leaves_ros import ActionLeaf, SubscriberLeaf, ServiceLeaf, PublisherLeaf
from py_trees.composites import Sequence, Selector, Parallel, Composite

from .types import Groundable

class Condition(ServiceLeaf):
  def __init__(self, name, condition, arguments, save=False, *args, **kwargs):
    super(Condition, self).__init__(name, service_name='/kb/assert', save=save, *args, **kwargs)
    self.condition = condition

  def _extra_update()


class Subtree(py_trees.composites.Sequence):
  def __init__(self, name, method_name, arguments, mapping, *args, **kwargs):
    super(Subtree, self).__init__(name, children=[], *args, **kwargs)
    self.method_name = method_name
    self.mapping = mapping
    self.arguments = arguments

  def setup(self, timeout):
    super(Subtree, self).setup(timeout)
    self.method = Method.methods[self.method_name]
    return True
  
  def initialise(self):
    args = {}
    for t, key in self.method.get_arguments():
      args[key] = self.arguments[self.mapping[key]]
    self.add_child(self.method.instantiate(self.arguments))
    super(Subtree, self).initialise()
    
  def stop(self, new_status=py_trees.Status.INVALID):
    super(Subtree, self).stop(new_status)
    self.remove_all_children()

class Method:
  methods = {}

  def __init__(self, name, root):
    self.name = name
    self.root = root

    self.preconditions = []
    self.effects = []

  def instantiate(self, arguments={}):
    branch = self.generate_tree(arguments)
    branch.setup(0)
    return branch
    #return InstantiatedMethod(self, state, {'arg' + str(idx): arg for idx, arg in enumerate(arguments)})

  def get_name(self):
    return self.name

  def get_arguments(self):
    return [tuple(arg.split()) for arg in re.findall('\(([^\)]+)', self.name)[0].split(', ')]

  def get_cost(self):
    if self.is_operator():
      return 1
    return sum((methods[subtask.get_name()].get_cost() for subtask in self.root))

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

  def generate_tree(self, arguments={}):
    return self.generate_branch(self.root, arguments)

  def generate_branch(self, branch, arguments):
    def load_fn(leaf):
      value = leaf.load_value
      if value in arguments:
        if isinstance(arguments[value], Groundable):
          if not arguments[value].is_grounded():
            arguments[value].ground()
          value = arguments[value].get_id()
        else:
          value = arguments[value]
      leaf.load_value = value
      return leaf._default_load_fn()


    if branch['type'] == 'sequence':
      children = list([self.generate_branch(child, arguments) for child in branch['children']])
      return Sequence(branch['name'] if 'name' in branch else 'sequence', children=children)

    if branch['type'] == 'selector':
      children = list([generate_branch(child, arguments) for child in branch['children']])
      return Selector(branch['name'] if 'name' in branch else 'selector', children=children)

    if branch['type'] == 'condition':
      return ConditionLeaf(name=branch['name'])
    
    if branch['type'] == 'action':
      return ActionLeaf(name=branch['name'], load_fn=load_fn, **branch['args'])

    if branch['type'] == 'service':
      return ServiceLeaf(name=branch['name'], load_fn=load_fn, **branch['args'])
    
    if branch['type'] == 'subscriber':
      return SubscriberLeaf(name=branch['name'], load_fn=load_fn, **branch['args'])

    if branch['type'] == 'publisher':
      return PublisherLeaf(name=branch['name'], load_fn=load_fn, **branch['args'])

    if branch['type'] == 'behaviour':
      return Subtree(name=branch['name'], arguments=arguments, **branch['args'])
    