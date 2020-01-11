import copy
import re
import itertools
import uuid
import py_trees
import rv_trees.data_management as dm
from rv_trees.trees import BehaviourTree
from rv_trees.leaves_ros import ActionLeaf, SubscriberLeaf, ServiceLeaf, PublisherLeaf
from py_trees.composites import Sequence, Selector, Parallel, Composite

from lingua_pddl.parser import Parser
from .types import Groundable, DummyObject

class ConditionLeaf(ServiceLeaf):
  def __init__(self, name, condition, arguments, save=False, *args, **kwargs):
    super(ConditionLeaf, self).__init__(
      name, 
      service_name='/kb/assert', 
      load_value=condition, 
      save=save, 
      load_fn=self.load_fn, 
      eval_fn=self.eval_fn, 
      *args, **kwargs)

    self.condition = condition
    self.arguments = arguments

  def load_fn(self):
    data = super(ConditionLeaf, self)._default_load_fn()
    for arg_key in self.arguments:
      if not self.arguments[arg_key].is_grounded():
        self.arguments[arg_key].ground()
      data.query = re.sub(r'({})([)\s])'.format(arg_key), r'{}\2'.format(self.arguments[arg_key].get_id()), self.condition)
    return data

  def eval_fn(self, result):
    return result.result


class EffectLeaf(ServiceLeaf):
  def __init__(self, name, condition, arguments, save=False, *args, **kwargs):
    super(EffectLeaf, self).__init__(
      name, 
      service_name='/kb/tell', 
      load_value=condition, 
      load_fn=self.load_fn, 
      save=save, 
      *args, **kwargs)

    self.condition = condition
    self.arguments = arguments

  def load_fn(self):
    data = super(EffectLeaf, self)._default_load_fn()
    for arg_key in self.arguments:
      if not self.arguments[arg_key].is_grounded():
        self.arguments[arg_key].ground()
      data.statement = re.sub(r'({})([)\s])'.format(arg_key), r'{}\2'.format(self.arguments[arg_key].get_id()), self.condition)
    return data

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
    for key in arguments:
      arguments[key].ground()

    is_iterable = bool([key for key in arguments if Parser.is_iterable(arguments[key].get_id())])
    
    if not is_iterable:
      return self.generate_tree(arguments, setup=True)
      
    children = []

    for args in self.zip_arguments(arguments):
      branch = self.generate_tree(args, setup=True)
      children.append(branch)

    return Sequence(name='sequence:{}'.format(self.name), children=children)
    
    #return InstantiatedMethod(self, state, {'arg' + str(idx): arg for idx, arg in enumerate(arguments)})

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
    for key in arguments:
      objects[key] = []

      if not Parser.is_iterable(arguments[key].get_id()):
        objects[key].append(argument[key])
        continue
      
      object_ids = Parser.logical_split(arguments[key].get_id())[1:]
      
      for object_id in object_ids:
        objects[key].append(DummyObject(arguments[key].get_type_name(), object_id, arguments[key].descriptor))
    
    return list(self.product_dict(**objects))
    
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
    tree = self.generate_branch(self.root, arguments)
    if setup:
      tree.setup(0)
    return tree

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
      children = list([self.generate_branch(child, arguments) for child in branch['children']])
      return Selector(branch['name'] if 'name' in branch else 'selector', children=children)

    if branch['type'] == 'precondition' or branch['type'] == 'postcondition':
      return ConditionLeaf(name=branch['name'], arguments=arguments, **branch['args'])
    
    if branch['type'] == 'effect':
      return EffectLeaf(name=branch['name'], arguments=arguments, **branch['args'])
        
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
    