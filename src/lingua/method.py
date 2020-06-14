import itertools
import re
import copy
import importlib
from py_trees.composites import Sequence, Selector, Parallel, Composite

class Method:
  methods = {}

  def __init__(self, name, preconditions=None, postconditions=None, root=None):
    self.name = name

    self.root = root if root else {}

    self.preconditions = preconditions if preconditions else []
    self.postconditions = postconditions if postconditions else []

  def instantiate(self, arguments=None):
    is_iterable = arguments is not None and \
      bool([key for key in arguments if (isinstance(arguments[key], Conjunction) or
        (isinstance(arguments[key], Groundable) and len(arguments[key].get_id()) > 1))
      ])
    
    if not is_iterable:
      subtree = self.generate_tree(arguments if arguments is not None else {}, setup=True)
      return subtree
      
    children = []

    for args in self.zip_arguments(arguments if arguments is not None else {}):
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

  def generate_tree(self, arguments=None, setup=False):
    if self.postconditions:
      root = Selector(self.get_name(), children=[
        Sequence('Postconditions', children=[
          self.generate_branch(postcondition, arguments if arguments is not None else {}) for postcondition in self.postconditions
        ]),
        self.generate_branch(self.root, arguments if arguments is not None else {})
      ])
    else:
      root = self.generate_branch(self.root, arguments if arguments is not None else {})
  
    root.setup(0)
    return root

  def generate_branch(self, branch, arguments):
    if branch['type'] == 'sequence':
      children = list([self.generate_branch(child, arguments) for child in branch['children']])
      return Sequence(branch['name'] if 'name' in branch else None if 'name' in branch else 'sequence', children=children)

    if branch['type'] == 'selector':
      children = list([self.generate_branch(child, arguments) for child in branch['children']])
      return Selector(branch['name'] if 'name' in branch else None if 'name' in branch else 'selector', children=children)

    if branch['type'] == 'class':
      data = copy.deepcopy(branch)

      if 'args' in data and 'load_value' in data['args']:
        if 'method:' in data['args']['load_value']:
          data['args']['load_value'] = arguments[data['args']['load_value'].split(':')[1]]

      module = importlib.import_module(data['package'])
      class_type = module.__getattribute__(data['class_name'])
      return class_type(name=data['name'] if 'name' in data else None, **data['args'] if 'args' in data else {})

    if branch['type'] == 'decorator':
      module = importlib.import_module(branch['package'])
      class_type = module.__getattribute__(branch['class_name'])
      
      try:
        node_name = branch['name'] if 'name' in branch else branch['args']['predicate']
      except KeyError:
        node_name = branch['class_name']

      return class_type(self.generate_branch(branch['child'], arguments), name=node_name)

    if branch['type'] == 'behaviour':
      return Subtree(name=branch['name'] if 'name' in branch else branch['method_name'], method_name=branch['method_name'], arguments=arguments, **branch['args'] if 'args' in branch else {})

from .trees import Subtree
from .types import Conjunction, Groundable, Attribute
