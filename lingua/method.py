import os
import rospkg
import itertools
import re
import copy
import json
import importlib
from py_trees.composites import Sequence, Selector, Parallel, Composite

class Method:
  methods = {}
  _path = None

  def __init__(self, name, root, preconditions=None, postconditions=None, ground=True):
    self.name = name

    self.root = root if root else {}

    self._preconditions = preconditions if preconditions else []
    self._postconditions = postconditions if postconditions else []

    self.ground = ground

  def instantiate(self, arguments=None):
    return InstantiatedMethod(self, arguments)
  
  def get_name(self):
    return self.name

  def get_arguments(self):
    arguments = re.findall('\(([^\)]+)', self.name)
    if arguments:
      return [tuple(arg.split()) for arg in arguments[0].split(', ')]
    return []

  def get_cost(self):
    if self.is_operator():
      return 1
    return sum((methods[subtask.get_name()].get_cost() for subtask in self.root))

  def toJSON(self):
    result = {
      'name': self.name, 
      'root': self.root
    }

    if self.preconditions is not None:
      result['preconditions'] = self._preconditions
    
    if self.postconditions is not None:
      result['postconditions'] = self._postconditions

    return result
    
  def __str__(self):
    output = str(self.name)
    output += '\n\tPreconditions: '
    for precondition in self._preconditions:
      output += '\n\t\t' + str(precondition)
    output += '\n\tPostconditions: '
    for effect in self._postconditions:
      output += '\n\t\t' + str(effect)
    if self.root:
      output += '\n\tTree:'
      output += '\n\t\t' + str(self.root)

    return output

  @property
  def preconditions(self):
    return map(lambda p: p['args']['predicate'], self._preconditions)
  
  @property
  def postconditions(self):
    return map(lambda p: p['args']['predicate'], self._postconditions)

  @staticmethod
  def add(method, save=True):
    Method.methods[method.name] = method
    if save:
      Method.save()

  @staticmethod
  def save():
    to_save = {}

    for key in Method.methods:
      name = key.split('(')[0]

      if name not in to_save:
        to_save[name] = []
      
      to_save[name].append(Method.methods[key].toJSON())
      
    for name in to_save:
      with open('{}/{}.json'.format(Method.get_path(), name), 'w') as f:
        f.write(json.dumps(to_save[name], indent=4, sort_keys=True))

  @staticmethod
  def load(clear=False):
    if clear:
      Method.methods = {}
      
    to_load = filter(lambda f: f.endswith('.json'), os.listdir(Method.get_path()))

    for filename in to_load:
      with open('{}/{}'.format(Method.get_path(), filename)) as f:
        methods = json.loads(f.read())

        for method in methods:
          Method.methods[method['name']] = Method(**method)

  @staticmethod
  def clear():
    Method.methods = {}

  @staticmethod
  def get_path():
    if Method._path:
      return str(Method._path)

    rospack = rospkg.RosPack()
    return str('{}/data/methods'.format(rospack.get_path('lingua')))

  @staticmethod
  def set_path(path=None):
    Method._path = path

  @staticmethod
  def from_tree(root):
    typename = type(root).__name__

    if typename == 'Subtree':
      return {
        'type': 'behaviour',
        'method_name': root.method_name
      }
      
    return ''

class InstantiatedMethod(Method):
  def __init__(self, method, arguments=None):
    self.__dict__ = copy.deepcopy(method.__dict__)
    self.arguments = arguments if arguments is not None else {}

  def to_tree(self):
    is_iterable = self.ground and bool([key for key in self.arguments if (isinstance(self.arguments[key], Conjunction) or
      (isinstance(self.arguments[key], Groundable) and self.arguments[key].count() > 1))
    ])
    
    if not is_iterable:
      subtree = self.generate_tree(self.arguments, setup=True)
      return subtree
      
    children = []
    
    for args in self.zip_arguments(self.arguments):
      branch = self.generate_tree(args, setup=True)
      children.append(branch)
    
    return Sequence(name='sequence:{}'.format(self.name), children=children)
    
  def is_applicable(self, state):
    try:
      for precondition in self.preconditions:
        if not state.is_satisfied(precondition):
          return False
      return True
    except Exception as e:
      return False

  def is_complete(self, state):
    try:
      for postcondition in self.postconditions:
        if not state.is_satisfied(postcondition):
          return False
      return True
    except Exception as e:
      return False

  def result(self, state):
    snapshot = state.copy()
    for effect in self.postconditions:
      snapshot.update(effect)
    return snapshot

  @property
  def preconditions(self):
    keys = sorted(self.arguments.keys(), key=lambda x: 1 - len(x))
    result = []
    for precondition in map(lambda p: p['args']['predicate'], self._preconditions):
      for key in keys:
          replacement = self.arguments[key].get_id()[0]
          result.append(precondition.replace('${' + key + '}', replacement))
    return result
  
  @property
  def postconditions(self):
    keys = sorted(self.arguments.keys(), key=lambda x: 1 - len(x))
    result = []
    for postcondition in map(lambda p: p['args']['predicate'], self._postconditions):
      for key in keys:
          replacement = self.arguments[key].get_id()[0]
          result.append(postcondition.replace(key, replacement))
    return result

  def get_objects(self, groundable):
    if isinstance(groundable, Object) and groundable.count() > 1:
      result = []
      for idx in groundable.get_id():
        obj = Object(groundable.type_name, groundable.name, groundable.attributes, groundable.relation, groundable.limit)
        obj.set_id(idx)
        result.append(obj)
      return result
      
    elif isinstance(groundable, Conjunction):
      return self.get_objects(groundable.get_left()) + self.get_objects(groundable.get_right())

    return [ groundable ]
        

  def product_dict(self, **kwargs):
    keys = list(kwargs.keys())
    vals = list(kwargs.values())

    
    for val_id, val in enumerate(vals):
      vals[val_id] = self.get_objects(val)

    for instance in itertools.product(*vals):
      print('Instance: {}'.format(instance))
      yield dict(zip(keys, instance))

  def zip_arguments(self, arguments):
    return list(self.product_dict(**arguments))

  def generate_tree(self, arguments, setup=False):
    if (not self.root):
      root = LearnMethod(method=self)
      root.setup(0)
      return root

    root = Base.from_json(self.root, arguments)

    if isinstance(root, Base):
      root = root.to_btree()
    #root = self.generate_branch(self.root, arguments)

    if self._preconditions:
      root = Sequence('Method', children=[
        Preconditions('Preconditions', children=[
          Base.from_json(precondition, arguments).to_btree() for precondition in self._preconditions  
        ]),
        root
      ])

    if self._postconditions:
      root = Selector(self.get_name(), children=[
        Sequence('Postconditions', children=[
          Base.from_json(postcondition, arguments).to_btree() for postcondition in self._postconditions
        ]),
        root
      ])
    
    root.setup(0)
    return root

  def generate_branch(self, branch, arguments):
    if not isinstance(branch, dict):
      return branch
      
    if branch['type'] == 'sequence':
      children = list([self.generate_branch(child, arguments) for child in branch['children']])
      return Sequence(branch['name'] if 'name' in branch else None if 'name' in branch else 'sequence', children=children)

    if branch['type'] == 'selector':
      children = list([self.generate_branch(child, arguments) for child in branch['children']])
      return Selector(branch['name'] if 'name' in branch else None if 'name' in branch else 'selector', children=children)

    if branch['type'] == 'class':
      data = copy.deepcopy(branch)

      if 'args' in data and 'load_value' in data['args']:
        matches = re.findall(r'\${([^}]*)}', data['args']['load_value'])

        for match in matches:
          data['args']['load_value'] = arguments[match]

      module = importlib.import_module(data['package'])
      class_type = module.__getattribute__(data['class_name'])
              
      return class_type(**data['args'] if 'args' in data else {})

    if branch['type'] == 'decorator':
      data = copy.deepcopy(branch)

      module = importlib.import_module(data['package'])
      class_type = module.__getattribute__(data['class_name'])
      
      if 'args' in data and 'predicate' in data['args']:
        matches = re.findall('\${([^}]*)}', data['args']['predicate'])

        for match in matches:
          data['args']['predicate'] = data['args']['predicate'].replace('${' + match + '}', str(arguments[match].to_query()))
        
      try:
        node_name = data['name'] if 'name' in data else data['args']['predicate']
      except KeyError:
        node_name = data['class_name']

      return class_type(self.generate_branch(data['child'], arguments), name=node_name, **data['args'] if 'args' in data else {})

    if branch['type'] == 'behaviour':
      local_arguments = copy.deepcopy(arguments)

      if 'args' in branch and 'mapping' in branch['args']:
        for idx in branch['args']['mapping']:
          if isinstance(branch['args']['mapping'][idx], dict):
            local_arguments[idx] = Base.from_json(branch['args']['mapping'][idx], local_arguments)
            branch['args']['mapping'][idx] = idx
      
      return Subtree(name=branch['name'] if 'name' in branch else branch['method_name'], method_name=branch['method_name'], arguments=local_arguments, **branch['args'] if 'args' in branch else {})
    
  def toJSON(self):
    return {
      'name': self.name,
      'args': {
        key: self.arguments[key].get_id() for key in self.arguments
      }
    }
    
from .trees import Subtree, Preconditions, LearnMethod
from .types import Base, Conjunction, Groundable, Attribute, Object
