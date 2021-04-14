import sys
import re
import copy
import json
import random
import importlib
import math

from py_trees.composites import *
from py_trees.decorators import *
from rv_trees.leaves import *

from .errors import *

from lingua_pddl.parser import Parser

try:
  from collections.abc import Iterable
except ImportError:
  from collections import Iterable

class Terminal(object):
  pass

class Base(object):
  def __init__(self):
    super(Base, self).__init__()

  def to_btree(self, name=None, training=False):
    raise NotImplementedError()

  def to_json(self, args=None):
    raise NotImplementedError()

  def ground(self, state):
    raise NotImplementedError()

  def is_valid(self):
    return False

  def is_iterable(self):
    return isinstance(self, Iterable)

  def __bool__(self):
    return bool(self.is_valid())

  def __nonzero__(self):    
    return self.__bool__()

  @staticmethod
  def from_json(data, arguments):
    if 'package' in data:
      module = importlib.import_module(data['package'])
      class_type = module.__getattribute__(data['class_name'])

    else:
      class_type = globals()['__builtins__'][data['class_name']]
    
    args = {}
    
    if 'args' in data:
      for idx in data['args']:
        if isinstance(data['args'][idx], dict):
          args[idx] = Base.from_json(data['args'][idx], arguments)
        elif isinstance(data['args'][idx], list):
          args[idx] = [Base.from_json(arg, arguments) for arg in data['args'][idx]]
        else:
          args[idx] = data['args'][idx]
          try:
            matches = re.findall(r'\${([^}]*)}', args[idx])

            for match in matches:
              args[idx] = arguments[match]
          except:
            pass

    return class_type(**args)

class LinguaSequence(Base):
  def __init__(self, children=[]):
    super(LinguaSequence, self).__init__()
    self.children = children

    def to_json(self, args):
      return {
        'type': 'class',
        'class_name': 'LinguaSequence',
        'package': 'lingua.types',
        'args': { 
          'children': [ node.to_json(args) for node in self.children ]
        }
      }

  def to_btree(self, name=None, training=False):
    return Sequence(children=[child.to_btree() if isinstance(child, Base) else child for child in self.children])

class Groundable(Base):
  def __init__(self):
    self.id = None

  def set_id(self, idx):
    self.id = [ idx ] if isinstance(idx, str) else idx

  def unset(self):
    self.id = None

  def count(self):
    return len(self.id)
    
  def get_id(self):
    return self.id

  def to_query(self):
    return ' '.join(self.id)
    
  def ground(self, state):
    if self.is_grounded():
      return
    try:
      query = self.to_query()
      res = state.ask(query)
    except Exception as e:
      self.set_id([])
      return
    if Parser.is_iterable(res):
      res = Parser.logical_split(res)[1:]
    self.set_id(res)

  def is_grounded(self):
    return self.id is not None and len(self.id)
    
class Task(Groundable):
  def __init__(self, name, arguments):
    self.method_arguments = arguments

    self.method_argument_types = {}
    
    if arguments:
      for arg in re.findall(r'\(([^\)]+)', name)[0].split(', '):
        type_name, var = arg.split()
        self.method_argument_types[var] = type_name

    self.method_name = name.split('(')[0]
    self.task_name = name

  def get_name(self):
    return self.method_name + '(' + ', '.join([self.method_argument_types[key] + ' ' + key
                           for key in self.method_argument_types]) + ')'

  def get_argument_keys(self):
    for arg in self.method_arguments:
      yield arg

  def get_arguments(self):
    return list(self.method_arguments.values())

  def get_argument(self, key):
    return self.method_arguments[key]

  def set_argument(self, key, value):
    self.method_arguments[key] = value

  def get_argument_type(self, key):
    return self.method_argument_types[key]

  def set_argument_type(self, key, value):
    self.method_argument_types[key] = value

  def to_btree(self, name=None, training=False):
    return Subtree(
      name=name if name else self.get_name(),
      method_name=self.get_name(),
      arguments=self.method_arguments,
      mapping={key: key for key in self.get_argument_keys()}
    )

  def ground(self, state):
    for key in self.method_arguments:
      self.method_arguments[key].ground(state)

  def is_grounded(self):
    for key in self.method_arguments:
      if not self.method_arguments[key].is_grounded():
        return False
    return True

  def is_valid(self):
    for key in self.method_arguments:
      if not self.method_arguments[key]:
        return False
    return True

  def __iter__(self):
    for key in self.method_arguments:
      yield self.method_arguments[key]
    
  def __str__(self):
    outstr = self.get_name()
    for key in self.method_arguments:
      outstr += '\n {}:'.format(key)
      for line in str(self.method_arguments[key]).split('\n'):
        outstr += '\n  {}'.format(line)
     
    return outstr

  def to_json(self, args):
    return {
      'type': 'class',
      'class_name': 'Task',
      'package': 'lingua.types',
      'method_name': self.task_name,
      'args': { 
        'name': self.task_name,
        'arguments': {
          'class_name': 'dict',
          'args': {
            key: self.method_arguments[key].to_json(args) for key in self.method_arguments 
          }
        }
      }
    }

class Conjunction(Groundable):
  def __init__(self, tag, left, right):
    self.tag = tag
    self.left = left
    self.right = right

  def get_type_name(self):
    if isinstance(self.left, Object):
      return self.left.get_type_name()
    return None

  def get_descriptor(self):
    return str(self)

  def get_id(self):
    if self.tag == 'and':
      return self.get_left().get_id() + self.get_right().get_id()
    else:
      return random.choice([self.get_left().get_id(), self.get_right().get_id()])

  def ground(self, state):
    if not self.left.is_grounded():
      self.left.ground(state)

    if not self.right.is_grounded():
      self.right.ground(state)

    # if self.tag == 'and':
    #   self.set_id(tuple(list(self.left.get_id()) + list(self.right.get_id())))
    
    # if self.tag == 'or':
    #   self.set_id(self.left.get_id()) #TODO fix this...
    # self.set_id(parse(state, '({0} {1} {2})'.format(self.tag,
    #                         self.left.get_id(),
    #                         self.right.get_id())))

  def is_grounded(self):
    return self.left.is_grounded() and self.right.is_grounded()

  def get_left(self):
    return self.left

  def set_left(self, left):
    self.left = left

  def get_right(self):
    return self.right

  def set_right(self, right):
    self.right = right

  def get_type(self):
    return self.tag if self.tag != ',' else self.right.get_type()

  def to_btree(self, name=None, training=False):
    if self.get_type() == 'and':
      return Sequence(name if name else 'and', children=[self.left.to_btree(), self.right.to_btree()])

    return Selector(name if name else 'or', children=[self.left.to_btree(), self.right.to_btree()])

  def __iter__(self):
    if not isinstance(self.left, Conjunction):
      yield self.left
    else:
      for item in self.left:
        yield item
    if not isinstance(self.right, Conjunction):
      yield self.right
    else:
      for item in self.right:
        yield item

  def is_valid(self):
    return self.left and self.right

  def __str__(self):
    outstr = self.tag
    
    outstr += '\n -'
    for line in str(self.left).split('\n'):
      outstr += '\n  {}'.format(line)
      
    outstr += '\n -'
    for line in str(self.right).split('\n'):
      outstr += '\n  {}'.format(line)
      
    return outstr

  def to_json(self, args):
    return {
      'type': 'class',
      'class_name': self.__class__.__name__,
      'package': 'lingua.types',
      'args': {
        'tag': self.tag,
        'left': self.left.to_json(args),
        'right': self.right.to_json(args)
      }
    }

class ForLoop(Base):
  def __init__(self, duration, body):
    super(ForLoop, self).__init__()
    
    self.duration = duration
    self.body = body

  def get_duration(self):
    return self.duration

  def set_duration(self, duration):
    self.duration = duration

  def get_body(self):
    return self.body

  def set_body(self, body):
    self.body = body

  def to_btree(self, name=None, training=False):
    return RepeatForDuration(name='Repeat for {}'.format(self.duration), child=self.body.to_btree(), duration=self.duration.to_seconds())

  def ground(self, state):
    self.body.ground(state)

  def is_grounded(self):
    return self.body.is_grounded()

  def is_valid(self):
    return self.duration and self.body

  def __iter__(self):
    yield self.body

  def __str__(self):
    outstr = 'for:\n duration:'
    
    for line in str(self.duration).split('\n'):
      outstr += '\n  {}'.format(line)
    outstr += '\n body:'
    for line in str(self.body).split('\n'):
      outstr += '\n  {}'.format(line)

    return outstr

  def to_json(self, args):
    return {
      'type': 'class',
      'package': 'lingua.types',
      'class_name': 'ForLoop',
      'args': {
        'duration': self.duration.to_json(args),
        'body': self.body.to_json(args)
      }
    }

class InfiniteLoop(Base):
  def __init__(self, body):
    super(InfiniteLoop, self).__init__()
    self.body = body

  def get_body(self):
    return self.body

  def set_body(self, body):
    self.body = body

  def to_btree(self, name=None, training=False):
    if training:
      return self.body.to_btree()
    return SuccessIsRunning(name='Repeat', child=self.body.to_btree())

  def ground(self, state):
    self.body.ground(state)

  def is_valid(self):
    return self.body

  def __iter__(self):
    yield self.body

  def __str__(self):
    outstr = 'repeat:'
    outstr += '\n body:'
    for line in str(self.body).split('\n'):
      outstr += '\n  {}'.format(line)

    return outstr

  def to_json(self, args):
    return {
      'type': 'class',
      'package': 'lingua.types',
      'class_name': 'InfiniteLoop',
      'args': {
        'class_name': 'SuccessIsRunning',
        'body': self.body.to_json(args)
      }
    }

class Conditional(Groundable):
  def __init__(self, condition, body, inverted=False):
    super(Conditional, self).__init__()

    self.condition = condition
    self.body = body

    self.inverted = inverted

  def is_inverted(self):
    '''
    The conditional is inverted if the condition uttered after the body
    For example: pick the ball up if it is red
    '''
    return self.inverted

  def get_condition(self):
    return self.condition

  def set_condition(self, condition):
    self.condition = condition

  def get_body(self):
    return self.body

  def set_body(self, body):
    self.body = body

  def to_btree(self, name=None, training=False):
    return Selector(name=name if name else 'if', children=[
      Inverter(self.condition.to_btree('condition')), self.body.to_btree('body')
    ])

  def ground(self, state):
    try:
      self.condition.ground(state)
    except Exception as e:
      pass
    self.body.ground(state)

  def is_valid(self):
    return self.condition and self.body

  def __iter__(self):
    yield self.get_body()

  def __str__(self):
    outstr = 'if:\n condition:'
    
    for line in str(self.condition).split('\n'):
      outstr += '\n  {}'.format(line)
    outstr += '\n body:'
    for line in str(self.body).split('\n'):
      outstr += '\n  {}'.format(line)

    return outstr
  
  def to_json(self, args):
    return {
      'type': 'class',
      'package': 'lingua.types',
      'class_name': self.__class__.__name__,
      'args': {
        'condition': self.condition.to_json(args),
        'body': self.body.to_json(args)
      }
    }

class Event(Conditional):
  def to_btree(self, name=None, training=False):
    return SuccessIsFailure(Sequence(name if name else 'when', children=[
      self.condition.to_btree(), self.body.to_btree()
    ]))

  def __str__(self):
    outstr = 'when:\n condition:'
    
    for line in str(self.condition).split('\n'):
      outstr += '\n  {}'.format(line)
    outstr += '\n body:'
    for line in str(self.body).split('\n'):
      outstr += '\n  {}'.format(line)

    return outstr


class WhileLoop(Conditional):
  def to_btree(self, name=None, training=False):
    return FailureIsSuccess(SuccessIsRunning(Sequence(name if name else 'while', children=[
      self.condition.to_btree(), FailureIsSuccess(self.body.to_btree(), name='body')
    ])))

  def __str__(self):
    outstr = 'while:\n condition:'
    
    for line in str(self.condition).split('\n'):
      outstr += '\n  {}'.format(line)
    outstr += '\n body:'
    for line in str(self.body).split('\n'):
      outstr += '\n  {}'.format(line)

    return outstr


class Assertion(Groundable):
  def __init__(self, child, attribute):
    super(Assertion, self).__init__()
    self.child = child
    self.attribute = attribute

  def get_body(self):
    return self.child

  def set_body(self, child):
    self.child = child

  def get_attribute(self):
    return self.attribute

  def set_attribute(self, attribute):
    self.attribute = attribute

  def to_btree(self, name=None, training=False):
    return Assert(
      name if name else str(self),
      load_value=(self.child, self.attribute),
    )  

  def ground(self, state):
    self.child.ground(state)
    self.attribute.ground(state)

  def is_valid(self):
    return self.child

  def to_query(self):
    print('(intersec {} {})'.format(self.child.to_query(), self.attribute.to_query()))
    return self.child.to_query()

  def to_json(self, args):
    return { 
      'type': 'class', 
      'package': 'lingua.types', 
      'class_name': self.__class__.__name__,
      'args': {
        'child': self.child.to_json(args),
        'attribute': self.attribute.to_json({})
      }
    }

  def __str__(self):
    outstr = 'assert:'

    outstr += '\n object:'
    for line in str(self.child).split('\n'):
      outstr += '\n  {}'.format(line)
      
    outstr += '\n is:'
    for line in str(self.attribute).split('\n'):
      outstr += '\n  {}'.format(line)
    
    return outstr

class Object(Groundable):
  def __init__(self, type_name, name, attributes=None, relation=None, limit=None):
    super(Object, self).__init__()

    self.type_name = type_name
    self.name = name

    self.attributes = attributes if attributes else []
    
    self.relation = relation
    self.limit = limit
    
  def get_type_name(self):
    return self.type_name

  def set_type_name(self, value):
    self.type_name = value

  def set_id(self, idx):
    super(Object, self).set_id(idx)
    
    if self.limit:
      super(Object, self).set_id(self.limit(self.get_id()))

  def count(self):
    return min(self.limit.count if self.limit else sys.maxint, super(Object, self).count())

  def get_id(self):
    idx = super(Object, self).get_id()
    if self.limit:
      return self.limit(idx)
    else:
      return idx

  def to_btree(self):
    return GroundObjects(load_value=self)

  def to_query(self):
    if self.is_grounded():
      return super(Object, self).to_query()

    atoms = ['(class_label {} ?)'.format(self.name)]

    if self.attributes is not None:
      for attr in self.attributes:
        atoms.append(attr.to_query())
    
    if self.relation is not None:
      atoms.append(self.relation.to_query())
      
    if len(atoms) == 1:
      return atoms[0]

    return '(intersect {})'.format(' '.join(atoms))

  def to_json(self, args):
    for key in args:
      if args[key].get_id() == self.get_id():
        return '${{{}}}'.format(key)
    
    result = { 
      'type': 'class', 
      'package': 'lingua.types', 
      'class_name': 'Object', 
      'args': {
        'type_name': self.type_name,
        'name': self.name
      }
    }

    if self.attributes:
      result['args']['attributes'] = []
      for attr in self.attributes:
        result['args']['attributes'].append(attr.to_json(args))
    
    if self.relation:
      result['args']['relation'] = self.relation.to_json(args)

    if self.limit:
      result['args']['limit'] = self.limit.to_json(args)

    return result

  def is_valid(self):
    return self.type_name and self.name

  def __iter__(self):
    yield self.relation

  def __str__(self):
    outstr = '{}:{}'.format(self.type_name, self.name)
    
    if self.attributes:
      outstr += '\n attributes:'
    
      for attr in self.attributes:
        for line in str(attr).split('\n'):
          outstr += '\n  {}'.format(line)

    if self.relation:
      outstr += '\n relation:'
      
      for line in str(self.relation).split('\n'):
        outstr += '\n  {}'.format(line)

    if self.limit:
      outstr += '\n limit:'
      
      for line in str(self.limit).split('\n'):
        outstr += '\n  {}'.format(line)
    
    return outstr

class Anaphora(Object):
  def __init__(self, type_name, name, attributes=None, relation=None, limit=None):
    if relation is not None:
      raise Exception('Cannot attach relation to anaphora')
    super(Anaphora, self).__init__(type_name, name, attributes, relation, limit)

  def __str__(self):
    return Object.__str__(self).replace('{}:{}'.format(self.type_name, self.name), 'anaphora:it')

class DummyObject(Object):
  def __init__(self, type_name='', name='', attributes=None, relation=None, limit=None, idx=None):
    super(DummyObject, self).__init__(type_name, name, attributes, relation, limit)
    
    if idx is not None:
      self.set_id(idx)

  def to_query(self):
    if self.is_grounded():
      return super(DummyObject, self).to_query()

    atoms = []

    if self.attributes is not None:
      for attr in self.attributes:
        atoms.append(attr.to_query())
    
    if self.relation is not None:
      atoms.append(self.relation.to_query())
      
    if len(atoms) == 1:
      return atoms[0]

    return '(intersect {})'.format(' '.join(atoms))

  def to_json(self, args):
    for key in args:
      if args[key].get_id() == self.get_id():
        return '${{{}}}'.format(key)
    
    result = { 
      'type': 'class', 
      'package': 'lingua.types', 
      'class_name': 'DummyObject', 
      'args': {
        'type_name': self.type_name,
        'name': self.name
      }
    }

    if self.attributes:
      result['args']['attributes'] = []
      for attr in self.attributes:
        result['args']['attributes'].append(attr.to_json(args))
    
    if self.relation:
      result['args']['relation'] = self.relation.to_json(args)

    if self.limit:
      result['args']['limit'] = self.limit.to_json(args)

    return result

class Modifier(Base):
  def __init__(self, type_name, value):
    super(Modifier, self).__init__()

    self.type_name = type_name
    self.value = value

  def to_json(self, args):
    return {
      'type': 'class', 
      'package': 'lingua.types',
      'args': {
        'type_name': self.type_name,
        'value': self.value
      }
    }
    
  def __str__(self):
    return 'not:[{}={}]'.format(self.type_name, self.value)

class Attribute(Groundable):
  def __init__(self, type_name, value):
    super(Attribute, self).__init__()

    self.type_name = type_name
    self.value = value

  def ground(self, state):
    res = state(json.dumps(Object(attributes=[self])))
    self.set_id(res.ids)

  def to_query(self):
    return '({} {} ?)'.format(self.type_name, self.value)

  def to_json(self, args):
    return { 
      'type': 'class', 
      'package': 'lingua.types', 
      'class_name': 'Attribute', 
      'args': {
        'type_name': self.type_name,
        'value': self.value
      }
    }
  
  def __str__(self):
    return '[{}={}]'.format(self.type_name, self.value)

class Relation(Base):
  def __init__(self, predicate, child):
    super(Relation, self).__init__()

    self.predicate = predicate
    self.child = child

  def to_query(self):
    return '({} {} ?)'.format(self.predicate, self.child.to_query())

  def __iter__(self):
    yield self.child

  def __str__(self):
    return '[{}={}]'.format(self.predicate, str(self.child))

class Duration(Base):
  def __init__(self, units, time):
    super(Duration, self).__init__()

    self.units = units
    self.time = time

  def to_seconds(self):
    if self.units == 'hour':
      return self.time * 60 * 60
    if self.units == 'minute':
      return self.time * 60
    return self.time

  def is_valid(self):
    return True

  def __str__(self):
    return '{} {}(s)'.format(self.time, self.units)

  def to_json(self, args):
    return {
      'package': 'lingua.types',
      'class_name': self.__class__.__name__,
      'args': {
        'units': self.units,
        'time': self.time
      }
    }

class Limit(Base):
  def __init__(self, count):
    super(Limit, self).__init__()
    self.count = count

  def to_json(self, args):
    return {
      'package': 'lingua.types',
      'class_name': self.__class__.__name__,
      'args': {
        'count': self.count
      }
    }

  def __call__(self, value):
    raise NotImplementedError()

  def __str__(self):
    return 'Limit: {}'.format(self.count)

class Any(Limit):
  def __init__(self, count):
    super(Any, self).__init__(count)

  def is_valid(self):
    return True

  def __call__(self, value):
    return random.sample(value, min(len(value), self.count))

  def __str__(self):
    return 'Any: {}'.format(self.count)

class Only(Limit):
  def __init__(self, count):
    super(Only, self).__init__(count)

  def is_valid(self):
    return True

  def __call__(self, value):
    if value and len(value) > self.count:
      raise AmbigiousStatement('Ambigious Statement')

    return value

  def __str__(self):
    return 'Only: {}'.format(self.count)

class Affirmative(Groundable):
  def to_btree(self, name=None, training=False):
    return Leaf(name='noop', eval_fn=lambda l, v: True)

  def is_valid(self):
    return True

  def ground(self, state):
    pass

  def is_grounded(self):
    return True

class Negative(Groundable):
  def to_btree(self, name=None, training=False):
    return Leaf(name='noop', eval_fn=lambda l, v: True)

  def is_valid(self):
    return True

  def ground(self, state):
    pass

  def is_grounded(self):
    return True

from .decorators import RepeatForDuration
from .leaves import Assert, GroundObjects
from .trees import Subtree
