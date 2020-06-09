import re
import copy
import json

from py_trees.composites import *
from py_trees.decorators import *
from rv_trees.leaves import *

try:
    from collections.abc import Iterable
except ImportError:
    from collections import Iterable

class Base(object):
    def __init__(self):
        super(Base, self).__init__()

    def to_btree(self, name=None):
        raise NotImplementedError()
        
    def is_valid(self):
        return False

    def is_iterable(self):
        return isinstance(self, Iterable)

    def __bool__(self):
        return bool(self.is_valid())

    def __nonzero__(self):      
        return self.__bool__()

class Groundable(Base):
    def __init__(self):
        self.id = None

    def set_id(self, idx):
        self.id = idx

    def get_id(self):
        return self.id

    def to_query(self):
      return self.id
        
    def ground(self, state):
      if self.is_grounded():
        return
      res = state(self.to_query())
      self.set_id(res.ids)

    def is_grounded(self):
        return self.id is not None
        
class Task(Base):
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

    def to_btree(self, name=None):
        return Subtree(
            name=name if name else self.get_name(),
            method_name=self.get_name(),
            arguments=self.method_arguments,
            mapping={key: key for key in self.get_argument_keys()}
        )

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

    def toJSON(self):
        return {
            'type': 'task',
            'task_name': self.task_name,
            'method_name': self.method_name,
            'argument_types': self.method_argument_types,
            'arguments': { key: self.method_arguments[key].toJSON() for key in self.method_arguments }
        }
    #def __deepcopy__(self, memodict={}):
    #    return Task(self.task_name, {arg_id: copy.deepcopy(self.method_arguments[arg_id]) for arg_id in self.method_arguments})

class Conjunction(Base):
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
        return self.id

    def set_id(self, id):
        self.id = id

    def ground(self, state):
        if not self.left.is_grounded():
            self.left.ground(state)

        if not self.right.is_grounded():
            self.right.ground(state)

        if self.tag == 'and':
          self.set_id(tuple(list(self.left.get_id()) + list(self.right.get_id())))
        
        if self.tag == 'or':
          self.set_id(self.left.get_id()) #TODO fix this...
        # self.set_id(parse(state, '({0} {1} {2})'.format(self.tag,
        #                                                 self.left.get_id(),
        #                                                 self.right.get_id())))

    def is_grounded(self):
        return self.id is not None

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

    def to_btree(self, name=None):
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

    def toJSON(self):
        return {
          'type': 'conjunction', 
          'items': [item.toJSON() for item in self]
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

    def to_btree(self, name=None):
        return RepeatForDuration(name='Repeat for {}'.format(self.duration), child=self.body.to_btree(), duration=self.duration.to_seconds())

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

class Conditional(Base):
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

    def to_btree(self, name=None):
        return Selector(name=name if name else 'if', children=[
            Inverter(self.condition.to_btree('condition')), self.body.to_btree('body')
        ])

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
    
    def toJSON(self):
        return {
            'type': 'condition',
            'condition': self.condition.toJSON(),
            'body': self.body.toJSON()
        }


class WhileLoop(Conditional):
    def to_btree(self, name=None):
        return Selector(name if name else 'while', children=[
            Inverter(self.condition.to_btree()), SuccessIsRunning(self.body.to_btree())
        ])

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

    def to_btree(self, name=None):
        return Assert(
          name if name else str(self),
          load_value=(self.child, self.attribute),
        )    

    def is_valid(self):
        return self.child

    def to_query(self):
        print('(intersec {} {})'.format(self.child.to_query(), self.attribute.to_query()))
        return self.child.to_query()

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

    def to_btree(self):
      return GroundObjects(load_value=self.toJSON())

    def to_query(self):
        return self.toJSON()

    def to_query(self):
      if self.is_grounded():
        return self.id

      atoms = ['(class_label {} ?)'.format(self.name)]

      if self.attributes is not None:
            for attr in self.attributes:
              atoms.append(attr.to_query())
      
      if self.relation is not None:
          atoms.append(self.relation.to_query())
          
      if len(atoms) == 1:
        return atoms[0]

      return '(intersect {})'.format(' '.join(atoms))

    def toJSON(self):
        result = { 'type': 'object', 'attributes': [] }

        if self.name != '*':
            result['attributes'].append({
              'type': 'attribute',
              'attr_type': 'class_label',
              'value': self.name
            })

        if self.attributes:
            for attr in self.attributes:
              result['attributes'].append(attr.toJSON())
        
        if self.relation:
            result['relation'] = str(relation.toJSON())

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
        
        return outstr

class Anaphora(Object):
    def __init__(self, type_name, name, attributes=None, relation=None, limit=None):
        super(Anaphora, self).__init__(type_name, name, attributes, relation, limit)

    def __str__(self):
        return Object.__str__(self).replace('{}:{}'.format(self.type_name, self.name), 'anaphora:it')

class Modifier(Base):
    def __init__(self, type_name, value):
        self.type_name = type_name
        self.value = value

    def toJSON(self):
      return {
        'type': 'modifier',
        'type_name': self.type_name,
        'value': self.value
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

    def toJSON(self):
        return {
            'type': 'attribute',
            'attr_type': self.type_name,
            'value': self.value
        }
    
    def __str__(self):
        return '[{}={}]'.format(self.type_name, self.value)

class Limit(Base):
    pass

class Relation(Base):
    def __init__(self, predicate, child):
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

from .decorators import RepeatForDuration
from .leaves import Subtree, Assert, GroundObjects
