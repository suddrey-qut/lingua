import itertools
import re
import xml.etree.ElementTree as ET
from lingua.types import *

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

class CCGReader:
    @staticmethod
    def read(text):
        tree = ET.fromstring(text)
        return tree.find('target').text, XMLReader.read(tree)

class XMLReader: 
    @staticmethod
    def read(node):
        try:
            if node.tag == 'xml':
                return XMLReader.read(node.find('lf')[0])
            
            if TaskReader.is_task(node):
                return TaskReader.read(node)

            if EventReader.is_event(node):
                return EventReader.read(node)

            if ConditionalReader.is_conditional(node):
                return ConditionalReader.read(node)

            if ConjunctionReader.is_conjunction(node):
                return ConjunctionReader.read(node)

            if WhileLoopReader.is_while(node):
                return WhileLoopReader.read(node)

            if ForLoopReader.is_for(node):
                return ForLoopReader.read(node)

            if InfiniteLoopReader.is_repeat(node):
                return InfiniteLoopReader.read(node)

            if ObjectReader.is_object(node):    
                return ObjectReader.read(node)

            if AssertionReader.is_assertion(node):
                return AssertionReader.read(node)

            if DurationReader.is_duration(node):
                return DurationReader.read(node)

            if ExclamationReader.is_exclamation(node):
                return ExclamationReader.read(node)

            raise Exception("Unknown node type: {}".format(node.get('nom') if node.tag == 'satop' else node.find('nom').get('name')))

        except Exception as e:
            pass
        
        return None
    
class AttributeReader:
    @staticmethod
    def read(node):
        if node.tag == 'xml':
            return XMLReader.read(node)
        return Attribute(AttributeReader.get_type(node), AttributeReader.get_value(node))

    @staticmethod
    def get_type(node):
        return node.find('nom').get('name').split(':')[1]

    @staticmethod
    def get_value(node):
        return node.find('prop').get('name')

    @staticmethod
    def is_attribute(node):
        try:
            if node.tag == 'satop':
                return ':adj' in node.get('nom')
            return ':adj' in node.find('nom').get('name')
        except Exception as e:
            print(str(e))
        return False

class ModifierReader:
    @staticmethod
    def read(node):
        if node.tag == 'xml':
            return XMLReader.read(node)

        return Modifier(ModifierReader.get_type(node), ModifierReader.get_value(node))

    @staticmethod
    def get_type(node):
        return node.find('nom').get('name').split(':')[1]

    @staticmethod
    def get_value(node):
        for child in node.findall('diamond'):
            if child.get('mode') == 'attr':
                return AttributeReader.read(child)
        return None

    @staticmethod
    def is_attribute(node):
        try:
            if node.tag == 'satop':
                return ':modifier' in node.get('nom')
            return ':modifier' in node.find('nom').get('name')
        except Exception as e:
            print(str(e))
        return False

class RelationReader:
    @staticmethod
    def read(node):
        if node.tag == 'xml':
            return XMLReader.read(node)

        return Relation(RelationReader.get_type(node), ObjectReader.read(node))

    @staticmethod
    def get_type(node):
        for child in node.findall('diamond'):
            if child.get('mode') == 'predicate':
                return child.find('prop').get('name')
        return None

    @staticmethod
    def get_value(node):
        return node.find('prop').get('name')

class ObjectReader:
    @staticmethod
    def read(node):
        if node.tag == 'xml':
            return XMLReader.read(node)

        return ObjectReader.get_object(node)

    @staticmethod
    def get_type_name(node):
        if node.tag == 'satop':
            return node.get('nom').split(':')[1]
        return node.find('nom').get('name').split(':')[1]

    @staticmethod
    def get_object(node):
        if ObjectReader.is_anaphora(node):
            return Anaphora(
                ObjectReader.get_type_name(node),
                ObjectReader.get_name(node),
                ObjectReader.get_attributes(node),
                ObjectReader.get_relation(node),
                ObjectReader.get_limit(node)
            )

        if ObjectReader.is_dummy(node):
            return DummyObject(
                ObjectReader.get_type_name(node),
                ObjectReader.get_name(node),
                ObjectReader.get_attributes(node),
                ObjectReader.get_relation(node),
                ObjectReader.get_limit(node)
            )

        return Object(
            ObjectReader.get_type_name(node),
            ObjectReader.get_name(node),
            ObjectReader.get_attributes(node),
            ObjectReader.get_relation(node),
            ObjectReader.get_limit(node)
        )

    @staticmethod
    def get_name(node):
        return node.find('prop').get('name') if not ObjectReader.is_dummy(node) else '*'

    @staticmethod
    def get_attributes(node):
        attributes = []

        for child in node.findall('diamond'):
            if child.get('mode') == 'mod':
                attributes.append(ModifierReader.read(child))
            if child.get('mode') == 'attr':
                attributes.append(AttributeReader.read(child))

        return attributes

    @staticmethod
    def is_anaphora(node):
        return node.find('prop') is not None and node.find('prop').get('name') in ['it']

    @staticmethod
    def is_dummy(node):
        try:
            return node.find('nom').get('name').split(':')[1] == 'dummy'
        except:
            return

    @staticmethod
    def get_relation(node):
        for child in node.findall('diamond'):
            if child.get('mode') == 'relation':
                return RelationReader.read(child)
        return None

    @staticmethod
    def get_limit(node):
        determiner = None
        form = None
        number = None


        for child in node.findall('diamond'):
            if child.get('mode') == 'num':
                value = child.find('prop').get('name')

                if is_number(value):
                    number = float(value)
                else:
                    form = value

        for child in node.findall('diamond'):
            if child.get('mode') == 'det':
                if 'det' == 'the':
                    return Only()
                determiner = child.find('prop').get('name')

        if not form or form == 'pl':
            return None

        if number or determiner == 'a':
            return Any(number if number is not None else 1)

        return Only(number if number is not None else 1)

    @staticmethod
    def is_object(node):
        try:
            if node.tag == 'satop':
                return node.get('nom').split(':')[1] in ['object', 'tool', 'dummy', 'room']
            return node.find('nom').get('name').split(':')[1] in ['object', 'tool', 'dummy', 'room']
        except Exception as e:
            print(str(e))
        return False

class TaskReader:
    @staticmethod
    def read(node):
        if node.tag == 'xml':
            return XMLReader.read(node.find('lf')[0])

        arguments = TaskReader.get_method_args(node)

        return Task(TaskReader.get_method_name(node, arguments),
                    {'arg' + str(idx) : argument for idx, argument in enumerate(arguments)})


    @staticmethod
    def get_method_name(node, arguments):
        task_name = node.find('prop').get('name')
        type_names = []

        for diamond in node.findall('diamond'):
            if not diamond.get('mode').startswith('particle'):
                continue

            task_name = task_name + '_' + diamond.find('prop').get('name')


        for argument in arguments:
            type_names.append(argument.get_type_name())

        return task_name + '(' + ', '.join([type_name + ' arg' + str(idx)
                                             for idx, type_name in enumerate(type_names)]) + ')'

    @staticmethod
    def get_method_args(node, layer = 0):
        children = [child for child in node.findall('diamond') if child.get('mode').startswith('arg')]
        args = []
        for child in children:
            args.append(XMLReader.read(child))

        return args

    @staticmethod
    def is_task(node):
        try:
            if node.tag == 'satop':
                return ':action' in node.get('nom')
            return ':action' in node.find('nom').get('name')
        except Exception as e:
            print(str(e))
        return False

class ConditionalReader:
    @staticmethod
    def read(node):
        if node.tag == 'xml':
            return XMLReader.read(node.find('lf')[0])

        return Conditional(
            ConditionalReader.get_condition(node),
            ConditionalReader.get_body(node),
            ConditionalReader.is_inverted(node)
        )

    @staticmethod
    def is_conditional(node):
        try:
            if node.tag == 'satop':
                return ':conditional' in node.get('nom') and node.find('prop').get('name') == 'if'
            return ':conditional' in node.find('nom').get('name') and node.find('prop').get('name') == 'if'
        except Exception as e:
            print(str(e))
        return False

    @staticmethod
    def get_condition(node):
        children = node.findall('diamond')
        return XMLReader.read([child for child in children if child.get('mode') == 'condition'][0])

    @staticmethod
    def get_body(node):
        children = node.findall('diamond')
        body = [child for child in children if child.get('mode') == 'body'][0]

        if body.find('nom').get('name').split(':')[1] in ['conditional', 'action', 'conjunction']:
            return XMLReader.read(body)
        
        return None

    @staticmethod
    def is_inverted(node):
        children = node.findall('diamond')
        return [child for child in children if child.get('mode') == 'inverted'][0].find('prop').get('name') == 'true'

class EventReader:
    @staticmethod
    def read(node):
        if node.tag == 'xml':
            return XMLReader.read(node.find('lf')[0])

        return Event(
            ConditionalReader.get_condition(node),
            ConditionalReader.get_body(node),
            ConditionalReader.is_inverted(node)
        )

    @staticmethod
    def is_event(node):
        try:
            if node.tag == 'satop':
                return ':conditional' in node.get('nom') and node.find('prop').get('name') == 'when'
            return ':conditional' in node.find('nom').get('name') and node.find('prop').get('name') == 'when'
        except Exception as e:
            print(str(e))
        return False

class WhileLoopReader:
    @staticmethod
    def read(node):
        return WhileLoop(
          ConditionalReader.get_condition(node),
          ConditionalReader.get_body(node),
          ConditionalReader.is_inverted(node)
        )
      
    @staticmethod
    def is_while(node):
        try:
            if node.tag == 'satop':
                return ':conditional' in node.get('nom') and node.find('prop').get('name') == 'while'
            return ':conditional' in node.find('nom').get('name') and node.find('prop').get('name') == 'while' 
        except Exception as e:
           print(str(e))

        return None

class ForLoopReader:
    @staticmethod
    def read(node):
        return ForLoop(
          ForLoopReader.get_duration(node),
          ConditionalReader.get_body(node)
        )

    @staticmethod
    def get_duration(node):
        children = node.findall('diamond')
        return DurationReader.read([child for child in children if child.get('mode') == 'duration'][0])
      
    @staticmethod
    def is_for(node):
        try:
            if node.tag == 'satop':
                return ':loop' in node.get('nom') and node.find('prop').get('name') == 'for'
            return ':loop' in node.find('nom').get('name') and node.find('prop').get('name') == 'for' 
        except Exception as e:
           print(str(e))

        return None

class InfiniteLoopReader:
    @staticmethod
    def read(node):
        return InfiniteLoop(
          ConditionalReader.get_body(node)
        )

    @staticmethod
    def is_repeat(node):
        try:
            if node.tag == 'satop':
                return ':loop' in node.get('nom') and node.find('prop').get('name') == 'repeatedly'
            return ':loop' in node.find('nom').get('name') and node.find('prop').get('name') == 'repeatedly' 
        except Exception as e:
           print(str(e))

        return None

class DurationReader:
    @staticmethod
    def read(node):
        return Duration(
          DurationReader.get_units(node),
          DurationReader.get_time(node)
        )
    
    @staticmethod
    def get_units(node):
        return node.find('prop').get('name')
    
    @staticmethod
    def get_time(node):
        return float(node.find('diamond').find('prop').get('name'))

    @staticmethod
    def is_duration(node):
        try:
            if node.tag == 'satop':
                return ':duration' in node.get('nom')
            return ':duration' in node.find('nom').get('name')
        except Exception as e:
            print(str(e))
        return False

class ConjunctionReader:
    @staticmethod
    def read(node):
        if node.tag == 'xml':
            return XMLReader.read(node.find('lf')[0])

        left = ConjunctionReader.get_left(node)
        right = ConjunctionReader.get_right(node)

        return Conjunction(ConjunctionReader.get_tag(node), XMLReader.read(left), XMLReader.read(right))

    @staticmethod
    def get_tag(node):
        return node.find('prop').get('name')

    @staticmethod
    def get_left(node):
        children = node.findall('diamond')
        return [child for child in children if child.get('mode') == 'left'][0]

    @staticmethod
    def get_right(node):
        children = node.findall('diamond')
        return [child for child in children if child.get('mode') == 'right'][0]

    @staticmethod
    def is_conjunction(node):
        try:
            if node.tag == 'satop':
                return ':conjunction' in node.get('nom')
            return ':conjunction' in node.find('nom').get('name')
        except Exception as e:
            print(str(e))
        return False

class AssertionReader:
    @staticmethod
    def read(node):
        return Assertion(
            XMLReader.read(AssertionReader.get_body(node)),
            XMLReader.read(AssertionReader.get_attribute(node)),
        )

    @staticmethod
    def get_body(node):
        children = node.findall('diamond')
        return [child for child in children if child.get('mode') == 'arg0'][0]

    @staticmethod
    def get_attribute(node):
        children = node.findall('diamond')
        return [child for child in children if child.get('mode') == 'arg1'][0]
        
    @staticmethod
    def is_assertion(node):
        try:
            if node.tag == 'satop':
                return ':assert' in node.get('nom')
            return ':assert' in node.find('nom').get('name')
        except Exception as e:
            print(str(e))
        return False

class QueryReader:
    @staticmethod
    def read(node):
        if node.tag == 'xml':
            return XMLReader.read(node.find('lf')[0])

        return Query(QueryReader.get_predicate(node), XMLReader.read(QueryReader.get_descriptor(node)))

    @staticmethod
    def get_predicate(node):
        children = node.findall('diamond')
        return [child for child in children if child.get('mode') == 'arg0'][0].find('prop').get('name')

    @staticmethod
    def get_descriptor(node):
        children = node.findall('diamond')
        return [child for child in children if child.get('mode') == 'arg1'][0]

    @staticmethod
    def is_query(node):
        try:
            if node.tag == 'satop':
                return ':query' in node.get('nom')
            return ':query' in node.find('nom').get('name')
        except Exception as e:
            print(str(e))
        return False

class ExclamationReader:
    @staticmethod
    def is_exclamation(node):
        try:
            if node.tag == 'satop':
                return ':exclamation' in node.get('nom')
            return ':exclamation' in node.find('nom').get('name')
        except Exception as e:
            print(str(e))
        return False

    @staticmethod
    def read(node):
        if ExclamationReader.get_value(node) == 'positive':
          return Affirmative()
        return Negative()

    @staticmethod
    def get_value(node):
        return node.find('prop').get('name')

class StopReader:
    @staticmethod
    def is_stop(node):
        try:
            if node.tag == 'satop':
                return ':stop' in node.get('nom')
            return ':stop' in node.find('nom').get('name')
        except Exception as e:
            print(str(e))
        return False