from .errors import AmbigiousStatement, NullStatement

import rospy
import re
import random
import itertools
import timeit

from lingua_kb.srv import Ask, AskRequest
from lingua_kb.srv import Assert, AssertRequest

class Parser:
  _kb_ask = None  
  _kb_assert = None

  @staticmethod
  def init():
    Parser._kb_ask = rospy.ServiceProxy('/kb/ask', Ask)
    Parser._kb_assert = rospy.ServiceProxy('/kb/assert', Assert)
    Parser._kb_ask.wait_for_service()
    Parser._kb_assert.wait_for_service()


  @staticmethod
  def parse(query, as_conjunction = True, debugging = False):
      return Parser.recursive_parse(query, as_conjunction, 0)

  @staticmethod
  def recursive_parse(query, as_conjunction, layer):
      if Parser.is_atom(query):
          return query

      if Parser.is_conditional(query):
          return Parser.evaluate_condition(query)

      for term in Parser.logical_split(query)[1:]:
          result = Parser.recursive_parse(term, as_conjunction, layer + 1)
          query = query.replace(term, result, 1)

      if Parser.is_query(query):
          return Parser.evaluate_query(query)

      if Parser.is_intersection(query):
          return Parser.evaluate_intersection(query)

      if Parser.is_union(query):
          return Parser.evaluate_intersection(query)

      if Parser.is_disjunction(query):
          return Parser.evaluate_disjunction(query)

      if Parser.is_conjunction(query):
          return Parser.evaluate_conjunction(query)

      if Parser.is_limit(query):
          return Parser.evaluate_limit(query)

      if not Parser.is_iterable(query) and Parser.contains_iterable(query):
          return Parser.build_conjunction(query)

      return query

  @staticmethod
  def evaluate_query(condition, layer = 0):
      if not Parser.is_query(condition):
          return condition

      atoms = Parser.logical_split(condition)

      if len(atoms) == 3 and (is_iterable(atoms[1]) or is_iterable(atoms[2])):
          for idx, component in enumerate(atoms[1:]):
              if is_iterable(component):
                  atoms[idx + 1] = logical_split(component)[1:]

          retval = set()
          for x in list(itertools.product(atoms[1], atoms[2])):
              retval.add(evaluate_query('(' + atoms[0] + ' ' + x[0] + ' ' + x[1] + ')', layer))

          if len(retval) > 1:
              return '(set ' + ' '.join(list(retval)) + ')'

          return retval.pop()

      if not Parser._kb_ask:
        Parser.init()

      result = Parser._kb_ask(condition).data

      if not result:
          raise NullStatement(condition)
        
      if len(set(result)) > 1:
          return '(set ' + ' '.join(set(result)) + ')'

      return result[0]

  @staticmethod
  def assert_statement(statement):
      return Parser._kb_assert(statement).result

  @staticmethod
  def evaluate_condition(condition):
      if not Parser.is_conditional(condition):
          return condition

      terms = Parser.logical_split(condition)[1:]

      if Parser.assert_statement(terms[0]):
          return parse(terms[1])

      if len(terms) > 2:
          return parse(terms[2])

      return str()

  @staticmethod
  def evaluate_late(condition):
      if not Parser.is_late(condition):
          return condition

      return Parser.logical_split(condition)[1]

  @staticmethod
  def evaluate_intersection(condition):
      if not Parser.is_intersection(condition):
          return condition

      atoms = [Parser.logical_split(term)[(1 if Parser.is_iterable(term) else 0):] for term in Parser.logical_split(condition)[1:]]
      atoms = list(set.intersection(*[set(atom) for atom in atoms]))

      if not atoms:
          raise NullStatement(condition)

      return ('(set ' + ' '.join(atoms) + ')' if len(atoms) > 1 else atoms[0])

  @staticmethod
  def evaluate_union(condition):
      if not is_union(condition):
          return condition

      atoms = [Parser.logical_split(term)[(1 if Parser.is_iterable(term) else 0):] for term in Parser.logical_split(condition)[1:]]
      atoms = list(set.union(*[set(atom) for atom in atoms]))

      if len(atoms) > 1:
          return '(set ' + ' '.join(atoms) + ')'

      return atoms[0]

  @staticmethod
  def evaluate_conjunction(condition):
      if not Parser.is_conjunction(condition):
          return condition

      atoms = [Parser.logical_split(term)[(1 if Parser.is_iterable(term) else 0):] for term in Parser.logical_split(condition)[1:]]
      atoms = list(set.union(*[set(atom) for atom in atoms]))

      if len(atoms) > 1:
          return '(set ' + ' '.join(atoms) + ')'

      return atoms[0]

  @staticmethod
  def evaluate_disjunction(condition):
      if not Parser.is_disjunction(condition):
          return condition

      return Parser.logical_split(condition)[1]#random.choice(logical_split(condition)[1:])

  @staticmethod
  def evaluate_limit(condition):
      if not Parser.is_limit(condition):
          return condition

      terms = Parser.logical_split(condition)

      if Parser.is_iterable(terms[1]):
          atoms = Parser.logical_split(terms[1])[1:]
      else:
          atoms = [terms[1]]

      if terms[0] == 'only':
          if len(atoms) != int(terms[2]):
              raise AmbigiousStatement(condition)

          return terms[1]

      if len(atoms) < int(terms[2]):
          raise AmbigiousStatement(condition)

      result = []

      for _ in range(int(terms[2])):
          idx = 0
          result.append(atoms.pop(idx))

      if len(result) == 1:
          return result[0]

      return '(set ' + ' '.join(result) + ')'

  @staticmethod
  def logical_split(logical):
      tokens =  logical.replace('(', ' ( ').replace(')', ' ) ').split()
      return Parser.recursive_logical_split(tokens)

  @staticmethod
  def recursive_logical_split(tokens, layer = 0):
      token = tokens.pop(0)
      if '(' == token:
          L = []
          while tokens[0] != ')':
              L.append(Parser.recursive_logical_split(tokens, layer + 1))
          tokens.pop(0)

          if layer:
              return '(' + ' '.join(L) + ')'
          else:
              return L

      if layer > 0:
          return token

      return [token]

  @staticmethod
  def is_atom(term):
      return not term.startswith('(')

  @staticmethod
  def is_negative(term):
      return term.startswith('(not ')

  @staticmethod
  def is_query(term):
      return '?' in term

  @staticmethod
  def is_conditional(term):
      return term.startswith('(if ')

  @staticmethod
  def is_intersection(term):
      return term.startswith('(intersect ')

  @staticmethod
  def is_union(term):
      return term.startswith('(union ')

  @staticmethod
  def is_conjunction(term):
      return term.startswith('(and ')

  @staticmethod
  def is_disjunction(term):
      return term.startswith('(or ')

  @staticmethod
  def is_iterable(term):
      return term.startswith('(set ')

  @staticmethod
  def is_limit(term):
      return term.startswith('(only ') or term.startswith('(any ')

  @staticmethod
  def is_late(term):
      return term.startswith('(late ')

  @staticmethod
  def is_complement(term):
      return term.startswith('(!')

  @staticmethod
  def contains_iterable(term):
      return '(set ' in term

  @staticmethod
  def is_tautology(term):
      if Parser.is_atom(term):
          return False

      if not Parser._kb_ask:
        Parser.init()

      constituents = Parser.logical_split(term)

      predicate_name = constituents[0]
      inverse_name = state.kb.inverse(predicate_name)

      for idx, constituent in enumerate(constituents[1:]):
          if Parser.is_atom(constituent):
              continue

          child_consituents = Parser.logical_split(constituent)

          if child_consituents[0] == predicate_name:
              if child_consituents[1 + idx] == '?' and child_consituents[2 - idx] == constituents[2 - idx]:
                  return True

          if child_consituents[0] == inverse_name:
              if child_consituents[2 - idx] == '?' and child_consituents[1 + idx] == constituents[2 - idx]:
                  return True

      return False

  @staticmethod
  def negate(term):
      if term.startswith('(not '):
          return Parser.logical_split(term)[1]
      return '(not ' + term + ')'

  @staticmethod
  def build_conjunction(term):
      if Parser.is_iterable(term) or not Parser.contains_iterable(term):
          return term

      atoms = logical_split(term)
      pieces = []

      for atom in atoms:
          atom = Parser.build_conjunction(atom)

          if Parser.is_iterable(atom):
              pieces.append(logical_split(atom)[1:])
          else:
              pieces.append([atom])

      pieces = itertools.product(*pieces)

      return '(and ' + ' '.join(['(' + ' '.join(piece) + ')' for piece in pieces]) + ')'