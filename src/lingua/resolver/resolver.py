from .search.search import *
import itertools
import sys

from ..types import DummyObject

class Resolver(Problem):
    frequency = {}

    def __init__(self, initial, goal, methods):
        self.initial = initial
        self.goal = goal
        self.methods = methods

    def actions(self, state):
        actions = []

        for method_name in sorted(self.methods, key=self.sort):
            method = self.methods[method_name]
            groundings = list(ground(state, method.get_arguments()))
            
            try:
              for grounding in groundings:
                  args = {}
                  for type_name, arg_id in method.get_arguments():
                      obj = DummyObject(type_name, '')
                      obj.set_id(grounding[arg_id])
                      args[arg_id] = obj

                  instantiated = method.instantiate(args)
                  if instantiated.is_applicable(state) and not instantiated.is_complete(state):
                      actions.append(instantiated)
            except KeyError:
              pass
            
        return actions

    def result(self, state, method):
        return method.result(state)

    def goal_test(self, state):
        for goal in self.goal:
            if not state.is_satisfied(goal):
                return False
        return True

    def conflict(self, state):
        return set(self.goal).difference(state)

    def heuristic(self, node):
        state = node.state
        score = 0

        for goal in self.goal:
            if not state.is_satisfied(goal):
                score += 1

        return score

    def solve(self):
        result = iterative_deepening_search(self, limit=4)

        if not result:
            return None

        result = result.solution()

        for instantiated in result:
            method_name = instantiated.name
            if not method_name in Resolver.frequency:
                Resolver.frequency[method_name] = 0

            Resolver.frequency[method_name] += 1

        return result

    def sort(self, method_name):
        return sys.maxsize - (Resolver.frequency[method_name] if method_name in Resolver.frequency else 0)

def ground(state, arguments):
    mapping = {}
    explored = set()
    for type_name, arg in arguments:
        parent_types, child_types = state.get_hierarchy(type_name)
        
        for class_label in parent_types + child_types + [type_name]:
            try:
                result = state.ask('(class_label ' + class_label + ' ?)')

                if not result:
                    continue

                mapping[arg] = result
                
            except Exception as e:
                pass

    return dict_product(mapping)

def dict_product(dicts):
    return (dict(list(zip(dicts, x))) for x in itertools.product(*iter(list(dicts.values()))))