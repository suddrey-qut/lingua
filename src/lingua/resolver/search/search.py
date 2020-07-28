
"""

This search module is based on the AIMA book.
Search (Chapters 3-4)

The way to use this code is to subclass Problem to create a class of problems,
then create problem instances and solve them with calls to the various search
functions.

Last modified 2015-03-07
by f.maire@qut.edu.au

"""
from lingua.utilities import *
import sys

# Data Types
expanded = 0

def get_expanded():
    return expanded

def set_expanded(n):
    global expanded
    expanded = n

def reset_expanded():
    set_expanded(0)

class Problem(object):
    """The abstract class for a formal problem.  You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal.  Your subclass's constructor can add
        other arguments."""
        self.initial = initial; self.goal = goal

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        raise NotImplementedError

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        raise NotImplementedError

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal, as specified in the constructor. Override this
        method if checking against a single self.goal is not enough."""
        return state == self.goal

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

class Node:
    """
    A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state.  Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node.  Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class.
    """

    def __init__(self, state, parent=None, action=None, path_cost=0):
        "Create a search tree Node, derived from a parent by an action."
        update(self, state=state, parent=parent, action=action,
               path_cost=path_cost, depth=0)
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def expand(self, problem):
        "List the nodes reachable in one step from this node."
        global expanded
        expanded += 1
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        "Fig. 3.10"
        next = problem.result(self.state, action)
        return Node(next, # next is a state
                    self, # parent is a node
                    action, # from this state to next state
                    problem.path_cost(self.path_cost, self.state, action, next)
                    )

    def solution(self):
        "Return the sequence of actions to go from the root to this node."
        return [node.action for node in self.path()[1:]]

    def path(self):
        "Return a list of nodes forming the path from the root to this node."
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        return hash(self.state)

# Search Algorithms

def breadth_first_tree_search(problem):
    "Search the shallowest nodes in the search tree first."
    return tree_search(problem, FIFOQueue())

def breadth_first_graph_search(problem):
    "Search the shallowest nodes in the search tree first."
    return graph_search(problem, FIFOQueue())

def depth_first_tree_search(problem):
    "Search the deepest nodes in the search tree first."
    return tree_search(problem, LIFOQueue())

def depth_first_graph_search(problem):
    "Search the deepest nodes in the search tree first."
    return graph_search(problem, LIFOQueue())

def uniform_cost_search(problem):
    return priority_graph_search(problem, lambda node: node.path_cost)

def depth_limited_search(problem, limit=50):
    def recursive_dls(node, problem, limit):
        if problem.goal_test(node.state):
            return node
        elif node.depth == limit:
            return 'cutoff'
        else:
            cutoff_occurred = False
            for child in node.expand(problem):
                result = recursive_dls(child, problem, limit)
                if result == 'cutoff':
                    cutoff_occurred = True
                elif result is not None:
                    return result
            if cutoff_occurred:
                return 'cutoff'
            else:
                return None

    # Body of depth_limited_search:
    return recursive_dls(Node(problem.initial), problem, limit)

def iterative_deepening_search(problem, limit=sys.maxsize):
    for depth in range(limit):
        result = depth_limited_search(problem, depth)
        if result != 'cutoff':
            return result

# Informed Search

def greedy_search(problem, h=None):
    h = memoize(h or problem.h)
    return priority_graph_search(problem, lambda n: h(n))

def astar_search(problem, h=None):
    h = memoize(h or problem.h)
    return priority_graph_search(problem, lambda n: n.path_cost + h(n))


# Base Search Functions

def tree_search(problem, frontier):
    """
        Search through the successors of a problem to find a goal.
        The argument frontier should be an empty queue.
        Don't worry about repeated paths to a state. [Fig. 3.7]
        Return
             the node of the first goal state found
             or None is no goal state is found
    """
    assert isinstance(problem, Problem)
    frontier.append(Node(problem.initial))
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            return node
        frontier.extend(node.expand(problem))
    return None

def graph_search(problem, frontier):
    """
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    If two paths reach a state, only use the first one. [Fig. 3.7]
    Return
        the node of the first goal state found
        or None is no goal state is found
    """
    assert isinstance(problem, Problem)
    frontier.append(Node(problem.initial))
    explored = set() # initial empty set of explored states
    while frontier:
        node = frontier.pop()

        if problem.goal_test(node.state):
            return node
        explored.add(node.state)

        # Python note: next line uses of a generator
        frontier.extend(child for child in node.expand(problem)
                        if child.state not in explored
                        and child not in frontier)

    return None

def priority_graph_search(problem, f):
    """
    Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned.
    """
    f = memoize(f)
    node = Node(problem.initial)
    if problem.goal_test(node.state):
        return node
    frontier = PriorityQueue(f)
    frontier.append(node)
    explored = set()
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            return node
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                incumbent = frontier[child] # incumbent is a node
                if f(child) < f(incumbent):
                    del frontier[incumbent]
                    frontier.append(child)
    return None
