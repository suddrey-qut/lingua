import rospy

from py_trees.composites import Sequence, Selector, Composite
from py_trees.common import Status
from rv_trees.leaves import Leaf
from rv_leaves.leaves.generic.console import Print

from lingua.ccg_reader import CCGReader
from lingua.types import *
from lingua_pddl.state import State

from openccg_ros.srv import Parse
from std_msgs.msg import String

from .errors import *

class OneShotSequence(Composite):
  def __init__(self, name='Oneshot Sequence', children=None, *args, **kwargs):
    super(OneShotSequence, self).__init__(name=name, children=children if children else [], *args, **kwargs)
    self.current_child = None
    
  def tick(self):
      """
      Run the tick behaviour for this root.

      Yields:
          :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
      """
      self.logger.debug("%s.tick()" % self.__class__.__name__)
      
      if self.status != Status.RUNNING:
          self.initialise()

      self.update()

      if not self.children:
        self.status = Status.SUCCESS
        yield self
        return

      previous = self.current_child
      for child in self.children:
          for node in child.tick():
              yield node
              if node is child:
                  if node.status == Status.RUNNING or node.status == Status.SUCCESS:
                      self.current_child = child
                      self.status = Status.RUNNING
                      
                      if previous is None or previous != self.current_child:
                          # we interrupted, invalidate everything at a lower priority
                          passed = False
                          for child in self.children:
                              if passed:
                                  if child.status != Status.INVALID:
                                      child.stop(Status.INVALID)
                              passed = True if child == self.current_child else passed
                      
                      if child.status == Status.SUCCESS:
                        self.remove_child(child)

                      yield self
                      return
      
      try:
          self.current_child = self.children[-1]
      except IndexError:
          self.current_child = None
      yield self

  def stop(self, new_status=Status.INVALID):
      """
      Stopping a root behaviour requires setting the current child to none. Note that it
      is important to implement this here instead of terminate, so users are free
      to subclass this easily with their own terminate and not have to remember
      that they need to call this function manually.

      Args:
          new_status (:class:`~py_trees.common.Status`): the composite is transitioning to this new status
      """
      # retain information about the last running child if the new status is
      # SUCCESS or FAILURE
      if new_status == Status.INVALID:
          self.current_child = None
      Composite.stop(self, new_status)


class Root(OneShotSequence):
  def __init__(self, name='Root', children=None, *args, **kwargs):
    super(Root, self).__init__(name=name, children=children if children else [], *args, **kwargs)
    self.current_child = None
    self.listener = None

    self.sub_speech = rospy.Subscriber('/speech/in', String, self.speech_cb)
    self.pub_speech = rospy.Publisher('/speech/out', String, queue_size=1)

    self.parser = rospy.ServiceProxy('/ccg/parse', Parse)

    self.input_stack = []

    self.focus = None
    self.topic = None

  def set_listener(self, node):
    self.listener = node

  def get_listener(self):
    return self.listener

  def speech_cb(self, msg):
    # Get XML parses from CCG parser for input text
    result = self.parser(msg.data
      .replace(',', ' , ')
      .replace('  ', ' ')
    )

    success = False

    for xml in result.parses:
    
      utterance, t = CCGReader.read(xml)
      
      if t:
        print(str(t))

        self.input_stack.append(utterance)
        self.handle_anaphora(t)

        if self.get_listener():
          self.get_listener().set_input(t)
          success = True
          break

        subtree = t.to_btree()
        
        if not subtree.setup(timeout=1):
          continue
        
        self.add_child(subtree)
        
        self.pub_speech.publish(String(data='Okay'))
        
        success = True
        break
    
    if not success:
      self.pub_speech.publish(String(data='Sorry, I did not understand'))

  def handle_anaphora(self, node):
    replace = False

    if isinstance(node, Conditional):
      if node.is_inverted():
        if self.handle_anaphora(node.get_body()):
          node.set_body(self.topic[1])
        if self.handle_anaphora(node.get_condition()):
          node.set_condition(self.topic[1])
      else:
        if self.handle_anaphora(node.get_condition()):
          node.set_condition(self.topic[1])
        if self.handle_anaphora(node.get_body()):
          node.set_body(self.topic[1])

    if isinstance(node, ForLoop):
      if self.handle_anaphora(node.get_body()):
        node.set_body(self.topic[1])

    if isinstance(node, Conjunction):
      if self.handle_anaphora(node.get_left()):
        node.set_left(self.topic[1])
      if self.handle_anaphora(node.get_right()):
        node.set_right(self.topic[1])

    if isinstance(node, Assertion):
      if self.handle_anaphora(node.get_body()):
        node.set_body(self.topic[1])

    if isinstance(node, Task):
      located = False

      for key in node.get_argument_keys():
        if isinstance(node.get_argument(key), Assertion):
          if self.handle_anaphora(node.get_argument(key).get_descriptor()):
              node.get_argument(key).set_descriptor(self.topic[1])

        elif (isinstance(node.get_argument(key), Conjunction)):
          if self.handle_anaphora(node.get_argument(key).get_left()):
              node.get_argument(key).set_left(self.topic[1])
          if self.handle_anaphora(node.get_argument(key).get_right()):
              node.get_argument(key).set_right(self.topic[1])

        elif isinstance(node.get_argument(key), Anaphora):
          if not self.topic:
              raise Exception(self.input_stack[-1].replace(' it ', ' what ') + '?')

          node.set_argument_type(key, self.topic[0])
          node.set_argument(key, self.topic[1])

          located = True

      if not located:
        for key in node.get_argument_keys():
          self.topic = (node.get_argument_type(key), node.get_argument(key))
          break

      return False

    if isinstance(node, Object):
      if isinstance(node, Anaphora):
        if not self.topic:
          raise Exception(self.input_stack[-1].replace(' it ', ' what ') + '?')
        return True
      
      self.topic = (node.get_type_name(), node)

    return replace

class Subtree(Sequence):
  def __init__(self, name, method_name, arguments, mapping=None, *args, **kwargs):
    super(Subtree, self).__init__(name, children=[], *args, **kwargs)
    self.method_name = method_name
    self.mapping = mapping if mapping else {}
    self.arguments = arguments

  def setup(self, timeout):
    super(Subtree, self).setup(timeout)
    self.method = Method.methods[self.method_name]
    return True
  
  def initialise(self):
    self.remove_all_children()
    
    args = {}

    for type_name, key in self.method.get_arguments():
      if key not in self.mapping:
        continue
      args[key] = self.arguments[self.mapping[key]]

      if isinstance(self.arguments[self.mapping[key]], Groundable) and not args[key].is_grounded():
        try:
          args[key].ground(State())
        
        except AmbigiousStatement:
          resolver = DisambiguateGroundable(groundable=args[key])
          resolver.setup(0)

          self.add_child(resolver)
          
    
    self.add_child(self.method.instantiate(args).to_tree())
    super(Subtree, self).initialise()
    
  def terminate(self, new_status=Status.INVALID):
    super(Subtree, self).terminate(new_status)
    self.remove_all_children()  

class DisambiguateGroundable(FailureIsRunning):
  def __init__(self, name='Disambiguate Groundable', groundable=None):
    def merge(leaf):
      groundable.set_id(set(groundable.get_id()).intersection(leaf.loaded_data))
      return groundable.get_id()

    super(DisambiguateGroundable, self).__init__(name=name, child=Sequence(children=[
      Say(load_value='What item?'),
      PollInput(),
      GroundObjects(),
      Leaf(name='Merge', result_fn=merge)
    ]))

class Preconditions(Sequence):
  def __init__(self, name="Sequence", children=None, *args, **kwargs):
        super(Preconditions, self).__init__(name, children, *args, **kwargs)
        self.current_index = -1  # -1 indicates uninitialised
        self.reset_children = False
        self.resolver = None
        
  def initialise(self):
    super(Preconditions, self).initialise()
    self.reset_children = False
    self.resolver = None

  def terminate(self, new_status=Status.INVALID):
    super(Preconditions, self).terminate(new_status)
    
    if self.resolver is not None:
      self.remove_child(self.resolver)

  def tick(self):
      """
      Tick over the children.

      Yields:
          :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
      """
      self.logger.debug("%s.tick()" % self.__class__.__name__)
      if self.status != Status.RUNNING or self.reset_children:
          self.logger.debug("%s.tick() [!RUNNING->resetting child index]" % self.__class__.__name__)
          # sequence specific handling
          self.current_index = 0
          for child in self.children:
              # reset the children, this helps when introspecting the tree
              if child.status != Status.INVALID:
                  child.stop(Status.INVALID)
          # subclass (user) handling
          if not self.reset_children:
            self.initialise()
          self.reset_children = False

      # run any work designated by a customised instance of this class
      self.update()
      for child in itertools.islice(self.children, self.current_index, None):
          for node in child.tick():
              yield node
              if node is child and node.status != Status.SUCCESS:
                  if self.create_resolver():
                    self.status = Status.RUNNING
                  else:
                    self.status = node.status

                  yield self
                  return
          self.current_index += 1
      # At this point, all children are happy with their SUCCESS, so we should be happy too
      self.current_index -= 1  # went off the end of the list if we got to here
      self.stop(Status.SUCCESS)
      yield self


  def create_resolver(self):
    if self.resolver is not None:
      return False

    resolver = OneShotSequence(
      name='Resolution', children=[
        Planner(conditions=self.predicates)
      ]
    )
    resolver.setup(0)

    self.insert_child(resolver, 0)
    self.current_index += 1

    self.reset_children = True
    
    self.resolver = resolver
    return self.resolver

  @property
  def predicates(self):
    result = []
    for child in self.children:
      if hasattr(child, 'predicate'):
        print(child.predicate)
        result.append(child.predicate)
    print(result)
    return result

from .method import Method
from .leaves import Say, PollInput, GroundObjects, Planner
from .types import Groundable