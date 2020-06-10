import time
import py_trees.common as common

from py_trees import Status
from py_trees.decorators import Decorator, Timeout

class RepeatForDuration(Timeout):
    def update(self):
        """
        Terminate the child and return :data:`~py_trees.common.Status.SUCCESS`
        if the timeout is exceeded.
        """
        current_time = time.time()
        if current_time > self.finish_time and self.children[0].status:
            self.feedback_message = "finished"
            self.logger.debug("{}.update() {}".format(self.__class__.__name__, self.feedback_message))
            # invalidate the decorated (i.e. cancel it), could also put this logic in a terminate() method
            self.decorated.stop(Status.INVALID)
            return Status.SUCCESS
        # Don't show the time remaining, that will change the message every tick and make the tree hard to
        # debug since it will record a continuous stream of events
        self.feedback_message = self.decorated.feedback_message + " [repeat until: {}]".format(self.finish_time)
        return Status.RUNNING

class Conditional(Decorator):
  def __init__(self,
                 child,
                 name=common.Name.AUTO_GENERATED,
                 predicate=''):
      
      super(Conditional, self).__init__(name=name, child=child)
      self.predicate = predicate

