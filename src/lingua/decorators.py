import time

from py_trees import Status
from py_trees.decorators import Timeout

class RepeatForDuration(Timeout):
    def update(self):
        """
        Terminate the child and return :data:`~py_trees.common.Status.SUCCESS`
        if the timeout is exceeded.
        """
        current_time = time.time()
        if current_time > self.finish_time:
            self.feedback_message = "finished"
            self.logger.debug("{}.update() {}".format(self.__class__.__name__, self.feedback_message))
            # invalidate the decorated (i.e. cancel it), could also put this logic in a terminate() method
            self.decorated.stop(Status.INVALID)
            return Status.SUCCESS
        # Don't show the time remaining, that will change the message every tick and make the tree hard to
        # debug since it will record a continuous stream of events
        self.feedback_message = self.decorated.feedback_message + " [repeat until: {}]".format(self.finish_time)
        return Status.RUNNING

