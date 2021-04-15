
#______________________________________________________________________________
# Queues: LIFOQueue (also known as Stack), FIFOQueue, PriorityQueue

class Queue:
    """
    Queue is an abstract class/interface. There are three types:
        LIFOQueue(): A Last In First Out Queue.
        FIFOQueue(): A First In First Out Queue.
        PriorityQueue(order, f): Queue in sorted order (min-first).
    Each type of queue supports the following methods and functions:
        q.append(item)  -- add an item to the queue
        q.extend(items) -- equivalent to: for item in items: q.append(item)
        q.pop()         -- return the top item from the queue
        len(q)          -- number of items in q (also q.__len())
        item in q       -- does q contain item?
    """

    def __init__(self):
        raise NotImplementedError

    def extend(self, items):
        for item in items: self.append(item)

def LIFOQueue():
    """
    Return an empty list, suitable as a Last-In-First-Out Queue.
    Last-In-First-Out Queues are also called stacks
    """
    return []


import collections # for dequeue
class FIFOQueue(collections.deque):
    """
    A First-In-First-Out Queue.
    """
    def __init__(self):
        collections.deque.__init__(self)
    def pop(self):
        return self.popleft()


import heapq
class PriorityQueue(Queue):
    """
    A queue in which the minimum  element (as determined by f) is returned first.
    The item with minimum f(x) is returned first
    """
    def __init__(self, f=lambda x: x):
        self.A = []
        self.f = f
    def append(self, item):
        heapq.heappush(self.A, (self.f(item), item))
    def __len__(self):
        return len(self.A)
    def __str__(self):
        return str(self.A)
    def pop(self):
        return heapq.heappop(self.A)[1]
        # (self.f(item), item) is returned by heappop
        # (self.f(item), item)[1]   is item
    def __contains__(self, item):
        # Note that on the next line a generator is used!
        return any(x==item for _, x in self.A)
    def __getitem__(self, key):
        for _, item in self.A:
            if item == key:
                return item
    def __delitem__(self, key):
        for i, (value, item) in enumerate(self.A):
            if item == key:
                self.A.pop(i)
                return

def memoize(fn):
    """Memoize fn: make it remember the computed value for any argument list"""
    def memoized_fn(*args):
        if args not in memoized_fn.cache:
            memoized_fn.cache[args] = fn(*args)
        return memoized_fn.cache[args]
    memoized_fn.cache = {}
    return memoized_fn

def update(x, **entries):
    """Update a dict; or an object with slots; according to entries.
    >>> update({'a': 1}, a=10, b=20)
    {'a': 10, 'b': 20}
    >>> update(Struct(a=1), a=10, b=20)
    Struct(a=10, b=20)
    """
    if isinstance(x, dict):
        x.update(entries)
    else:
        x.__dict__.update(entries)
    return x
