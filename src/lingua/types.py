import copy
from .parser import Parser
from .errors import AmbigiousStatement, NullStatement

class Groundable:
    def __init__(self, descriptor):
        self.descriptor = descriptor
        self.arguments = {}
        self.id = None

        self.persistent = {}

    def get_descriptor(self, raw=False):
        if raw:
            return self.descriptor

        descriptor = self.descriptor
        for key in self.arguments:
            descriptor = descriptor.replace(key, self.arguments[key].get_descriptor())
        for key in self.persistent:
            descriptor = descriptor.replace(key, self.persistent[key].get_descriptor())
        return descriptor

    def set_descriptor(self, value):
        self.descriptor = value

    def ground(self):
        descriptor = self.descriptor

        for key in self.arguments:
            if not self.arguments[key].is_grounded():
                self.arguments[key].ground()
            descriptor = descriptor.replace(key, self.arguments[key].get_id())

        for key in self.persistent:
            if not self.persistent[key].is_grounded():
                self.persistent[key].ground()
            descriptor = descriptor.replace(key, self.persistent[key].get_id())

        try:
            self.set_id(Parser.parse(descriptor))

        except AmbigiousStatement as e:
            e.set_object(self)
            raise e

        except NullStatement as e:
            e.set_object(self)
            raise e

    def get_id(self):
        return self.id

    def set_id(self, id):
        self.id = id

    def is_grounded(self):
        return self.id is not None

    def get_arguments(self):
        return self.arguments

    def get_argument(self, key):
        return self.arguments[key]

    def set_arguments(self, arguments):
        for key in self.persistent:
            self.persistent[key].set_arguments(arguments)
        self.arguments = arguments

    def get_persistent(self):
        return self.persistent

    def simplify(self, parent_arguments, verbose=False):
        if not self.is_grounded():
            raise Exception('groundable must be grounded in order to simplify')

        parent_arguments = {'arg' + str(idx): arg for idx, arg in enumerate(parent_arguments)}

        components = Parser.logical_split(self.descriptor)
        ids = Parser.logical_split(self.get_id())

        for arg_id in parent_arguments:
            argument = parent_arguments[arg_id]

            if argument.get_id() not in ids:
                continue

            idx = ids.index(argument.get_id())

            if components[idx] == arg_id:
                continue

            components[idx] = arg_id

        self.descriptor = '(' + ' '.join(components) + ')'

        new_persistent = {}
        for p_id in self.persistent:
            if p_id in self.descriptor:
                new_persistent[p_id] = self.persistent[p_id]
        self.persistent = new_persistent

    def set_persistent(self, persistent, clear=False):
        arguments = copy.deepcopy(self.arguments)

        for idx, key in enumerate(persistent):
            self.descriptor = self.descriptor.replace(key, 'const' + str(idx))
            self.persistent['const' + str(idx)] = persistent[key]
            arguments.pop(key)

    def replace_with_persistent(self, parent_arguments):
        parent_arguments = {'arg' + str(idx): arg for idx, arg in enumerate(parent_arguments)}
        persistent = {}

        arguments = copy.deepcopy(self.arguments)

        for key in self.get_arguments():
            argument = self.get_argument(key)
            located = False

            if key not in self.descriptor:
                continue

            for p_key in parent_arguments:
                p_argument = parent_arguments[p_key]

                if argument.get_descriptor() == p_argument.get_descriptor():
                    located = True
                    break

            if not located:
                argument.replace_with_persistent(list(parent_arguments.values()))
                persistent[key] = argument

        self.set_persistent(persistent)

        for p_key in parent_arguments:
            for key in arguments:
                if parent_arguments[p_key].get_descriptor() == arguments[key].get_descriptor():
                    self.descriptor = self.descriptor.replace(key, p_key)
                    break

    def clear(self):
        for key in self.persistent:
            self.persistent[key].clear()
        self.arguments = {}
        self.id = None

    def equals(self, other):
        if not isinstance(other, self.__class__):
            return False

        if self.is_grounded() or other.is_grounded():
            return self.get_id() == other.get_id()

        return self.descriptor == other.descriptor

    def __str__(self):
        return str(self.get_descriptor()) + ('' if not self.id else ' - ' + self.id)

    def __contains__(self, item):
        return item in self.descriptor

class Object(Groundable):
    def __init__(self, descriptor='', type_name='obj'):
        Groundable.__init__(self, descriptor)
        self.type_name = type_name

    def get_type_name(self):
        return self.type_name

    def set_type_name(self, value):
        self.type_name = value

    def is_anaphora(self):
        return 'anaphora:it' in self.descriptor

class DummyObject(Groundable):
    def __init__(self, type_name, descriptor = ''):
        Groundable.__init__(self, descriptor)
        self.type_name = type_name

    def ground(self, state):
        raise Exception('unable to ground dummy object')

class Query(Groundable):
    def __init__(self, predicate, descriptor):
        self.predicate = predicate
        self.descriptor = descriptor
        self.id = None

    def get_descriptor(self):
        return self.descriptor

    def set_descriptor(self, descriptor):
        self.descriptor = descriptor

    def get_id(self):
        return self.id

    def set_id(self, id):
        self.id = id

    def get_type_name(self):
        return '?'

    def __str__(self):
        return '({0} ? {1})'.format(self.predicate, self.descriptor)