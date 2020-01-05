class AmbigiousStatement(Exception):
    def __init__(self, value):
        self.value = value

    def set_object(self, object):
        self.object = object

    def get_object(self):
        return self.object

    def __str__(self):
        return self.value


class NullStatement(Exception):
    def __init__(self, value):
        self.value = value
        self.object = None

    def set_object(self, object):
        self.object = object

    def get_object(self):
        return self.object

    def __str__(self):
        return self.value

class UnknownMethodException(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return str(self.value)