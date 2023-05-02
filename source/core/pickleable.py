import os.path

import pickle


class Pickleable():
    """
    Base class for all pickleable classes.
    Pickleable classes can be saved and loaded from a file.
    """

    def load(self, filename):
        """Loads the object from a file."""
        if not os.path.isfile(filename):
            return

        with open(filename, "rb") as infile:
            tmp = pickle.load(infile)
            self.__dict__.update(tmp)

    def save(self, filename):
        """Saves the object to a file."""
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        with open(filename, "wb") as outfile:
            pickle.dump(self.__dict__, outfile)
