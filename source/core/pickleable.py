import os.path

import pickle


class Pickleable():
    def load(self, filename):
        if not os.path.isfile(filename):
            return

        with open(filename, "rb") as infile:
            tmp = pickle.load(infile)
            self.__dict__.update(tmp)

    def save(self, filename):
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, "wb") as outfile:
            pickle.dump(self.__dict__, outfile)
