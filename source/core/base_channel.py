import pickle


class BaseChannel():
    def load(self, filename):

        with open(filename, "rb") as infile:
            tmp = pickle.load(infile)
            self.__dict__.update(tmp)

    def save(self, filename):
        with open(filename, "wb") as outfile:
            pickle.dump(self.__dict__, outfile)
