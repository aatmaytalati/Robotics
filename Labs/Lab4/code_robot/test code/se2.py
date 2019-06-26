import numpy as np

class SE2(object):
    """Class representing a node in RRT
    """

    def __init__(self, d = (0, 0), tetha = 0):
        super(SE2, self).__init__()
        self.M = [[np.cos(tetha), -np.sin(tetha), d[0]], [np.sin(tetha), np.cos(tetha), d[1]], [0,0,1]]

    @property
    def x(self):
        return self.M[0][2]

    @property
    def y(self):
        return self.M[1][2]

    def __getitem__(self, key):
        assert (key == 0 or key == 1)
        return self.M[key][2]

    def __str__(self):
        return str(self.M)

    def __mul__(self, other):
        ret = SE2()
        if type(other) is SE2:
            ret.M = np.matmul(self.M, other.M)
            return ret
        if type(other) is list and len(other) == 2:
            other = [[other[0]],[other[1]],[1]]
            coord = np.matmul(self.M, other)
            return (coord[0][0], coord[1][0])
    
    def inv(self):
        ret = SE2()
        ret.M = np.linalg.inv(self.M)
        return ret

# Gets position angle from current node to coordinate
def getAngle(coord):
    return np.arctan2(coord[1],coord[0])