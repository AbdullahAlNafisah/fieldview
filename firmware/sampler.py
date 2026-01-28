from i2c_nodes import make_i2c_nodes

class Sampler:
    def __init__(self):
        self.nodes = make_i2c_nodes()
        self.last = [(0.0,0.0,0.0,25.0) for _ in self.nodes]

    def read_all(self):
        out = []
        for i, node in enumerate(self.nodes):
            v = node.read()
            if v:
                self.last[i] = v
            x,y,z,t = self.last[i]

            # keep your even/odd face flip
            if (i % 2) == 0:
                out.extend((x, y, z, t))
            else:
                out.extend((-x, y, -z, t))
        return out
