import random
import numpy as np
import matplotlib.pyplot as plt

# define the class graph with the startposition endposition and graphbounds

class Graph:

    def __init__(self, startpos, endpos,graphbound):
        self.startpos = startpos
        self.endpos = endpos
        self.xmin, self.ymin = graphbound[0]
        self.xmax, self.ymax = graphbound[1]

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos: 0}
        self.neighbors = {0: []}
        self.distances = {0: 0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))

    def randomPosition(self):
        x = round(random.uniform(self.xmin,self.xmax),4)
        y = round(random.uniform(self.ymin,self.ymax),4)
        return x, y


def distance(node1, node2):
    return np.linalg.norm(np.array(node1) - np.array(node2))


def isinsideobstacle(node, obstacle):
    x, y = node
    obs = obstacle.copy()
    while len(obs) > 0:
        cx, cy, cr = obs.pop(0)
        d = (x - cx) ** 2 + (y - cy) ** 2
        if d <= cr ** 2:
            return True
        else:
            return False


def intersect(x1, x2, y1, y2, obstacles):
    obs = obstacles.copy()
    while len(obs) > 0:
        cx, cy, cr = obs.pop(0)
        for i in range(0, 151):
            u = i / 100
            x = x1 * u + x2 * (1 - u)
            y = y1 * u + y2 * (1 - u)
            d = (x - cx) ** 2 + (y - cy) ** 2

            if d <= cr ** 2:
                return True
    return False


def nearest(G, vex, obstacles):
    Nvex = None
    Nidx = None
    minDist = float("inf")
    x1, y1 = vex
    for idx, v in enumerate(G.vertices):
        x2, y2 = v

        if intersect(x1, x2, y1, y2, obstacles):
            continue

        dist = distance(v, vex)
        if dist < minDist:
            minDist = dist
            Nidx = idx
            Nvex = v

    return Nvex, Nidx


def AddnewNode(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min(stepSize, length)

    newvex = (nearvex[0] + dirn[0], nearvex[1] + dirn[1])
    return newvex


def reconstruct_path(cameFrom, endpos, endid):
    # gives the path from start to end using parent tracing from cameFrom dictionaty input

    total_path = [[endid, endpos]]
    path_cost = 0
    while endid in cameFrom.keys():
        parent, node_coords = cameFrom[endid]
        total_path.insert(0, [parent, node_coords])  # insert the paernt node before the current node in total path list
        endid = parent
    return total_path


def RRT(startpos, endpos, graphbound, obstacles, itr, stepSize):
    cameFrom = {}
    G = Graph(startpos, endpos, graphbound)

    for i in range(itr):

        rand_node = G.randomPosition()

        if isinsideobstacle(rand_node, obstacles):
            continue

        near_node, near_id = nearest(G, rand_node, obstacles)
        # continue loop if cant find nearest node
        if near_node is None:
            continue

        # add new node if its out of obstacles and nearest node is found
        new_node = AddnewNode(rand_node, near_node, stepSize)
        new_node_id = G.add_vex(new_node)

        cameFrom[new_node_id] = [near_id, near_node]

        dist = distance(new_node, near_node)
        G.add_edge(new_node_id, near_id, dist)

        dist = distance(new_node, G.endpos)
        if dist < 0.05:
            end_id = G.add_vex(G.endpos)
            G.add_edge(new_node_id, end_id, dist)
            G.success = True
            cameFrom[end_id] = [near_id, near_node]
            print('success')
            break  # break the loop when goal is reached
        print(i)
    return G, cameFrom,end_id  # return graph, parent list(cameFrom) and id of end node

startpos = (-0.5,-0.5)
endpos = (0.5,0.5)
graphbound = [(-0.5,-0.5),(0.5,0.5)]
obstacles = [(-0.285,-0.075,0.165),(0.365,-0.295,0.135),(0.205,0.155,0.075)]

graph,cameFrom,endid = RRT(startpos, endpos, graphbound, obstacles, 500, 0.2)

print("end id of the end node is " , endid)

path = reconstruct_path(cameFrom,endpos, endid)
print(path)

ax = plt.gca()
xmin, ymin = graphbound[0]
xmax, ymax = graphbound[1]
ax.set(xlim=(xmin - 0.2, xmax + 0.2), ylim=(ymin - 0.2, ymax + 0.2))
# plt.axis([bounds[0]-0.5, bounds[1]+0.5, bounds[0]-0.5, bounds[1]+0.5])
width = xmax - xmin
height = ymax - ymin
ax.add_patch(plt.Rectangle(graphbound[0], width, height, fill=False, color='cyan'))
plt.plot(xmin, ymin, 'bo')
plt.plot(xmax, ymax, 'ro')

for x1, y1, z1 in obstacles:
    circle = plt.Circle((x1, y1), z1, color='k')
    ax.add_patch(circle)

x_list = []
y_list = []

for nodeid, nodecords in path:
    x, y = nodecords
    x_list.append(x)
    y_list.append(y)

plt.plot(x_list, y_list)
plt.axis('equal')
plt.show()