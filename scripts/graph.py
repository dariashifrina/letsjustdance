
example_nodes = [(0,0),(3,0),(0,3)(3,3)]

class Graph:
        def __init__(self, nodes):
                # this makes an undirected graph, we could do a directed one too?
                # take in a list of nodes stored as tuples
                self.edges = {} # list of edges to be used in bfs
             
                for node in nodes:
                        # for movement purposes, we want edges[node] = [left_node, right_node, up_node, down_node]
                        # edges[node][i] = -1 if movement in that direction is impossible

                        new = [-1]*4

                        for point in nodes:
                                if node[0] in point or node[1] in point:
                                        n = 3 # size of grid square
                                        if point[0] == node[0]-n and point[1] == node[1]:
                                                new[0] = point
                                        if point[0] == node[0] + n and point[1] == node[1]:
                                                new[1] = point
                                        if point[1] == node[1]+n and point[0] == node[0]:
                                                new[2] = point
                                        if point[1] == node[1]-n and point[0] == node[0]:
                                                new[3] = point
                        self.edges[node] = new

        def remove_edge(self, src, goal):
                # method to remove edge that is no longer navigable(i.e traffic cone blocking, etc)
                # remove src -> goal edge:
                direction = self.edges[src].index(goal)
                self.edges[src][direction] = -1

                direction = self.edges[goal].index(src)
                self.edges[goal][direction] = -1

        def is_equal(self,node1,node2):
                # check if two nodes are equal
                return node1[0] == node2[0] and node1[1] == node2[1] 

        def plan_path(self, src, goal):
        # bfs search to find fastest path on unweighted, undirected graph
                explored = [] # array of explored nodes
                queue = [[src]] # array of paths
                if self.is_equal(src, goal):
                        return []
                
                while queue:
                        path = queue.pop(0) # queue stores the paths of each branch
                        node = path[-1] # last item of path

                        if node not in explored:
                                adjacent = [i for i in self.edges[node] if i != -1] # find neighbors
                                for nbr in adjacent:
                                        new_path = list(path)
                                        new_path.append(nbr)
                                        queue.append(new_path)

                                        if nbr == goal:
                                                return new_path
                                explored.append(node)
                return -1
