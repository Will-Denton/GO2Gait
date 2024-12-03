#!/usr/local/bin/python3
# ^ put path to your python3 here if running prm.py as executable

import matplotlib.pyplot as plt
import random
import numpy as np
import math
from collections import deque
import copy
from workspace import *


def bfs(graph, start, goal):
    '''Breadth-first search algorithm, finds path with fewest "hops"'''
    todo = deque()
    visited = set()
    init_path = [start]
    todo.append(init_path)
    while len(todo) > 0:
        curr_path = todo.popleft()
        curr_pt = curr_path[-1]
        if curr_pt is goal:
            return curr_path
        if curr_pt not in visited:
            visited.add(curr_pt)
            for pt in graph[curr_pt]:
                new_path = copy.copy(curr_path) # path is list of points. shallow copy of points is fine
                new_path.append(pt)
                todo.append(new_path) 
                pass
            pass
        pass
    return None # no valid path


class ConnectionError(Exception):
    '''Exception for when node not connected to roadmap'''
    pass


class PRM:
    '''Class for a probabilistic roadmap path-finding algorithm in planar C-space'''

    def __init__(self, ws, start, goal):
        '''Initializes a PRM planner

        Takes workspace object, start and goal points, 
        Roadmap is conceptually an adjacency list: 
          nodes are configs in free cspace and 
          edges are collision-free paths, 
          implemented as dictionary of Points to set of Points
        Path initialized as no path from start to goal found yet
        '''
        self.ws = ws
        self.start = start
        self.goal = goal
        self.roadmap = {} 
        self.path = None
        pass

    def line_collision(self,p1,p2):
        '''Samples points along edge to detect collision'''
        n_pts = int(dist(p1,p2)/max(self.ws.width,self.ws.height)*100)
        xs = np.linspace(p1.x,p2.x,n_pts)
        yx = np.linspace(p1.y,p2.y,n_pts)
        for i in range(n_pts):
            if self.ws.is_collision(Point(xs[i],yx[i])):
                return True
            pass
        return False

    def add_node(self,pt):
        '''Adds node to roadmap if no collision'''
        if not self.ws.is_collision(pt):
            self.roadmap[pt] = set()
            return True
        return False

    def add_edge(self,p1,p2):
        '''Adds edge to roadmap if no collision'''
        if not self.line_collision(p1, p2):
            self.roadmap[p1].add(p2)
            self.roadmap[p2].add(p1)
            return True
        return False

    def sample_cspace(self,n_samples):
        '''Generates n_samples random samples of the C-space

        Adds samples to roadmap if there is no collision
        '''
        count = 0
        while count < n_samples: # keep trying until n_samples added
            x = random.random() * self.ws.width
            y = random.random() * self.ws.height
            if self.add_node(Point(x, y)): # add node if no collision
                count += 1 # only increment if node added
        pass

    def nearest_pts(self, p1):
        '''Sorted list of points and distance to p1'''
        pts = [(pt,dist(pt,p1)) for pt in self.roadmap.keys() if not pt is p1]
        pts.sort(key = lambda x: x[1]) # sort by second elem in tuple (dist)
        return pts

    def connect_one(self, p1):
        '''Connects point to closest node in roadmap'''
        if not self.add_node(p1):
            raise ConnectionError # node has collision
        pts = self.nearest_pts(p1)
        added = self.add_edge(p1,pts[0][0])
        ind = 1
        while (ind < len(pts) and not added):
            added = self.add_edge(p1,pts[ind][0])
            ind += 1
            pass
        if not added:
            raise ConnectionError # no collision free edge possible
        pass

    def connect_samples(self,k):
        '''Connects each sample to k nearest neighbors'''
        for p1 in self.roadmap.keys():
            pts = self.nearest_pts(p1)
            for i in range(k):
                self.add_edge(p1, pts[i][0])
                pass
            pass
        pass

    def plan(self,n_samples=50,k=3):
        '''Plans a path by PRM algorithm:

        1) Sample configurations in free C-space
        2) Connect samples if sample and line segment are collision-free
        3) Attempt to connect start and goal vertices
        4) Attempt to find path from start to goal (using BFS)
        5) If no path is found, report failure
        '''
        print(f'Planning path with {n_samples} samples and {k} nearest neighbors')

        # Sample from C-space
        self.sample_cspace(n_samples)

        # Connect each sample to its k nearest neighbors
        # (See connect_samples above)
        self.connect_samples(k)
        
        # Connect start and goal nodes
        # (See connect_one above)
        try:
            self.connect_one(self.start)
            self.connect_one(self.goal)
        except ConnectionError:
            print('Failed to connect start or goal')
            return
        
        # Search the roadmap for a path from start to goal
        # (See BFS provided above)
        self.path = bfs(self.roadmap, self.start, self.goal)

        # Print the path
        if not self.path:
            print('No path found')
        else:
            print(f'Found path {self.path}')
            pass
        pass
        
    def plot(self):
        '''Plots workspace, each point in roadmap, edges, and a path if one is found'''
        self.ws.plot()
        for p1,adj in self.roadmap.items():
            plt.plot(p1.x,p1.y,'o')
            for p2 in adj:
                plt.plot([p1.x,p2.x],[p1.y,p2.y])
                pass
            pass
        if self.path:
            x = [pt.x for pt in self.path]
            y = [pt.y for pt in self.path]
            plt.plot(x, y, 'k-')
            pass
        pass
    pass


def main():
    # Here is one test case. Feel free to write your own!
    # Make a width x height workspace
    width=5
    height=4
    # Obstacles are a list of tuples representing rectangles (x,y,w,h)
    obs = [(1.0,1.0,1.5,.3),(2.5,2.5,1.0,1.0),(3.5,.5,.5,.5)]
    ws = RectWorkspace(width,height,obs)
    # Create PRM object with workspace, start, goal
    prm = PRM(ws,Point(0.5,0.5),Point(4.0,3.0))
    # Execute plan and plot methods
    prm.plan(20,3) # change n_samples and k neighbors
    prm.plot()
    plt.show()
    pass


if __name__ == '__main__':
    main()
