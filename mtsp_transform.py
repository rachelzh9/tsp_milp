import random
import gurobipy as gp
from gurobipy import GRB
import numpy as np
import matplotlib.pyplot as plt
from itertools import combinations


# tested with Python 3.7 & Gurobi 9.0.0
def distance(c1, c2):
    diff = (c1[0] - c2[0], c1[1] - c2[1])
    return np.sqrt(diff[0] * diff[0] + diff[1] * diff[1])


def createGraph(m, env_size):
    nodes = []
    adj = np.zeros((m+n-1, m+n-1))+1000
    for i in range(m):
        nodes.append([random.uniform(0,1)*env_size, random.uniform(0,1)*env_size])
    nodes = np.array(nodes)
    for i in range(m):
        for j in range(i+1,m):
            adj[i, j] = distance(nodes[i,:], nodes[j,:])
            adj[j, i] = adj[i,j]
    # add dummys
    for i in range(m, m + n-1):
        for j in range(1,m):
            adj[i, j] = 0
            adj[j, i] = adj[i, j]
    for i in range(m, m + n-1):
        adj[i, 0] = 1000
        adj[0, i] = 1000
        for j in range(i + 1, m + n-1):
            adj[i, j] = 1000
            adj[j, i] = adj[i, j]

    return nodes, adj


def create_dist(idx):
    dist = {}
    m = len(idx)

    for i in range(m):
        for j in range(i+1,m):
            dist[(str(idx[i]), str(idx[j]))] = adj[idx[i],idx[j]]

    return dist

"""### Callback Definition
Subtour constraints prevent multiple loops in a TSP tour. Because there are an exponential number of these constraints, we don't want to add them all to the model. Instead, we use a callback function to find violated subtour constraints and add them to the model as lazy constraints.
"""

# Callback - use lazy constraints to eliminate sub-tours

def subtourelim(model, where):
    if where == GRB.Callback.MIPSOL:
        # make a list of edges selected in the solution
        vals = model.cbGetSolution(model._vars)
        selected = gp.tuplelist((i, j) for i, j in model._vars.keys()
                             if vals[i, j] > 0.5)
        # find the shortest cycle in the selected edge list
        tour = subtour(selected)
        if len(tour) < len(nodes_idx):
            # add subtour elimination constr. for every pair of cities in subtour
            model.cbLazy(gp.quicksum(model._vars[i, j] for i, j in combinations(tour, 2))
                         <= len(tour)-1)

# Given a tuplelist of edges, find the shortest subtour

def subtour(edges):
    unvisited = nodes_idx[:]
    cycle = nodes_idx[:] # Dummy - guaranteed to be replaced
    while unvisited:  # true if list is non-empty
        thiscycle = []
        neighbors = unvisited
        while neighbors:
            current = neighbors[0]
            thiscycle.append(current)
            unvisited.remove(current)
            neighbors = [j for i, j in edges.select(current, '*')
                         if j in unvisited]
        if len(thiscycle) <= len(cycle):
            cycle = thiscycle # New shortest subtour
    return cycle


def solve_MILP():
    dist = create_dist(range(len(adj)))
    m = gp.Model()

    # Variables: is city 'i' adjacent to city 'j' on the tour?
    vars = m.addVars(dist.keys(), obj=dist, vtype=GRB.BINARY, name='x')

    # Symmetric direction: Copy the object
    for i, j in vars.keys():
        vars[j, i] = vars[i, j]  # edge in opposite direction

    # Constraints: two edges incident to each city
    cons = m.addConstrs(vars.sum(c, '*') == 2 for c in nodes_idx)

    m._vars = vars
    m.Params.lazyConstraints = 1
    m.optimize(subtourelim)

    vals = m.getAttr('x', vars)
    selected = gp.tuplelist((i, j) for i, j in vals.keys() if vals[i, j] > 0.5)

    tour = subtour(selected)
    print(tour)
    assert len(tour) == len(nodes_idx)+n-1
    return tour


if __name__ == '__main__':
    n = 2
    nodes, adj = createGraph(50, 7)

    tours = []
    nodes_idx = [str(j) for j in range(len(nodes))]
    tour = solve_MILP()

    for idx, node in enumerate(tour):
        if int(node) >= len(nodes):
            tour[idx] = 0

    # Plotting
    plt.plot(nodes[:, 0], nodes[:, 1], 'ko', zorder=100)
    plot_tour = []
    for c in tour:
        node = nodes[int(c)]
        plot_tour.append(node)
    plot_tour = np.array(plot_tour)
    plt.plot(plot_tour[:, 0], plot_tour[:, 1], 'r')
    plt.show()