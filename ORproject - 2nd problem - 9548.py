#Georgios Tsakiridis - 9548 + https://www.gurobi.com/documentation/9.1/examples/tsp_py.html
from itertools import combinations
import gurobipy as gp
from gurobipy import GRB

#Callback - using lazy constraints to eliminate sub-tours
def subtourElim(model, where):
    if where == GRB.Callback.MIPSOL:
        #Making a list of edges selected in the solution
        vals = model.cbGetSolution(model._vars)
        selected = gp.tuplelist((i, j) for i, j in model._vars.keys()
                                if vals[i, j] > 0.5)
        #Finding the shortest cycle in the selected edge list
        tour = subtour(selected)
        if len(tour) < n:
            #Adding subtour elimination constraint for every pair of cities in tour
            model.cbLazy(gp.quicksum(model._vars[i, j]
                                     for i, j in combinations(tour, 2))
                         <= len(tour)-1)
#Given a tuplelist of edges, finding the shortest subtour
def subtour(edges):
    unvisited = list(range(n))
    cycle = range(n+1)  #Initial length has 1 more city
    while unvisited:  #True if list is non-empty
        thiscycle = []
        neighbors = unvisited
        while neighbors:
            current = neighbors[0]
            thiscycle.append(current)
            unvisited.remove(current)
            neighbors = [j for i, j in edges.select(current, '*')
                         if j in unvisited]
        if len(cycle) > len(thiscycle):
            cycle = thiscycle
    return cycle
n = 7
times = [ [0,789, 549, 657, 331, 559, 250],
               [0,0,668,979,593,224,905],
               [0,0,0,316,607,472,467],
               [0,0,0,0,890,769,400],
               [0,0,0,0,0,386,559],
               [0,0,0,0,0,0,681],
               [0,0,0,0,0,0,0]]
tms = {(i, j): times[i][j] for i in range(n) for j in range(i + 1, n)}
m = gp.Model()

#Creating the variables
vars = m.addVars(tms.keys(), obj=tms, vtype=GRB.BINARY, name='2nd problem')
for i, j in vars.keys():
    vars[j, i] = vars[i, j]  # edge in opposite direction

#Entering and leaving a city constraint
m.addConstrs(vars.sum(i, '*') == 2 for i in range(n))

# Optimizing the model
m._vars = vars
m.Params.lazyConstraints = 1
m.optimize(subtourElim)

#Extracting the solution and printing it
vals = m.getAttr('x', vars)
selected = gp.tuplelist((i, j) for i, j in vals.keys() if vals[i, j] > 0.5)
tour = subtour(selected)
assert len(tour) == n
print('')
print('Optimal tour: %s' % str(tour))