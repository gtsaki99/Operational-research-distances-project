#Georgios Tsakiridis - 9548
import sys
import math
import random
from itertools import permutations
import gurobipy as gp
from gurobipy import GRB

#Callback - use lazy constraints to eliminate sub-tours
def subtourElim(model, where):
    if where == GRB.Callback.MIPSOL:
        #Make a list of edges selected in the solution
        vals = model.cbGetSolution(model._x)
        selected = gp.tuplelist((i,j) for i, j, k in model._x.keys()
                                if vals[i, j, k] > 0.5)
        #Find the shortest cycle in the selected edge list
        tour = subtour(selected)
        if len(tour) < n:
            for k in trucks:
                model.cbLazy(gp.quicksum(model._x[i, j, k]
                                         for i, j in permutations(tour, 2))
                             <= len(tour)-1)

#Given a tuplelist of edges, find the shortest subtour not containing capital (0)
def subtour(edges):
    unvisited = list(range(1, n))
    cycle = range(n+1)  # initial length has 1 more city
    while unvisited:
        thiscycle = []
        neighbors = unvisited
        while neighbors:
            current = neighbors[0]
            thiscycle.append(current)
            if current != 0:
                unvisited.remove(current)
            neighbors = [j for i, j in edges.select(current, '*')
                         if j == 0 or j in unvisited]
        if 0 not in thiscycle and len(cycle) > len(thiscycle):
            cycle = thiscycle
    return cycle

#Number of cities, including the capital. The index of the capital is 0
n = 15
locations = [*range(n)]

#Number of trucks
K = 8
trucks = [*range(K)]

distances = [[0, 180, 240, 85, 285, 205, 235, 255, 155, 120, 230, 340, 220, 160, 240],  # Α
             [0, 0, 255, 150, 125, 100, 175, 235, 25, 65, 95, 210, 55, 135, 215],  # Β1
             [0, 0, 0, 160, 235, 160, 105, 45, 245, 255, 195, 225, 310, 115, 50],  # Β2
             [0, 0, 0, 0, 215, 125, 150, 170, 130, 110, 160, 260, 200, 75, 150],  # Β3
             [0, 0, 0, 0, 0, 90, 130, 195, 150, 190, 50, 85, 155, 155, 185],  # Β4
             [0, 0, 0, 0, 0, 0, 70, 135, 110, 135, 40, 135, 155, 65, 120],  # Β5
             [0, 0, 0, 0, 0, 0, 0, 70, 175, 200, 95, 125, 225, 70, 55],  # Β6
             [0, 0, 0, 0, 0, 0, 0, 0, 235, 250, 165, 180, 290, 110, 20],  # Β7
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 40, 110, 225, 70, 130, 215],  # Β8
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 145, 265, 100, 135, 230],  # Β9
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 115, 135, 105, 150],  # Β10
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 240, 185, 175],  # Β11
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 190, 270],  # Β12
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 90],  # Β13
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]  # B14
for i in range(n):
    for j in range(n):
        if j > i:
            distances[j][i] = distances[i][j]
time = {(i, j): distances[i][j] for i in range(n) for j in range(n) if i != j}
m = gp.Model('1st problem.lp')

#Creating variables:

#x =1, if truck  k  visits and goes directly from location  i  to location  j
x = m.addVars(time.keys(), trucks, vtype=GRB.BINARY, name='FromToBy')

#y = 1, if city i is visited by truck k
y = m.addVars(locations, trucks, vtype=GRB.BINARY, name='visitBy')

#Number of trucks used is a decision variable
z = m.addVars(trucks, vtype=GRB.BINARY, name='used')

#Distance driven per truck
t = m.addVars(trucks, ub=400, name='distance')

#Maximum distance
s = m.addVar(name='maxDistance')

#Truck utilization constraint
visitCity = m.addConstrs((y[i, k] <= z[k] for k in trucks for i in locations if i > 0), name='visitCity')

#Distance constraint
#Exclude the distance to return to the capital
distance = m.addConstrs((gp.quicksum(time[i, j] * x[i, j, k] for i, j in time.keys() if j > 0) == t[k] for k in trucks),
                        name='distanceConstr')
# Visit all cities constraint
visitAll = m.addConstrs((y.sum(i,'*') == 1 for i in locations if i > 0), name='visitAll' )
#Capital constraint
capitalConstr = m.addConstr(y.sum(0, '*') >= z.sum(), name='capitalConstr')
#Arriving at a city constraint
ArriveConstr = m.addConstrs((x.sum('*',j,k) == y[j,k] for j,k in y.keys()), name='ArriveConstr' )
#Leaving a city constraint
LeaveConstr = m.addConstrs((x.sum(j,'*',k) == y[j,k] for j,k in y.keys()), name='LeaveConstr' )
#Max distance constraint
maxDistance = m.addConstrs((t[k] <= s for k in trucks), name='maxDistanceConstr')
m.ModelSense = GRB.MINIMIZE
m.setObjectiveN(z.sum(), 0, priority=1, name="Number of trucks")
m.setObjectiveN(s, 1, priority=0, name="Distance")
#Breaking symmetry constraint
breakSymm = m.addConstrs((y.sum('*',k-1) >= y.sum('*',k) for k in trucks if k>0), name='breakSymm' )

#Verify model formulation
m.write('1st problem.lp')

#Run optimization engine
m._x = x
m.Params.LazyConstraints = 1
m.optimize(subtourElim)
# Print optimal routes
for k in trucks:
    route = gp.tuplelist((i,j) for i,j in time.keys() if x[i,j,k].X > 0.5)
    if route:
        i = 0
        print("Route for truck {" + str(k) + "}: {" + str(i) + "}", end='')
        while True:
            i = route.select(i, '*')[0][1]
            print(" -> {" + str(i) + "}", end='')
            if i == 0:
                break
        print(". Distance driven: {" + str(round(t[k].X,2)) + "} km.")


