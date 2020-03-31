from pulp import *
import coinor.dippy as dippy

from math import floor, ceil
import matplotlib.pyplot as plt
from veh_rout_prob import FIGSIZE, get_graphs, get_subtour

tol = pow(pow(2, -20), 2.0 / 3.0)
myopts = {
    "Tol": tol,
    "Interval": 1000,
}


def formulate(vrp, options={}):
    prob = dippy.DipProblem("VRP",
                            display_mode='matplotlib',
                            display_interval=10)

    assign_vars = LpVariable.dicts("y",
                                   [(i, j, k) for i in vrp.EXTLOCS
                                    for j in vrp.EXTLOCS
                                    for k in vrp.VEHS
                                    if i != j],
                                   cat=LpBinary)
    use_vars = LpVariable.dicts("x", vrp.VEHS, cat=LpBinary)

    # Objective function
    prob += lpSum(vrp.dist[i, j] * assign_vars[i, j, k]
                  for i in vrp.EXTLOCS
                  for j in vrp.EXTLOCS
                  for k in vrp.VEHS
                  if i != j), "min_dist"

    # Each node (excluding 'O') must have one arc entering from any other node (including 'O')
    for j in vrp.LOCS:
        prob += lpSum(assign_vars[i, j, k]
                      for i in vrp.EXTLOCS
                      for k in vrp.VEHS
                      if i != j) == 1

    # Each node (excluding 'O') must have one arc leaving to any other node (including 'O')
    for i in vrp.LOCS:
        prob += lpSum(assign_vars[i, j, k]
                      for j in vrp.EXTLOCS
                      for k in vrp.VEHS
                      if j != i) == 1

    for k in vrp.VEHS:
        # Conservation of flows
        for j in vrp.LOCS:
            prob += lpSum(assign_vars[i_1, j, k]
                          for i_1 in vrp.EXTLOCS
                          if i_1 != j) == lpSum(assign_vars[j, i_2, k]
                                                for i_2 in vrp.EXTLOCS
                                                if i_2 != j)

        if vrp.allused:
            # Specify that all vehicles must enter the depot
            prob += lpSum(assign_vars[i, 'O', k]
                          for i in vrp.LOCS) == 1

            # Specify all vehicles must leave the depot
            prob += lpSum(assign_vars['O', j, k]
                          for j in vrp.LOCS) == 1
        else:
            # Specify that if a vehicle is used it must enter the depot
            prob += lpSum(assign_vars[i, 'O', k]
                          for i in vrp.LOCS) == use_vars[k]

            # Specify that if a vehicle is used it must leave the depot
            prob += lpSum(assign_vars['O', j, k]
                          for j in vrp.LOCS) == use_vars[k]

        if vrp.distcap is not None:
            # For each vehicle k, ensure that the maximum distance travelled is less than the distance
            # capacity and 0 if that vehicle is not used.
            prob += lpSum(vrp.dist[i, j] * assign_vars[i, j, k]
                          for i in vrp.EXTLOCS
                          for j in vrp.EXTLOCS
                          if i != j) <= vrp.distcap * use_vars[k]
        else:
            # Cardinality of arcs for vehicles in use
            prob += lpSum(assign_vars[i, j, k]
                          for i in vrp.EXTLOCS
                          for j in vrp.EXTLOCS
                          if i != j) <= len(vrp.EXTLOCS) * use_vars[k]

    # Attach the problem data and variable dictionaries to the DipProblem
    prob.vrp = vrp
    prob.assign_vars = assign_vars
    prob.use_vars = use_vars

    if "Tol" in options:
        prob.tol = options["Tol"]
    else:
        prob.tol = pow(pow(2, -24), 2.0 / 3.0)

    return prob


# Solve the TSP
def solve(prob, options={}):

    # Set the options
    prob.options = options

    # When checking feasibility, use a callback
    # to check for subtours
    prob.is_solution_feasible = is_solution_feasible
    # When generating cuts, use a callback
    # to generate subtour elimination constraints
    prob.generate_cuts = generate_cuts

    dippyOpts = {
        #               'CutCGL': 1, # <----- Cuts turned on
        'CutCGL': 0,  # <----- Cuts turned off
        #               'LogDumpModel': 5,
        #               'LogDebugLevel': 5,
    }
    # Can use Cut Generator Library (CGL) cuts too
    if ("Cuts" in options) and (options["Cuts"] == "CGL"):
        dippyOpts['CutCGL'] = 1
    if "Interval" in options:
        prob.display_interval = options["Interval"]

    plt.figure(figsize=FIGSIZE)
    status, message, primals, duals = dippy.Solve(prob, dippyOpts)

    if status == LpStatusOptimal:
        return dict((var, var.value()) for var in prob.variables())
    else:
        return None


def solve_and_display(prob, options={}):
    xopt = solve(prob, options)

    # Reads and displays the solution if one is found
    if xopt is not None:
        for var in prob.variables():
            if abs(xopt[var]) > options["Tol"]:
                print(var.name, "=", xopt[var])
    else:
        print("Dippy could not find and optimal solution")

    # Draw the final B-&-B tree if it is being displayed
    if prob.display_mode != 'off':
        tree_nodes = prob.Tree.get_node_list()
        numNodes = len(tree_nodes)
        print("Number of nodes =", numNodes)
        print("Tree depth =", max(prob.Tree.get_node(n).attr['level'] for n in tree_nodes) - 1)
        if prob.display_mode in ['pygame', 'xdot', 'matplotlib']:
            prob.Tree.display(pause=True, wait_for_click=False)

    return xopt


# User callback for generating cuts
def generate_cuts(prob, sol):

    # No constraints added
    cons = []
    cons_added = 0

    # Get the assignment variables and values
    assign_vars = prob.assign_vars
    assign_vals = dict([((i, j, k), sol[assign_vars[i, j, k]]) for (i, j, k) in assign_vars.keys()])

    # Get the threshold for whether an arc should be considered
    # as part of the solution (almost = 1 by default)
    if "Tours" in prob.options:
        threshold = prob.options["Tours"]
    else:
        threshold = 1.0 - prob.tol  # Default is only consider integer arcs

    # Get the graphs for each vehicle
    nodes = prob.vrp.EXTLOCS[:]
    arcs = [(i, j, k) for (i, j, k) in assign_vars.keys() if sol[assign_vars[i, j, k]] > threshold]

    # Loop over the vehicles that are used
    for k in prob.vrp.VEHS:

        # Extract the arcs for each vehicle
        vehArcs = [x[:2] for x in arcs if x[2] == k]

        # Define the set of nodes that have not been put in a connected component
        not_connected = set(nodes[:])

        # While any nodes are not in a connected component
        while not_connected:

            # Get an unconnected node
            start = not_connected.pop()

            # Find a subtour from that node
            tNodes, tArcs = get_subtour(nodes, vehArcs, start)

            # If it is a subtour (and not a complete tour), add a subtour elimination constraint
            if len(tNodes) == len(tArcs) and len(tNodes) < len(nodes):
                cons_added += 1

                #   If a subtour is found then that
                #   graph must be banned

                # Option 1
                # cons.append(lpSum(assign_vars[i, j, k]
                #                   for (i, j) in tArcs) <= len(tArcs) - 1)

                # Option 2
                cons.append(lpSum(assign_vars[i, j, k]
                                  for i in tNodes
                                  for j in set(nodes).difference(tNodes)) +
                            lpSum(assign_vars[j, i, k]
                                  for i in tNodes
                                  for j in set(nodes).difference(tNodes))
                            >= 2)

                print("Subtour elimination!", cons[-1])

                # Return one subtour elimination constraint at a time
                if cons_added == 1:
                    return cons

            # Remove the subtour nodes as they are now connected
            not_connected -= set(tNodes)

    if len(cons) > 0:
        return cons
    else:
        return None


# User callback for checking feasibility
def is_solution_feasible(prob, sol, tol):

    # # # # # # # # # # # Display the current node solution
    assignments = get_assignments(prob, sol, tol)
    prob.vrp.setSolution(assignments, tol)
    prob.vrp.displaySolution(title="Feasibility Check", showProb=None)
    # # # # # # # # # # #

    # Get the threshold for whether an arc should be considered
    # as part of the solution (almost = 1 by default)
    if "Tours" in prob.options:
        threshold = prob.options["Tours"]
    else:
        threshold = 1.0 - prob.tol  # Default is only consider integer arcs

    # Get the assignment variables
    assign_vars = prob.assign_vars
    assign_vals = dict([((i, j, k), sol[assign_vars[i, j, k]])
                       for (i, j, k) in assign_vars.keys()])

    # Get the graphs for each vehicle
    nodes = prob.vrp.EXTLOCS[:]
    arcs = [(i, j, k) for (i, j, k) in assign_vars.keys() if sol[assign_vars[i, j, k]] > threshold]

    # Loop over the vehicles that are used
    for k in prob.vrp.VEHS:

        # Extract the arcs for each vehicle to pass into get_subtour()
        vehArcs = [x[:2] for x in arcs if x[2] == k]

        # Look for subtour in each vehicles graph from the first node
        tNodes, tArcs = get_subtour(nodes, vehArcs, nodes[1])

        #   If a subtour is found then the solution is not feasible, so will declare it as such
        if (len(tNodes) == len(tArcs)) and (len(tNodes) < len(nodes)):
            print("Solution has subtours!")
            return False

    # Otherwise it is feasible
    print("Solution has no subtours!")
    return True


def get_assignments(prob, sol, tol):
    assignments = {}
    for tup, var in prob.assign_vars.items():
        if sol[var] is not None:
            if sol[var] > tol:
                assignments[tup] = sol[var]

    return assignments
