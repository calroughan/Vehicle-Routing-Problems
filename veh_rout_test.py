"""This is a test module which runs your ???????_veh_rout_func.py code
and reports back the result. There are 10 problems in total.

In order to verify whether or not your code works, place this file,
veh_rout_test.py inside your directory so that it neighbours and can
import from ???????_veh_rout_func.py locally. Then run the code via
conda. If you obtain no errors, this means that for all ten problems,
your code worked.

If you wish to test your own problems as well, you can use
vehicle_router(...) to obtain the result. Use display=True if you wish
to see the problem's solution. If you are happy that your solution is
in fact the optimal solution, then you can use check_vehicle_router(...)
to test the results. Read the function documentation for more
information.

Author: Elliot Paton-Simpson
"""

# Import builtins.
from random import random, seed
from typing import Dict, List, Optional, Tuple, Union

# Import additional package.
import coinor.dippy as dippy
from pulp import LpVariable

# Import locally.
from veh_rout_prob import VRProb
from epat261_veh_rout_func import (
    formulate, get_assignments, myopts, solve, solve_and_display
)  # TODO: replace epat261 with UPI


class DipProblemExtended(dippy.DipProblem):
    """Extension of DipProblemExtended for resolving problems with
    introducing parameters outside of DipProblem.__init__.
    """
    assign_vars: Dict[Tuple[Union[int, str], Union[int, str], int], LpVariable]
    use_vars: Dict[int, LpVariable]


def vehicle_router(
        num_locations: int,
        num_vehicles: int = 1,
        max_dist: Optional[float] = None,
        use_all_vehicles: bool = False,
        seed_n: int = 0,
        display: bool = False
) -> Optional[Dict[int, List[Union[str, int]]]]:
    """
    Tests the vehicle routing problem with a number of parameters and
    returns the variables.
    :param int num_locations: The number of locations besides the depot.
    :param int num_vehicles: The number of vehicles available for
        travel.
    :param Optional[float] max_dist: The maximum distance a vehicle can
        travel.
    :param bool use_all_vehicles: Whether or not every vehicle must
        leave the depot.
    :param int seed_n: The random seed number. Affects the coordinate
        generation.
    :param bool display: Whether to display the solution to the problem.
    :rtype: Optional[Tuple[List[
            Tuple[Union[str, int], Union[int, str], int]
        ], List[int]]]
    :rtype: Optional[Dict[int, List[Union[str, int]]]]
    :return: If the problem is not solved, returns None. Otherwise,
        returns a dictionary with vehicle numbers for keys and lists
        of arcs for values, where each lists contains
    """
    # Gets the tolerance for the problem.
    tol = myopts['Tol']

    # Generates each of the different locations.
    locations = list(range(1, num_locations + 1))

    # Sets the seed before performing random generation.
    seed(seed_n)

    # Generates x, y coordinates for each of the different locations.
    x = {i: random() * 10 for i in locations}
    y = {i: random() * 10 for i in locations}

    # Centers the depot.
    x['O'] = 5
    y['O'] = 5

    # Initializes and formulates the linear program.
    vrp = VRProb(
        LOCS=locations, ncurr=num_vehicles, x=x, y=y, maxdist=max_dist,
        useall=use_all_vehicles
    )
    prob: dippy.DipProblem = formulate(vrp, options=myopts)

    # Solve the problem and display the result.
    if display:
        solution = solve_and_display(prob, options=myopts)
        if solution:
            vrp.setSolution(get_assignments(prob, solution, tol), tol)
            vrp.displaySolution(title="Solution")

    # Solve the problem without rendering the result.
    else:
        solution = solve(prob, options=myopts)

    # Returns None if no solution was found
    if solution is None:
        return None

    # Create lists for the solution.
    prob: DipProblemExtended
    assign_vals = [
        (i, j, k) for i, j, k in prob.assign_vars
        if solution[prob.assign_vars[i, j, k]] > 1 - tol
    ]
    use_vals = [k for k in prob.use_vars if bool(solution[prob.use_vars[k]])]

    # Return the results.
    return {
        veh: sorted(list(map(
            lambda p: p[:2], filter(lambda p: p[2] == veh, assign_vals)
        )), key=lambda p: f"{p[0]}{p[1]}") for veh in use_vals
    }


def check_vehicle_router(
        arcs: List[List[Tuple[Union[str, int], Union[str, int]]]],
        num_locations: int,
        num_vehicles: int = 1,
        max_dist: Optional[float] = None,
        use_all_vehicles: bool = False,
        seed_n: int = 0,
        display: bool = False
) -> None:
    """
    Checks the output from the vehicle_router function.
    :param List[List[Tuple[Union[int, str], Union[int, str]]]] arcs: A
        list of each vehicle's route, where each vehicle has a list
        containing each arc.
    :param int num_locations: The number of locations besides the depot.
    :param int num_vehicles: The number of vehicles available for
        travel.
    :param Optional[float] max_dist: The maximum distance a vehicle can
        travel.
    :param bool use_all_vehicles: Whether or not every vehicle must
        leave the depot.
    :param int seed_n: The random seed number. Affects the coordinate
        generation.
    :param bool display: Whether to display the solution to the problem.
    :return: None
    """
    # Obtains the result for the linear program.
    result = vehicle_router(
        num_locations, num_vehicles, max_dist, use_all_vehicles, seed_n,
        display
    )

    # Special case if the problem is not feasible.
    if not result:
        if not arcs:
            return
        else:
            raise ValueError("Problem infeasible. Expected a route.")

    # Sorts the arcs, both those expected and those obtained by the
    # test.
    routes = [
        sorted(x, key=lambda y: f"{y[0]}{y[1]}") for x in result.values()
    ]
    arcs = [sorted(a, key=lambda x: f"{x[0]}{x[1]}") for a in arcs]

    # Checks that the same number of vehicles were used.
    if len(routes) != len(arcs):
        raise ValueError(
            f"Expected {len(arcs)} vehicles to be used. Got {len(routes)}."
        )

    # Checks each of the different vehicles.
    for route in routes:
        # Checks that the route the vehicle takes was expected,
        # allowing it to travel the opposite direction.
        for arc_single_veh in arcs:
            if route == arc_single_veh or (
                sorted((r[::-1] for r in route), key=lambda y: f"{y[0]}{y[1]}")
                == arc_single_veh
            ):
                break

        # If no such route was expected, raises an error.
        else:
            raise ValueError(f"Unexpected tour: {route}.")


if __name__ == '__main__':
    # Test 1
    check_vehicle_router(
        arcs=[[(2, 1), (5, 2), (1, 3), (3, 4), ('O', 5), (4, 'O')]],
        num_locations=5
    )

    # Test 2
    check_vehicle_router(
        arcs=[[
            (1, 2), (2, 6), (3, 'O'), (4, 8), (5, 4), (6, 3), (7, 1),
            (8, 9), (9, 10), (10, 7), ('O', 5)
        ]],
        num_locations=10
    )

    # Test 3
    check_vehicle_router(
        arcs=[
            [(3, 6), (6, 'O'), ('O', 3)],
            [
                (10, 9), (1, 7), (2, 1), (4, 5), (5, 'O'), (7, 10), (8, 4),
                (9, 8), ('O', 2)
            ]
        ],
        num_vehicles=2,
        num_locations=10,
        use_all_vehicles=True
    )

    # Test 4
    check_vehicle_router(
        arcs=[[
            (1, 4), (2, 6), (3, 8), (4, 'O'), (5, 3), (6, 1), (7, 2), (8, 7),
            ('O', 5)
        ]],
        num_locations=8,
        num_vehicles=3,
        seed_n=1
    )

    # Test 5
    check_vehicle_router(
        arcs=[],
        num_locations=8,
        num_vehicles=2,
        max_dist=10
    )

    # Test 6
    check_vehicle_router(
        arcs=[
            [(1, 2), (2, 7), (3, 6), (6, 'O'), (7, 3), ('O', 1)],
            [(4, 'O'), (5, 8), (8, 4), ('O', 5)],
        ],
        num_locations=8,
        num_vehicles=2,
        max_dist=20
    )

    # Test 7
    check_vehicle_router(
        arcs=[
            [
                ('O', 2), (2, 11), (11, 1), (1, 7), (7, 10), (10, 5), (5, 4),
                (4, 6), (6, 9), (9, 'O')
            ],
            [('O', 3), (3, 13), (13, 8), (8, 'O')],
            [('O', 12), (12, 'O')]
        ],
        num_locations=13,
        num_vehicles=3,
        use_all_vehicles=True,
        max_dist=25
    )

    # Test 8 – this one might take a while.
    check_vehicle_router(
        arcs=[
            [(12, 3), (13, 8), (3, 13), (8, 'O'), ('O', 12)],
            [
                (10, 5), (11, 1), (1, 7), (2, 11), (4, 6), (5, 4), (6, 9),
                (7, 10), (9, 'O'), ('O', 2)
            ],
        ],
        num_locations=13,
        num_vehicles=3,
        max_dist=25
    )

    # Test 9
    check_vehicle_router(
        arcs=[
            [(1, 5), (2, 1), (4, 'O'), (5, 4), ('O', 2)],
            [(3, 'O'), ('O', 3)],
            [(6, 'O'), ('O', 6)]
        ],
        num_locations=6,
        num_vehicles=3,
        use_all_vehicles=True
    )

    # Test 10
    check_vehicle_router(
        arcs=[
            [(1, 6), (2, 4), (3, 5), (4, 3), (5, 'O'), (6, 2), ('O', 1)]
        ],
        num_locations=6,
        num_vehicles=3,
        seed_n=5
    )
