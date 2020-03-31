from random import random, seed

seed(1)
#seed(5)

from veh_rout_prob import VRProb
from crou060_veh_rout_func import myopts, formulate, solve_and_display, get_assignments

if __name__ == '__main__':
  # Python starts here
  numLocs = 5
  locs = list(range(1, numLocs + 1))
  itmBnds  = [0, 4]

  x = dict([(i, round(random() * (itmBnds[1] - itmBnds[0]) + itmBnds[0], 2)) for i in locs])
  y = dict([(i, round(random() * (itmBnds[1] - itmBnds[0]) + itmBnds[0], 2)) for i in locs])
  x['O'] = 2
  y['O'] = 2  
  
  ncurr = 5
  
  vrp = VRProb(LOCS    = locs,
               ncurr   = ncurr,
               x       = x,
               y       = y,
               maxdist = 6)
  
  vrp.drawProblem()
  
  tol = pow(pow(2, -20), 2.0 / 3.0)
  opts = {
      "Tol": tol,
      "Interval": 1000,
      }
  
  prob = formulate(vrp, options=opts)
  
  xopt = solve_and_display(prob, options=opts)

  assignments = get_assignments(prob, xopt, opts["Tol"])

  vrp.setSolution(assignments, prob.tol)

  vrp.displaySolution(title="Solution")
  vrp.displaySolution(title="Solution", showProb=False)
