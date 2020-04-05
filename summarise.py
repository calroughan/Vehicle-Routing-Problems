from random import random, seed

from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.pyplot as plt

from veh_rout_prob_Dav import VRProb
from crou060_veh_rout_func import myopts, formulate, solve_and_display, get_assignments


class Problem(object):

    def __init__(self, n_veh, distcap=None, useall=None):
        self.n_veh = n_veh
        self.distcap = distcap
        self.useall = useall

    def run_problem(self, seeds, nrow, ncol, name):
        fig, axs = plt.subplots(nrow, ncol)
        for (ax, s) in zip(axs.flatten(), seeds):
            print(s)
            seed(s)
            numLocs = 5
            locs = list(range(1, numLocs + 1))
            itmBnds = [0, 4]

            x = dict([(i, round(random() * (itmBnds[1] - itmBnds[0]) + itmBnds[0], 2)) for i in locs])
            y = dict([(i, round(random() * (itmBnds[1] - itmBnds[0]) + itmBnds[0], 2)) for i in locs])
            x['O'] = 2
            y['O'] = 2

            vrp = VRProb(LOCS=locs, ncurr=self.n_veh, x=x, y=y, maxdist=self.distcap, useall=self.useall)

            tol = pow(pow(2, -20), 2.0 / 3.0)
            opts = {"Tol": tol, "Interval": 1000, }

            prob = formulate(vrp, options=opts)
            xopt = solve_and_display(prob, options=opts)

            assignments = get_assignments(prob, xopt, opts["Tol"])
            vrp.setSolution(assignments, prob.tol)
            vrp.displaySolution(ax, name, s, title="Solution", showProb=False)

        return fig

def run():

    prob1 = Problem(5)
    fig1 = prob1.run_problem([1, 5, 23, 42, 1721, 6174], 3, 2, 'One')

    prob2 = Problem(5, distcap=6)
    fig2 = prob2.run_problem([1, 5, 23, 42, 1721, 6174], 3, 2, 'Two')

    prob3 = Problem(3, useall=True)
    fig3 = prob3.run_problem([1, 5, 23, 42, 1721, 6174], 3, 2, 'Three')

    with PdfPages('outputA.pdf') as pdf:
        pdf.savefig(fig1)
        plt.close()
        pdf.savefig(fig2)
        plt.close()
        pdf.savefig(fig3)

    prob1 = Problem(5)
    fig1 = prob1.run_problem([1006, 1007, 1008, 1010, 1012, 1013], 3, 2, 'One')

    prob2 = Problem(5, distcap=6)
    fig2 = prob2.run_problem([1006, 1007, 1008, 1010, 1012, 1013], 3, 2, 'Two')

    prob3 = Problem(3, useall=True)
    fig3 = prob3.run_problem([1006, 1007, 1008, 1010, 1012, 1013], 3, 2, 'Three')

    with PdfPages('outputB.pdf') as pdf:
        pdf.savefig(fig1)
        plt.close()
        pdf.savefig(fig2)
        plt.close()
        pdf.savefig(fig3)
        plt.close()

if __name__ == '__main__':
    run()
