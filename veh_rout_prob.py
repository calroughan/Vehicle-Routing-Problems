from math import sqrt

import networkx as nx
import matplotlib.pyplot as plt

FIGSIZE    = (3, 1.5)
FIGSTRETCH = 1.5
NODESIZE = 100 # Default = 300
FONTSIZE = 8   # Default = 12

class VRProb:
  def __init__(self, LOCS, ncurr, x=None, y=None, dist=None, maxdist=None, useall=False):
    self.LOCS = LOCS
    self.EXTLOCS = LOCS[:]
    self.EXTLOCS.append('O')
    self.VEHS = range(1, ncurr + 1)
    self.x = x
    self.y = y
    if (x is None) and (y is None) and (dist is None):
      raise Exception("No coordinates or distance matrix in VRPProb!")
    elif (dist is None):
      dist = {}  
      for i in x.keys():
        for j in x.keys():
          dist[i, j] = sqrt((x[i] - x[j])**2 + (y[i] - y[j])**2)      
    self.dist = dist
    self.fixed = ncurr
    self.allused = useall
    self.distcap = maxdist
    
  def drawProblem(self):
    if (self.x is None) and (self.y is None):
      print("No (x, y)-coordinates so can't draw VRPProb!")
    else:
      G = nx.DiGraph()
      G.add_nodes_from(self.EXTLOCS)
      pos = dict([(i, (self.x[i], self.y[i])) for i in self.EXTLOCS])
      fig = plt.gcf()
      fig.set_size_inches(FIGSIZE)
      nx.draw(G, pos, with_labels=True, node_size=NODESIZE, font_size=FONTSIZE)
      xmin = min(self.x.values())
      xmax = max(self.x.values())
      ymin = min(self.y.values())
      ymax = max(self.y.values())
      xscale = FIGSTRETCH * (xmax - xmin)
      yscale = FIGSTRETCH * (ymax - ymin)
      xmid = (xmin + xmax) / 2.0
      ymid = (ymin + ymax) / 2.0
      plt.xlim(xmid - xscale / 2.0, xmid + xscale / 2.0)
      plt.ylim(ymid - yscale / 2.0, ymid + yscale / 2.0)
      plt.show()
              
  def setSolution(self, assignments, tol):
    self.assignment = {}
    for k in self.VEHS:
      self.assignment[k] = []
      for (i, j, khat) in assignments:
        if (khat == k) and (assignments[i, j, k] > tol):
          self.assignment[k].append((i, j))
  
  def displaySolution(self, title=None, showProb=True):
    colors = ['b', 'r', 'g', 'm', 'c', 'k', 'y']
    # print solution
    G = nx.DiGraph()
    G.add_nodes_from(self.EXTLOCS)
    pos = dict([(i, (self.x[i], self.y[i])) for i in self.EXTLOCS])
    fig = plt.gcf()
    fig.set_size_inches(FIGSIZE[0], 2*FIGSIZE[1])
    xmin = min(self.x.values())
    xmax = max(self.x.values())
    ymin = min(self.y.values())
    ymax = max(self.y.values())
    xscale = FIGSTRETCH * (xmax - xmin)
    yscale = FIGSTRETCH * (ymax - ymin)
    xmid = (xmin + xmax) / 2.0
    ymid = (ymin + ymax) / 2.0
    if showProb:
      ax1 = plt.subplot(211)
      nx.draw_networkx_nodes(G, pos, node_size=NODESIZE)
      nx.draw_networkx_labels(G, pos, font_size=FONTSIZE)
      ax1.set_xlim(xmid - xscale / 2.0, xmid + xscale / 2.0)
      ax1.set_ylim(ymid - yscale / 2.0, ymid + yscale / 2.0)
      ax1.axis('off')
      ax2 = plt.subplot(212)
    else:
      ax2 = plt.gca()
    for i, k in enumerate(self.VEHS):
      print("Vehicle =", k)
      total = 0
      tour = []
      for arc in self.assignment[k]:
        print(" ", arc[0], arc[1], self.dist[arc[0], arc[1]])
        G.add_edge(arc[0], arc[1])
        tour.append((arc[0], arc[1]))
        total += self.dist[arc[0], arc[1]]
      print("  Total =", total)
      nx.draw_networkx_edges(G, pos, edgelist=tour, edge_color=colors[i])
    nx.draw_networkx_nodes(G, pos, node_size=NODESIZE)
    nx.draw_networkx_labels(G, pos, font_size=FONTSIZE)
    ax2.set_xlim(xmid - xscale / 2.0, xmid + xscale / 2.0)
    ax2.set_ylim(ymid - yscale / 2.0, ymid + yscale / 2.0)
    ax2.axis('off')
    if title:
      plt.title(title)
    plt.show()

def get_graphs(vrp, assignments, tol):
  vehNodes = {}
  vehArcs  = {}
  for k in vrp.VEHS:

    vehArcs[k] = list(set([(i, j) for (i, j, khat) in assignments
                           if (k == khat) and (assignments[i, j, khat] > tol)]))

    vehNodes[k] = list(set([i for (i, j) in vehArcs[k]]).union(set([j for (i, j) in vehArcs[k]])))
        
  return vehNodes, vehArcs

def get_subtour(vehNodes, vehArcs, node):
    # returns: list of nodes and arcs
    # in subtour containing node
    nodes = set([node])
    arcs = set([])
    not_processed = set(vehNodes)
    to_process = set([node])

    while to_process:
#        print(to_process)
#        print(not_processed)
        c = to_process.pop()
        not_processed.remove(c)
        new_arcs = [ (c, i)
                     for i in not_processed
                     if (c, i) in vehArcs]                     
        new_arcs.extend([ (i, c)
                         for i in not_processed
                         if (i, c) in vehArcs])      
        new_nodes = [ i for i in not_processed \
                      if (c, i) in new_arcs ]
        new_nodes.extend([ i for i in not_processed \
                          if (i, c) in new_arcs ])
#        print(new_arcs)
#        print(new_nodes)
        arcs |= set(new_arcs)
        nodes |= set(new_nodes)
        to_process |= set(new_nodes)

    nodes = list(nodes)
    arcs = list(arcs)
    
    return nodes, arcs
