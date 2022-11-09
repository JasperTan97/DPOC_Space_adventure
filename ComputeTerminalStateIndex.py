import numpy as np
import scipy
from Constants import *

def ComputeTerminalStateIndex(stateSpace, map_world):
    """
    Computes the index of the terminal state in the stateSpace matrix

    @type  stateSpace: (K x 4)-matrix
    @param stateSpace: Matrix where the i-th row represents the i-th
          element of the state space.

    @type  map_world: (M x N)-matrix
    @param  map_world:      A matrix describing the terrain.
          With values: FREE OBSTACLE PORTAL ALIEN MINE
          LAB BASE

    @return stateIndex: An integer that is the index of the terminal state in the
              stateSpace matrix

    """
    stateIndex = None

    return stateIndex
