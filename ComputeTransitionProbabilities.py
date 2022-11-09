import numpy as np
import scipy
from Constants import *

def ComputeTransitionProbabilities(stateSpace, map_world, K):
    """
    Computes the transition probabilities between all states in the state space for
    all control inputs.

    @type  stateSpace: (K x 4)-matrix
    @param stateSpace: Matrix where the i-th row represents the i-th
          element of the state space.

    @type  map_world: (M x N)-matrix
    @param  map_world:      A matrix describing the terrain.
          With values: FREE OBSTACLE PORTAL ALIEN MINE
          LAB BASE

    @type  K: integer
    @param K: An integer representing the total number of states in the state space

    @return P:
              A (K x K x L)-matrix containing the transition probabilities
              between all states in the state space for all control inputs.
              The entry P(i, j, l) represents the transition probability
              from state i to state j if control input l is applied.

    """

    P = []

    return P
