function P = ComputeTransitionProbabilities(stateSpace, map)
%ComputeTransitionProbabilities Compute transition probabilities.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) computes 
%   the transition probabilities between all states in the state space for 
%   all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 4)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain. 
%           With values: FREE OBSTACLE PORTAL ALIEN MINE LAB BASE
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

    global S N_a N_b P_DISTURBED P_PROTECTED
    global FREE OBSTACLE PORTAL MINE LAB BASE ALIEN
    global SOUTH NORTH EAST WEST STAY
    global EMPTY GEMS
    global UPPER LOWER
    global K M N L
    global TERMINAL_STATE_INDEX

end
