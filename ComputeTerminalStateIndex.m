function stateIndex = ComputeTerminalStateIndex(stateSpace, map)
%ComputeTerminalStateIndex Compute the index of the terminal state
%
%   stateIndex = ComputeTerminalStateIndex(stateSpace, map) 
%   Computes the index of the terminal state in the stateSpace matrix
%   
%   Input arguments:
%       
%       stateSpace:
%           A (K x 4)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain. 
%           With values: FREE OBSTACLE PORTAL ALIEN MINE 
%           LAB BASE
%
%   Output arguments:
%
%       stateIndex:
%           An integer that is the index of the terminal state in the
%           stateSpace matrix

    global S N_a N_b P_DISTURBED P_PROTECTED
    global FREE OBSTACLE PORTAL MINE LAB BASE ALIEN
    global SOUTH NORTH EAST WEST STAY
    global EMPTY GEMS
    global UPPER LOWER
    global K M N L

end
