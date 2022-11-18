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

    global GEMS UPPER LAB

    [x,y] = find(map==LAB);
    stateIndex = find(ismember(stateSpace,[x, y, GEMS, UPPER], "rows"));

end
