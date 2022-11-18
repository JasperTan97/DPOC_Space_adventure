% main.m
%
% Matlab script that calls all the functions for computing the optimal cost
% and policy of the given problem.
%
% Dynamic Programming and Optimal Control
% Fall 2022
% Programming Exercise
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% --

%% Clear workspace and command window
clear all;
close all;
clc;

%% Options
% [M, N]
mapSize = [20, 10];
% Set to true to generate a random map of size mapSize, else set to false 
% to load the pre-exsisting example map
generateRandomWorld = true;
predefined_map = 'exampleWorld_3.mat';
sol_choice = "LP";

% Plotting options
global PLOT_POLICY PLOT_COST
PLOT_POLICY = true;
PLOT_COST = false;

%% Global problem parameters
% IMPORTANT: Do not add or remove any global parameter in main.m
global S N_a N_b P_DISTURBED P_PROTECTED
S = 0.5;           % Shielding factor against radiation in the lower dimension
N_a = 3;           % Fuel required to fight with an alien
N_b = 10;          % Fuel lost when crashing and returning to base
P_DISTURBED = 0.2; % Probability that the robot is disturbed due to radiation
P_PROTECTED = 0.6; % Probability that the robot successfully protects its gems against an alien

% IDs of elements in the map matrix
global FREE OBSTACLE PORTAL MINE LAB BASE ALIEN
FREE = 0;
OBSTACLE = 1;
PORTAL = 2;
MINE = 3;
LAB = 4;
BASE = 5;
ALIEN = 6;

% Index of each action in the P and G matrices. Use this ordering
global SOUTH NORTH EAST WEST STAY
SOUTH  = 1;
NORTH = 2;
WEST = 3;
EAST = 4;
STAY = 5;

% Index of whether the robot is carrying gems
global EMPTY GEMS
EMPTY = 0;
GEMS = 1;

% Index of dimensions
global UPPER LOWER
UPPER = 0;
LOWER = 1;

%% Generate map
% map(m,n) represents the cell type at indices (m,n) according to the axes
% specified in the PDF.
disp('Generate map');
if generateRandomWorld
	[map] = GenerateWorld(mapSize(1), mapSize(2));
else
    % We can load a pre-generated map.
    load(predefined_map);
end
MakePlots(map);

%% Generate state space
disp('Generate state space');
% Generate a (K x 4)-matrix 'stateSpace', where each accessible cell is
% represented by 4 rows.
stateSpace = [];
for m = 1 : size(map, 1)
    for n = 1 : size(map, 2)
        if map(m, n) ~= OBSTACLE
            stateSpace = [stateSpace;
                          m, n, EMPTY, UPPER;
                          m, n, GEMS, UPPER;
                          m, n, EMPTY, LOWER;
                          m, n, GEMS, LOWER];
        end
    end
end
% State space size
global K M N L
K=size(stateSpace,1);
M=size(map,1);
N=size(map,2);
L=length([SOUTH, NORTH, WEST, EAST, STAY]);

%% Set the following to true as you progress with the files
terminalStateIndexImplemented = true;
transitionProbabilitiesImplemented = true;
stageCostsImplemented = true;
SolutionImplemented = true;

%% Compute the terminal state index
global TERMINAL_STATE_INDEX
if terminalStateIndexImplemented
    % Question a)
    TERMINAL_STATE_INDEX = ComputeTerminalStateIndex(stateSpace, map);
end                  
%% Compute transition probabilities
P = zeros(K,K,L);
if transitionProbabilitiesImplemented
    disp('Compute transition probabilities');
    % Compute the transition probabilities between all states in the
    % state space for all control inputs.
    % The transition probability matrix has the dimension (K x K x L), i.e.
    % the entry P(i, j, l) representes the transition probability from state i
    % to state j if control input l is applied.
    
    % Question b)
    P = ComputeTransitionProbabilities(stateSpace, map);
%     f_CTP = @() ComputeTransitionProbabilities(stateSpace, map); % handle to function
%     timeit(f_CTP)
end

%% Compute stage costs
G = inf(K,L);
if stageCostsImplemented 
    disp('Compute stage costs');
    % Compute the stage costs for all states in the state space for all
    % control inputs.
    % The stage cost matrix has the dimension (K x L), i.e. the entry G(i, l)
    % represents the cost if we are in state i and apply control input l.
    
    % Question c)
    G = ComputeStageCosts(stateSpace, map, P);
%     f_CSC = @() ComputeStageCosts(stateSpace, map, P);
%     timeit(f_CSC)
end

%% Solve stochastic shortest path problem
% Solve the stochastic shortest path problem
if SolutionImplemented
    disp('Solve stochastic shortest path problem');
    
    % Question d)
    if sol_choice == "LP"
        [ J_opt, u_opt_ind ] = LP_sol(P, G);
    elseif sol_choice == "VI"
        [ J_opt_VI, u_opt_ind_VI ] = VI_sol(P, G);
    else
        disp("Invalid choice")
    end

    % Below code measures times
%     [ J_opt_LP, u_opt_ind_LP ] = LP_sol(P, G);
%     [ J_opt_VI, u_opt_ind_VI ] = VI_sol(P, G);
% 
%       f_LP = @() LP_sol(P,G);
%       f_VI = @() VI_sol(P,G);
%       disp("LP");
%       timeit(f_LP)
%       disp("VI")
%       timeit(f_VI)
% 
%     list_of_diff = find( abs(u_opt_ind_VI - u_opt_ind_LP) > 1e-10 );
%     fprintf("Number of errors for between VI and LP: %d\n", size(list_of_diff,1) )
% 
%     for err_idx = 1: size(list_of_diff)
%         fprintf("State: \n")
%         stateSpace(list_of_diff(err_idx),:)
%         fprintf("VI chose: %d \n", u_opt_ind_VI(list_of_diff(err_idx)))
%         fprintf("LP chose: %d \n", u_opt_ind_LP(list_of_diff(err_idx)))
%     end

    
    if size(J_opt,1)~=K || size(u_opt_ind,1)~=K
        disp('[ERROR] the size of J and u must be K')
    else
        % Plot results
        disp('Plot results');
        MakePlots(map, stateSpace, J_opt, u_opt_ind, 'Solution');
    end
end

%% Terminated
disp('Terminated');
