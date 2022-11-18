function [ J_opt, u_opt_ind ] = Solution(P, G)
%SOLUTION 
%   Solve a stochastic shortest path problem by either Value Iteration,
%   Policy Iteration, or Linear Programming.
%
%   [J_opt, u_opt_ind] = Solution(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: STAY).

    global S N_a N_b P_DISTURBED P_PROTECTED
    global FREE OBSTACLE PORTAL MINE LAB BASE ALIEN
    global SOUTH NORTH EAST WEST STAY
    global EMPTY GEMS
    global UPPER LOWER
    global K M N L
    global TERMINAL_STATE_INDEX

    % Do yo need to do something with the teminal state before solving the problem?
    f = -1 * ones(K-1,1);

    % remove terminal rows
    P_old = P;
    G_old = G;
    P(TERMINAL_STATE_INDEX,:,:) = [];
    P(:,TERMINAL_STATE_INDEX,:) = [];
    G(TERMINAL_STATE_INDEX,:) = [];
    
    A_south = eye(K-1) - P(:,:,SOUTH); % K-1xK
    A_north = eye(K-1) - P(:,:,NORTH);
    A_east = eye(K-1) - P(:,:,EAST);
    A_west = eye(K-1) - P(:,:,WEST);
    A_stay = eye(K-1) - P(:,:,STAY);

    A = [A_south; A_north; A_east; A_west; A_stay];

    G(G == Inf) = 10000;

    b_south = G(:, SOUTH);
    b_north = G(:, NORTH);
    b_east = G(:, EAST);
    b_west = G(:, WEST);
    b_stay = G(:, STAY);

    b = [b_south; b_north; b_east; b_west; b_stay];

    lb = zeros(K-1,1);
    ub = [];

    J_opt = linprog(f, A, b, [], [], lb, ub);
    J_opt = [J_opt(1:TERMINAL_STATE_INDEX-1); 0; J_opt(TERMINAL_STATE_INDEX:end)];
    u_opt_ind = NaN(K,1);

    P = P_old;
    G = G_old;

    for state_i = 1:K
        cost_per_action = zeros(5,1);
        for action = 1:L
            % find Q(i,u) + sum[P(i,u,j)*V(j)]
            state_js = find(P(state_i,:,action)); % look for all nonzero values of state j
            expected_value = 0; % sum part
            for state_j = state_js % for each index that are non zeroes
                expected_value = expected_value + P(state_i, state_j, action) * J_opt(state_j);
            end
            cost_per_action(action) = G(state_i, action) + expected_value;
        end
        [~, min_idx] = min(cost_per_action);
        u_opt_ind(state_i) = min_idx;
    end
end