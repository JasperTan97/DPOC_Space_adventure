function [ J_opt, u_opt_ind ] = VI_sol(P, G)
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

    % Do yo need to do something with the terminal state before solving the problem?
    % since K states, we implement a value table
    V = inf(K,1);
    V_prime = zeros(K,1);
    u_opt_ind = NaN(K,1);
    epsilon = 1e-12;
    while norm(V-V_prime, "inf") > epsilon % line 3, infinity norm is the maximum V-V_prime value
        V = V_prime; % line 4
        for state_i = 1:K % line 5, for all states V'(i) = min{Q(i,u) + sum[P(i,u,j)*V(j)]}
            if state_i == TERMINAL_STATE_INDEX
                continue
            end
            cost_per_action = zeros(5,1);
            for action = 1:L
                % find Q(i,u) + sum[P(i,u,j)*V(j)]
                state_js = find(P(state_i,:,action)); % look for all nonzero values of state j
                expected_value = 0; % sum part
                for state_j = state_js % for each index that are non zeroes
                    expected_value = expected_value + P(state_i, state_j, action) * V(state_j);
                end
                cost_per_action(action) = G(state_i, action) + expected_value;
            end
            V_prime(state_i) = min(cost_per_action); % line 5, minimum section
        end
    end
    % line 6 V_star = V
    J_opt = V;
    % line 7 mu_star = argmin V(i)
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
