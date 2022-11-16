function [ J_opt, u_opt_ind ] = PI_sol(P, G)
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

    % Do you need to do something with the terminal state before solving the problem?

    P_prev = P;
    G_prev = G;

    % Get rid of terminal state row/cols
    P(TERMINAL_STATE_INDEX,:,:) = [];
    P(:,TERMINAL_STATE_INDEX,:) = [];
    G(TERMINAL_STATE_INDEX,:) = [];

    % Initialise a proper policy
    % Point left (arbitrary) except when it is impossible. Then point [right,north,south] (whichever is possible)
    mu = ones(K,1,'int8');
    % Cost-free termination state
    
    for k=1:K
        % A proper policy is if there exists an integer m s.t. P(x_m=0 | x_0 = i) > 0 for all i in S.
        % i.e. for all states in S, there exists a chance to reach the termination state.
        
        % Mu contains valid actions if the sum of probabilities to get elsewhere is > 1.
        if sum(P_prev(k,:,EAST)) > 1e-8
            mu(k) = EAST;
        elseif sum(P_prev(k,:,WEST)) > 1e-8
            mu(k) = WEST;
        elseif sum(P_prev(k,:,NORTH)) > 1e-8
            mu(k) = NORTH;
        elseif sum(P_prev(k,:,SOUTH)) > 1e-8
            mu(k) = SOUTH;
        else
            assert(false, "Unable to find valid action for state %d", k)
        end
    end

    mu(TERMINAL_STATE_INDEX) = STAY;

    while 1
        mu_prev = mu;
        mu = [mu(1:TERMINAL_STATE_INDEX-1); mu(TERMINAL_STATE_INDEX+1:end)];

        % Obtain J_mu by solving J_mu(i)=Q(i,mu(i)) + sum( P(i,mu(i)j)*J_,u(j)) - Line 4
        trans_probs = zeros(K-1,K-1);   % 'P' matrix
        policy_cost = zeros(K-1,1);     % 'G' matrix
        
        for k_it=1:K-1
            trans_probs(k_it,:) = P(k_it,:,mu(k_it));   % Select the appropriate transition prob
            policy_cost(k_it) = G(k_it, mu(k_it));
        end

        % J=G+PJ -> J=inv(I-P)*G
        det(eye(K-1)-trans_probs)
        J_mu = pinv(eye(K-1)-trans_probs) * policy_cost;    % gives K-1*1-sized matrix

        assert( size(J_mu,1)==K-1, "J_mu has size (%d,%d) instead of (K-1,1)", size(J_mu,1), size(J_mu,2) )
        assert( size(J_mu,2)==1, "J_mu has size (%d,%d) instead of (K-1,1)", size(J_mu,1), size(J_mu,2) )

        % Line 5 - Find the input that minimises the policy
        mu_prime = zeros(K,1);
        for k_it=1:K-1
            min_val = Inf;
            min_l = 0;

            for l_it=1:L
                sum_over_j = sum( P(k_it,:,l_it).*J_mu(:), 'all' );
                val = G(k_it, l_it) + sum_over_j;
                if val < min_val
                    min_val = val;
                    min_l = l_it;
                end
            end
            assert(min_l~=0, "Attempted to write an invalid action to mu_prime");

            % Revert to correct indexing
            mu_prime(k_it) = min_l;
        end

        % Insert STAY as a policy at TERMINAL_STATE
        mu_prime(TERMINAL_STATE_INDEX:end) = [STAY; mu_prime(TERMINAL_STATE_INDEX:end-1)];
        
        % Line 6 - Check if mu_prime = mu (PI break conditinon)
        if isequal(mu_prev, mu_prime)
            % Correctify J_opt before returning it
            % Return relevant values
            J_opt = [J_mu(1:TERMINAL_STATE_INDEX-1); 0; J_mu(TERMINAL_STATE_INDEX:end)];       

            u_opt_ind = mu_prev;
            break
        else
            mu = mu_prime;
        end
    end
end