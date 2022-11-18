function G = ComputeStageCosts( stateSpace, map, P )
%ComputeStageCosts Compute stage costs.
%
%   G = ComputeStageCosts(stateSpace, map, P) computes the stage costs 
%   for all states in the state space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 4)-matrix, where the i-th row represents the i-th
%           element of the state space.
%       
%       map:
%           A (M x N)-matrix describing the terrain of the estate map. 
%           With values: FREE OBSTACLE PORTAL ALIEN MINE 
%           LAB BASE
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%   Output arguments:
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global S N_a N_b P_DISTURBED P_PROTECTED
    global FREE OBSTACLE PORTAL MINE LAB BASE ALIEN
    global SOUTH NORTH EAST WEST STAY
    global EMPTY GEMS
    global UPPER LOWER
    global K M N L
    global TERMINAL_STATE_INDEX

    G = ones(K,L);  % by default all actions have a cost of 1.

    % Get terminal state
    G(TERMINAL_STATE_INDEX,:) = zeros(1,5);

    % Iterate over all states
    for k = 1:K
        % Don't need to process further if we are at the terminal state
        if k==TERMINAL_STATE_INDEX
            continue
        end

        m = stateSpace(k,1);    % 'x' value
        n = stateSpace(k,2);    % 'y' value
        phi = stateSpace(k,3);  % Carrying gems
        psi = stateSpace(k,4);  % Lower (1) or upper (0) world

        % find allowable control inputs
        [admissible_inputs, next_m, next_n] = get_admissible_control_inputs(m,n,psi,map);

        for input_idx = 1:5
            if admissible_inputs(input_idx)==false
                % Not possible to apply this action
                G(k,input_idx) = Inf;
                continue;
            end

            % Compute probababilities of certain events happening
            % assert(next_m(input_idx)<=M, "Error state (%d,%d)", next_m(input_idx), next_n(input_idx))
            % assert(next_n(input_idx)<=N, "Error state (%d,%d)", next_m(input_idx), next_n(input_idx))
            next_state = [next_m(input_idx) next_n(input_idx) phi psi];
            [next_state, num_aliens_1] = eventsresolver(next_state, map);

            % Calculate expected probability of aliens attacking
            % Extra cost simply the number of aliens attacking at that spot!
            G(k,input_idx) = G(k,input_idx) + N_a * num_aliens_1;

            % Calculate probability of being disturbed by radiation
            [ num_aliens_2_1, num_aliens_2_2, num_aliens_2_3,...
              crash2_1, crash2_2, crash2_3 ] = after_radiation(next_state, map);

            % Probability of being disturbed
            if next_state(4)==UPPER
                p_disturbed = P_DISTURBED/3;
            else
                p_disturbed = S*P_DISTURBED/3;
            end

            % Resolve cost at disturbed areas

            % If we crashed, then we add N_b+1 to the expectation. Else we just add 1
            if crash2_1
                G(k,input_idx) = G(k,input_idx) + p_disturbed*(N_b+1);
            else
                G(k,input_idx) = G(k,input_idx) + p_disturbed*(1);
            end
            % Also the expected number of aliens at the disturbed position
            G(k,input_idx) = G(k,input_idx) + p_disturbed*(N_a*num_aliens_2_1);

            if crash2_2
                G(k,input_idx) = G(k,input_idx) + p_disturbed*(N_b+1);
            else
                G(k,input_idx) = G(k,input_idx) + p_disturbed*(1);
            end
            % Also the expected number of aliens at the disturbed position
            G(k,input_idx) = G(k,input_idx) + p_disturbed*(N_a*num_aliens_2_2);

            if crash2_3
                G(k,input_idx) = G(k,input_idx) + p_disturbed*(N_b+1);
            else
                G(k,input_idx) = G(k,input_idx) + p_disturbed*(1);
            end
            % Also the expected number of aliens at the disturbed position
            G(k,input_idx) = G(k,input_idx) + p_disturbed*(N_a*num_aliens_2_3);
        end
    end
end


%%%%% Auxilliary functions
function [adm_inputs, next_m, next_n] = get_admissible_control_inputs(m,n,psi,map)
    % Given a current state (easting, northing, lower/upper) we return if actions 
    % [SOUTH NORTH WEST EAST STAY] are valid
    % Also returns a vector of where we will go.
    global M N NORTH SOUTH EAST WEST OBSTACLE

    % Not sure if directions are valid, but stay is always valid.
    adm_inputs = [false false false false true];
    next_m = [m m m-1 m+1 m];
    next_n = [n-1 n+1 n n n];

    if (m-1) >= 1 && map(m-1,n)~=OBSTACLE
        % Check if we can move west
        adm_inputs(WEST) = true;
    end
    
    if (m+1) <= M && map(m+1,n)~=OBSTACLE
        % Check if we can move east
        adm_inputs(EAST) = true;
    end
    
    vert_input = north_or_south(m,n,psi);
    if vert_input==NORTH
        % Check if we can move up or down
        if (n+1) <= N && map(m,n+1)~=OBSTACLE
            adm_inputs(NORTH) = true;
        end
    else
        if (n-1) >= 1 && map(m,n-1)~=OBSTACLE
            adm_inputs(SOUTH) = true;
        end
    end
end

function vert_dir = north_or_south(m,n,psi)
    % Returns which direction of vertical movement (NORTH, SOUTH) is currently 
    % possible
    global NORTH SOUTH

    vert_dir = mod( (m+n+psi), 2 );
    % It turns out that this is a neat way to express if we can go north or south
    if vert_dir==1
        vert_dir = NORTH;
    else
        vert_dir = SOUTH;
    end
end

function [nextstate, alien_attacks] = eventsresolver(statex, map)
    % Input: current state and map
    % Output: next state as a helper for calculating state
    % probabilities of losing gems, number of alien_attacks as a helper for
    % calulating probabilities of losing gems too
    global PORTAL LOWER
    % in any world, we have to check for:
    % 1) Portal == psi 0 -> 1. 
    % 2) if we take the portal, check if aliens attack
    % 3) Mine == psi 
    nextstate = statex;
    alien_attacks = 0;

    % Resolve portal
    if map(nextstate(1), nextstate(2)) == PORTAL
        nextstate(4) = ~nextstate(4);   % a NOT operation suffices
    end

    % checking if aliens attack
    if nextstate(4) == LOWER
        alien_attacks = alienchecker(nextstate, map);
        % we only care about the chance of losing gems
        % when imputing probabilities, now we simply store
        % the number of aliens encountered
    end
end

function alienattack = alienchecker(statex, map)
% looks around to find how many aliens can attack. 
% Returns number of aliens ready to strike
    global ALIEN M N NORTH
    m = statex(1);
    n = statex(2);
    psi = statex(4);
    alienattack = 0;

    if map(m,n) == ALIEN
        alienattack = alienattack + 1;
    end
    
    if m > 1 % check if alien on the left
        if map(m-1,n) == ALIEN
            alienattack = alienattack + 1;
        end
    end

    if m < M % alien on the right
        if map(m+1,n) == ALIEN
            alienattack = alienattack + 1;
        end
    end
    
    if north_or_south(m,n,psi)==NORTH
        if n<N && map(m,n+1) == ALIEN
            alienattack = alienattack + 1;
        end
    else
        if n>1 && map(m,n-1) == ALIEN
            alienattack = alienattack + 1;
        end
    end
end

function [num_aliens_2_1, num_aliens_2_2, num_aliens_2_3,...
          crash2_1, crash2_2, crash2_3] ...
          = after_radiation(statex, map)

    global NORTH M N OBSTACLE

    m = statex(1);
    n = statex(2);
    phi = statex(3);
    psi = statex(4);

    state2_1 = [m-1,n,phi,psi];
    state2_2 = [m+1,n,phi,psi];

    if north_or_south(m,n,psi)==NORTH
        state2_3 = [m,n+1,phi,psi];
    else
        state2_3 = [m,n-1,phi,psi];
    end

    % now check if out of bounds, if it is, return to base.
    % first we check if OOB 
    function oob = outofboundschecker(statex)
        m_oob = statex(1);
        n_oob = statex(2);

        oob = m_oob < 1 || m_oob > M || ... 
              n_oob < 1 || n_oob > N || ...
              map(m_oob,n_oob) == OBSTACLE;
    end

    num_aliens_2_1 = 0;
    num_aliens_2_2 = 0;
    num_aliens_2_3 = 0;
    crash2_1 = false;
    crash2_2 = false;
    crash2_3 = false;

    if outofboundschecker(state2_1)
        crash2_1 = true;
    else
        [~, num_aliens_2_1] = eventsresolver(state2_1, map);
    end
    if outofboundschecker(state2_2)
        crash2_2 = true;
    else
        [~, num_aliens_2_2] = eventsresolver(state2_2, map);
    end
    if outofboundschecker(state2_3)
        crash2_3 = true;
    else
        [~, num_aliens_2_3] = eventsresolver(state2_3, map);
    end
end