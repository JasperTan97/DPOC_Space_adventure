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

    % Iterate over all states
    for k = 1:K
        m = stateSpace(k,1);    % 'x' value
        n = stateSpace(k,2);    % 'y' value
        phi = stateSpace(k,3);  % Carrying gems
        psi = stateSpace(k,4);  % Lower (1) or upper (0) world

        % find allowable control inputs
        [admissible_inputs, next_m, next_n, next_phi, next_psi] = get_admissible_control_inputs(m,n,phi,psi);

        for input_idx = 1:5
            if admissible_inputs(input_idx)==false
                % Not possible to apply this action
                G(k,input_idx) = inf;
                continue;
            end

            % r corresponds to the list of states that have nonzero probability
            % of transition
            [r,~,~] = ind2sub(size(P),find(P(k,:,input_idx)~=0));
            k_next = ismember(stateSpace, [next_m(input_idx) next_n(input_idx) next_phi(input_idx) next_psi(input_idx)], 'rows');

            for k_end=r
                if k_next==k_end
                    % We got where we intended. Continue (cost is 1)
                else
                    % We did not get what we intended. ???

                    % Disambiguate probabilities here.

                    % -> Move more than 1 square away. => Disturbed, additional cost of 1
                    % -> Move home. => Crashed, extra fuel of N_b
                    %   => WITH THE EXCEPTION of deliberately moving home (covered above) 
                    %      or joint probabilities of moving home AND ...
                    % -> Alien attack.

                    % Therefore it makes more sense to have probability calculation in full?
                end
            end
        end
    end
end


%%%%% Auxilliary functions
function [adm_inputs, next_m, next_n, next_phi, next_psi] = get_admissible_control_inputs(m,n,phi,psi)
    % Given a current state (easting, northing, lower/upper) we return if actions 
    % [SOUTH NORTH WEST EAST STAY] are valid
    % Also returns a vector of where we will go.

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

    % ! Vectorize this if possible
    % Flip the value of psi if we are in a portal.
    next_psi = [psi psi psi psi psi];
    for k=1:5
        if map(next_m(k), next_n(k))==PORTAL
            next_psi(k) = ~psi;
        end
    end

    next_phi = [phi phi phi phi phi];
    for k=1:5
        if map(next_m(k), next_n(k))==MINE && next_psi(k)==LOWER
            next_phi(k) = GEMS;
        end
    end
end

function vert_dir = north_or_south(m,n,psi)
    % Returns which direction of vertical movement (NORTH, SOUTH) is currently 
    % possible

    vert_dir = mod( (m+n+psi), 2 );
    % It turns out that this is a neat way to express if we can go north or south
    if vert_dir
        vert_dir = NORTH;
    else
        vert_dir = SOUTH;
    end
end