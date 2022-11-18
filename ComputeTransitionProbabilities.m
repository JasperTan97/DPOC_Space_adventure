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
    
    % Compute mapping from one state to another
    keys = zeros(1,K);
    for k=1:K
        state = stateSpace(k,:);
        keys(k) = state(1)*N*16 + state(2)*8 + state(3)*4 + state(4);
    end
    state_to_k_map = containers.Map(keys, 1:K);

    % set P to all zeroes, because most probabilities are zero since far
    % away states are unreachable directly
    P = zeros(K,K,L);
    for base_x = 1:M % We find the base
        break_all = false;
        for base_y = 1:N
            if map(base_x,base_y) == BASE
                break_all = true;
                break
            end
        end
        if break_all
            break
        end
    end
    base_state = [base_x, base_y, EMPTY, UPPER];

    % iterate through all states to get transition probs for each
    parfor (k = 1:K, 4)
        statecurrent = stateSpace(k,:);
        m = statecurrent(1);
        n = statecurrent(2);
        phi = statecurrent(3);
        psi = statecurrent(4);
        prob_k = zeros(K,L);

        % just walking either left, right or north/south
        stateleft = [m-1,n,phi,psi];
        stateright = [m+1,n,phi,psi];
        statesouth = [m,n-1,phi,psi];
        statenorth = [m,n+1,phi,psi];

        % now, depending on terrain, is north or south possible
        northorsouthpossible = northorsouth(m,n,psi);

        % for each action in this current state
        for l = 1:L
            % should i stay or should i go
            motionpossible = false;
            staying = false;
            % disp([statecurrent, l])

            if (map(statecurrent(1), statecurrent(2)) == LAB && statecurrent(3) == GEMS && statecurrent(4) == UPPER)
                staying = true;
            end

            if l == SOUTH
                % going south, must be possible map wise, obstacle free, not off the map
                state0 = statesouth;
                if northorsouthpossible == "south" && statecurrent(2) > 1 && ...
                    map(state0(1), state0(2)) ~= OBSTACLE

                    motionpossible = true;
                end
            end
            if l == NORTH
                state0 = statenorth;
                if northorsouthpossible == "north" && statecurrent(2) < N && ...
                    map(state0(1), state0(2)) ~= OBSTACLE
                    
                    motionpossible = true;
                end
            end
            if l == WEST
                state0 = stateleft; % west is left
                if statecurrent(1) > 1 && map(state0(1), state0(2)) ~= OBSTACLE
                    motionpossible = true;
                end
            end
            if l == EAST
                state0 = stateright; % east is right
                if statecurrent(1) < M && map(state0(1), state0(2)) ~= OBSTACLE
                    motionpossible = true;
                end
            end
            if l == STAY % stay only at terminal state
                state0 = statecurrent;
                motionpossible = true;
            end

            if motionpossible
                [state1, gotgems1, aliens1] = eventsresolver(state0,map);
                % Radiation movement to state2_i where i = {1,2,3} ==
                % {left, right, north/south}. state1's psi is used to
                % determine serverity of radiation
                [state2_1, state2_2, state2_3,...
                    gotgems2_1, gotgems2_2, gotgems2_3,...
                    aliens2_1, aliens2_2, aliens2_3] = after_radiation(state1, map, base_state);
            
            end

            % for calculation of probabilities:
            % 1) get position states (x,y,psi) by using p_disturbed/3 or with shielding
            % by checking state1 psi value (and not state2)
            % 2) get gem state by calculating probabilities based on number of
            % alien attacks in state 2 (if gotgems2 is false) + number of alien
            % attacks in state 1 (if gotgems1 is false). Use that as the
            % exponent when using p_protected
            % in summary, we will have p_ij, where i is current_state aka state_k
            % and j can take either state1 or state 2_{1,2,3}

            if staying
                prob_k(k,l) = 1;
        
            elseif motionpossible
                if state1(4) == UPPER
                    % do 1)
                    p_k1_b4gems = 1 - P_DISTURBED;
                    p_k21_b4gems = P_DISTURBED/3;
                    p_k22_b4gems = P_DISTURBED/3;
                    p_k23_b4gems = P_DISTURBED/3;
                else
                    p_k1_b4gems = 1 - P_DISTURBED*S;
                    p_k21_b4gems = P_DISTURBED*S/3;
                    p_k22_b4gems = P_DISTURBED*S/3;
                    p_k23_b4gems = P_DISTURBED*S/3;
                end
                % now do 2
                gem_checker = [ aliens1  aliens2_1  aliens2_2  aliens2_3;
                                gotgems1 gotgems2_1 gotgems2_2 gotgems2_3];
                for check = 1:4
                    if check ~= 1
                        gem_checker(1,check) = gem_checker(1,check) + gem_checker(1,1);
                    end
                    if gem_checker(2,check) % if gems were gotten after alien attacks
                        gem_checker(1,check) = 0; % total alien fights that could independently make you lose gems
                    end
                end
                % we only need to do this step if the state has gems
                if state1(3) == GEMS
                    p_k1_wgems = p_k1_b4gems * P_PROTECTED^gem_checker(1,1);    % the number of aliens
                    p_k1_wogems = p_k1_b4gems - p_k1_wgems;
                else
                    % If you don't have gems before you won't have gems after
                    p_k1_wogems = p_k1_b4gems; 
                    p_k1_wgems = 0;
                end

                % The chance of keeping our gems after being disturbed is the prob of defending
                % from aliens in the first stage (k1) and then again after being displaced.
                if state2_1(3) == GEMS
                    p_k21_wgems = p_k21_b4gems * P_PROTECTED^(gem_checker(1,2));
                    p_k21_wogems = p_k21_b4gems - p_k21_wgems;
                else
                    p_k21_wogems = p_k21_b4gems;
                    p_k21_wgems = 0;
                end
                if state2_2(3) == GEMS
                    p_k22_wgems = p_k22_b4gems * P_PROTECTED^(gem_checker(1,3));
                    p_k22_wogems = p_k22_b4gems - p_k22_wgems;
                else
                    p_k22_wogems = p_k22_b4gems;
                    p_k22_wgems = 0;
                end
                if state2_3(3) == GEMS
                    p_k23_wgems = p_k23_b4gems * P_PROTECTED^(gem_checker(1,4));
                    p_k23_wogems = p_k23_b4gems - p_k23_wgems;
                else
                    p_k23_wogems = p_k23_b4gems;
                    p_k23_wgems = 0;
                end

                % find indexes in statespace
                k1_wgems = state_to_k(state1(1),state1(2),GEMS,state1(4), state_to_k_map);
                k1_wogems = state_to_k(state1(1),state1(2),EMPTY,state1(4), state_to_k_map);
                k21_wgems = state_to_k(state2_1(1),state2_1(2),GEMS,state2_1(4), state_to_k_map);
                k21_wogems = state_to_k(state2_1(1),state2_1(2),EMPTY,state2_1(4), state_to_k_map);
                k22_wgems = state_to_k(state2_2(1),state2_2(2),GEMS,state2_2(4), state_to_k_map);
                k22_wogems = state_to_k(state2_2(1),state2_2(2),EMPTY,state2_2(4), state_to_k_map);
                k23_wgems = state_to_k(state2_3(1),state2_3(2),GEMS,state2_3(4), state_to_k_map);
                k23_wogems = state_to_k(state2_3(1),state2_3(2),EMPTY,state2_3(4), state_to_k_map);

                % update transition probabilities
                prob_k( k1_wgems, l) = prob_k( k1_wgems, l) + p_k1_wgems;
                prob_k( k1_wogems, l) = prob_k( k1_wogems, l) + p_k1_wogems;
                prob_k( k21_wgems, l) = prob_k( k21_wgems, l) + p_k21_wgems;
                prob_k( k21_wogems, l) = prob_k( k21_wogems, l) + p_k21_wogems;
                prob_k( k22_wgems, l) = prob_k( k22_wgems, l) + p_k22_wgems;
                prob_k( k22_wogems, l) = prob_k( k22_wogems, l) + p_k22_wogems;
                prob_k( k23_wgems, l) = prob_k( k23_wgems, l) + p_k23_wgems;
                prob_k( k23_wogems, l) = prob_k( k23_wogems, l) + p_k23_wogems;
            end
        end

        P(k,:,:) = prob_k;
    end
end

function [state2_1, state2_2, state2_3,...
          gotgems2_1, gotgems2_2, gotgems2_3,...
          aliens2_1, aliens2_2, aliens2_3] = after_radiation(statex, map, base_state)
    m = statex(1);
    n = statex(2);
    phi = statex(3);
    psi = statex(4);

    state2_1 = [m-1,n,phi,psi];
    state2_2 = [m+1,n,phi,psi];
    northorsouthpossible = northorsouth(m,n,psi);
    if northorsouthpossible == "north"
        state2_3 = [m,n+1,phi,psi];
    else
        state2_3 = [m,n-1,phi,psi];
    end
    
    % now check if out of bounds, if it is, return to base.
    function oob = outofboundschecker(statex) % first we check if OOB 
        global M N OBSTACLE
        m_oob = statex(1);
        n_oob = statex(2);
        if m_oob < 1 || m_oob > M || n_oob < 1 || n_oob > N ||...
            map(m_oob,n_oob) == OBSTACLE
            oob = true;
        else
            oob = false;
        end
    end
    
    gotgems2_1 = false;
    gotgems2_2 = false;
    gotgems2_3 = false;
    aliens2_1 = 0;
    aliens2_2 = 0;
    aliens2_3 = 0;

    if outofboundschecker(state2_1)
        state2_1 = base_state;
    else
        [state2_1, gotgems2_1, aliens2_1] = eventsresolver(state2_1, map);
    end
    if outofboundschecker(state2_2)
        state2_2 = base_state;
    else
        [state2_2, gotgems2_2, aliens2_2] = eventsresolver(state2_2, map);
    end
    if outofboundschecker(state2_3)
        state2_3 = base_state;
    else
        [state2_3, gotgems2_3, aliens2_3] = eventsresolver(state2_3, map);
    end
end

function [nextstate, gotgems, alien_attacks] = eventsresolver(statex, map)
    % Input: current state and map
    % Output: next state, gotgems as a helper for calculating state
    % probabilities of losing gems, number of alien_attacks as a helper for
    % calulating probabilities of losing gems too
    global PORTAL MINE GEMS LOWER UPPER
    % in any world, we have to check for:
    % 1) Portal == psi 0 -> 1. 
    % 2) if we take the portal, check if aliens attack
    % 3) Mine == psi 
    nextstate = statex;
    gotgems = false;
    alien_attacks = 0;
    if nextstate(4) == UPPER
        if map(nextstate(1), nextstate(2)) == PORTAL
            nextstate(4) = LOWER;
        end
    else % if LOWER
        if map(nextstate(1), nextstate(2)) == PORTAL
            nextstate(4) = UPPER;
        end
    end
    if nextstate(4) == LOWER % checking if aliens attack
        alien_attacks = alienchecker(nextstate, map);
        % we only care about the chance of losing gems
        % when imputing probabilities, now we simply store
        % the number of aliens encountered
    end
    if nextstate(4) == LOWER && map(nextstate(1), nextstate(2)) == MINE % checking if mine
        nextstate(3) = GEMS;
        gotgems = true;
    end
end

function alienattack = alienchecker(statex, map)
% looks around to find how many aliens can attack
% returns number of aliens ready to strike
    global ALIEN M N
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
    northorsouthpossible = northorsouth(m,n,psi);
    if n > 1 && northorsouthpossible == "south" % alien is south
        if map(m,n-1) == ALIEN
            alienattack = alienattack + 1;
        end
    end
    if n < N && northorsouthpossible == "north" % alien is north
        if map(m,n+1) == ALIEN
            alienattack = alienattack + 1;
        end
    end
end


function northorsouthpossible = northorsouth(m,n,psi)
    if psi == 0 % UPPER
        if (mod(m,2)==1 && mod(n,2)==1) || (mod(m,2)==0 && mod(n,2)==0)
            northorsouthpossible = "south";
        else
            northorsouthpossible = "north";
        end
    else
        if (mod(m,2)==1 && mod(n,2)==1) || (mod(m,2)==0 && mod(n,2)==0)
            northorsouthpossible = "north";
        else
            northorsouthpossible = "south";
        end
    end
end

function k_idx = state_to_k(m,n,phi,psi,map)
    global N
    map_idx = m*N*16 + n*8 + phi*4 + psi;
    k_idx = map(map_idx);
end