% Tests our script for Stage Costs
main;
Gx = G;
load('exampleG_1.mat');

% Find places with errors
[startK, moveType] = find( abs(G(:,:) - Gx(:,:)) > 1e-10 );
fprintf("Number of errors: %d\n", size(startK,1) )

if size(startK,1)==0
    disp("Congrats, test_stage_costs has no errors.")
    return
end

disp("First error startK has state:")
stateSpace(startK(1),:)

fprintf("Chosen action: %d\n", moveType)

disp("Calculated cost: Gx")
Gx(startK(1), moveType(1))
disp("Answer cost: G")
P(startK(1), moveType(1))