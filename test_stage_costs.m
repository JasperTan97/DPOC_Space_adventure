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

fprintf("Chosen action: %d\n", moveType(1))

fprintf("Calculated cost (Gx): %0.9f\n", Gx(startK(1), moveType(1)) )
fprintf("Answer cost (G): %0.9f\n", G(startK(1), moveType(1)) )