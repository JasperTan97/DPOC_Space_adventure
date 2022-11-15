% Tests our script
main;
Px = P;
load('exampleP_3.mat');

openvar('P(:,:,1)');
openvar('Px(:,:,1)');

for moveType = 1:5       % Vary this from 1-5

    % Find places with errors
    [startK, destK] = find( abs(P(:,:,moveType) - Px(:,:,moveType)) > 1e-10 );
    fprintf("Number of errors for moveType %d: %d\n", moveType, size(startK,1) )

    if size(startK,1)==0
        continue
    end

    disp("First error startK has state:")
    stateSpace(startK(1),:)

    disp("First error destK has state:")
    stateSpace(destK(1),:)

    fprintf("Chosen action: %d\n", moveType)

    disp("Calculated probability: Px")
    Px(startK(1),destK(1),moveType(1))
    disp("Answer probability: P")
    P(startK(1),destK(1),moveType(1))

end