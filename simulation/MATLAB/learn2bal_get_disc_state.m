% x,y,z CONTINUOUS VALUES to state number
function [stateNum,idxVec,dStates] = learn2bal_get_disc_state(X,dStatesX,dStatesY,dStatesZ)

nx = length(dStatesX);
ny = length(dStatesY);
nz = length(dStatesZ);

[~,q1] = min(abs(dStatesX-X(3)));
[~,q2] = min(abs(dStatesY-X(4)));
[~,q3] = min(abs(dStatesZ-X(2)));

stateNum = (nx*ny)*(q3-1) + nx*(q2-1) + q1;   % discrete state number
idxVec = [q1,q2,q3]';
dStates = [dStatesX(q1), dStatesY(q2), dStatesZ(q3)]';

end