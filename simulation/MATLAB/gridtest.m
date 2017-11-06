
state_range = [
    -10, 10;
    0, 180;          % TODO: convert this to radians after debugging!
    -2000, 2000
    ];

%%
[X,Y,Z] = meshgrid([0:1:180],[0:180:1800],[0:0.5:20]);

%%
xIdx = [0:1:4];
yIdx = [0:1:3];
zIdx = [0:1:2];
nx = length(xIdx);
ny = length(yIdx);
nz = length(zIdx);
[X,Y,Z] = meshgrid(xIdx,yIdx,zIdx)



% state number to x,y,z indices
qn = 25
qz = ceil(qn / (nx*ny));
idxOnXYSlice = qn-(qz-1)*(nx*ny);
qy = ceil(idxOnXYSlice / nx);
qx = idxOnXYSlice-(qy-1)*nx;
[qx, qy, qz]

% x,y,z indices to state number
qp = [5,1,2];
stateNum = (nx*ny)*(qp(3)-1) + nx*(qp(2)-1) + qp(1)