%% Simple Interpolation Test
xIdx = [0:0.25:4];
yIdx = [0:1:3];
zIdx = [0:1:2];
nx = length(xIdx);
ny = length(yIdx);
nz = length(zIdx);

% x,y,z CONTINUOUS VALUES to state number
qp = [2.7,2.2,.75];                  % actual state values
[~,q1] = min(abs(xIdx-qp(1)));
[~,q2] = min(abs(yIdx-qp(2)));
[~,q3] = min(abs(zIdx-qp(3)));
qp = [q1,q2,q3]
stateNum = (nx*ny)*(q3-1) + nx*(q2-1) + q1   % discrete state number

% state number to ASSUMED x,y,z VALUES
qn = 114;
qz = ceil(qn / (nx*ny));
idxOnXYSlice = qn-(qz-1)*(nx*ny);
qy = ceil(idxOnXYSlice / nx);
qx = idxOnXYSlice-(qy-1)*nx;
[qx, qy, qz]   % indices
[xIdx(qx) yIdx(qy) zIdx(qz)]   % prototypical (discritized) state values

%%

state_range = [
    -10, 10;
    0, 180;          % TODO: convert this to radians after debugging!
    -2000, 2000
    ];

%%
xIdx = [0:1:180]*pi/180;      % [rad]
yIdx = [0:180:1800]*pi/180;   % [rad/s]
zIdx = [0:0.5:20];            % [m/s]
nx = length(xIdx);
ny = length(yIdx);
nz = length(zIdx);
[X,Y,Z] = meshgrid(xIdx,yIdx,zIdx);