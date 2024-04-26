%% NSTAR_ArmSim_demo.m

%% User input
INITPOS = "EXT"; % Options: {"EXT","STOW"}


%% Some constants
path = "G:\My Drive\RSat master\softwareWorkingDir\USNA_NSTAR_RSat\robotics\archive\manual";
addpath(path);

% Lengths (mm)
l2 = 312.24;
l4 = 182.02; % l3
l4p = 7.99; % l3p
l6 = 76.6; %l5

% Denavit-Hartenberg Parameters (Modified -- Craig*)
% Link twist angles (alpha, deg)
alpha = 90*[0 1 1 1 1 -1]'; % i-1
% Link Lengths (a, mm)
a = [0 0 0 l4p 0 0]'; % i-1
% (Intrinsic) Joint angles (theta, deg)
theta0ext = 90*[0 1 0 2 0 0]'; % i; these correspond to the 'extended' position
theta0stow = theta0ext + [0 0 -180 0 +7.2 0];

if INITPOS == "EXT"
    theta0 = theta0ext;
elseif INITPOS == "STOW"
    theta0 = theta0stow;
else % Catch all misinputs
    disp("Bad input for INITPOS")
    return;
end

% Link offsets (d, mm)
d = [0 -l2 0 l4 0 l6]'; % i

D_H = [alpha,a,theta0,d];


%% FORWARD KINEMATICS
% % FIND JOINT POSES GIVEN AN INPUT THETA
% q_fwd = [-90 0 45 0 0 0]'; % index similarly to the rest of the D_H params, add 0 entry
% q_fwd = [0 -90 -180 0 180 -90]'; %***
q_fwd = [0 0 0 0 0 0];
N = numel(q_fwd);
r0_6QR = zeros(3,1); % What would be more valuable is offset in the end effector's coordinates
% lee=44.5;  % Measured to endpoint of end effector 
%            % (not grabbing centroid)
q_types = zeros(1,N); % specify 0 for each joint (revolute)

T0_i = forwardKin(q_fwd, q_types, D_H, "mod","rel"); % **** why rel?

% % DEVISE A CANNED QR TARGET THAT MATCHES (6) OF CURRENT ARM ORIENTATION
% Rotation
T0_6 = T0_i(:,:,6); % Joint 6 (end effector) pose in 0 frame
% NOTE: since motor 6's axis points outwards through the wrist, must
% negate it to get an outward facing normal for the QR code
R0_6 = T0_6(1:3,1:3);
Xhat0_6 =  R0_6(:,1);
Yhat0_6 =  R0_6(:,2);
Zhat0_6 =  R0_6(:,3);

Xhat0_QR = -R0_6(:,1);
Yhat0_QR =  R0_6(:,2);
Zhat0_QR = -R0_6(:,3);
R0_QR = [Xhat0_QR,Yhat0_QR,Zhat0_QR];

% Position
r0_6QR = zeros(3,1); % relative offset
P0_6 = T0_6(1:3,4);
P0_QR = P0_6+r0_6QR; % Compensate for relative offset

% Combine in homogenous pose matrix
T0_QR = [R0_QR,   P0_QR;
         zeros(1,3), 1];

% % PLOT
plotRobotAxes(T0_i,T0_QR)

%% Test solutions
% Now plot solutions to see
q_fwd = [0 -90 0 0 0 0]'; %***
T0_i = forwardKin(q_fwd, q_types, D_H, "mod","rel"); % **** why rel?
plotRobotAxes(T0_i,T0_QR)



