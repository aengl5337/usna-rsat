%% DHdemo.m
% Test case for Modified D-H parameters

%% Some constants
path = "G:\My Drive\RSat master\softwareWorkingDir\USNA_NSTAR_RSat\robotics\archive\manual";
addpath(path);

% Denavit-Hartenberg Parameters (Modified -- Craig*)
% Link twist angles (alpha, deg)
alpha = 90*[0 1 0]'; % i-1
% Link Lengths (a, mm)
a = [0 0 0]'; % i-1
% (Intrinsic) Joint angles (theta, deg)
theta0 = [0 0 0]';

% Link offsets (d, mm)
l2 = 200;
d0 = [0 0 l2]'; % i

D_H = [alpha,a,theta0,d0];


%% FORWARD KINEMATICS
% % FIND JOINT POSES GIVEN AN INPUT THETA
% q_fwd = [-90 0 45 0 0 0]'; % index similarly to the rest of the D_H params, add 0 entry
% q_fwd = [0 -90 -180 0 180 -90]'; %***
q_fwd = [0 -50 0];
N = numel(q_fwd);
q_types = [0 1 0]; % specify 0 for each revolute joint, and 1 for each prismatic one

T0_i = forwardKin(q_fwd, q_types, D_H, "mod","abs");

% % DEVISE A CANNED QR TARGET THAT MATCHES (6) OF CURRENT ARM ORIENTATION
% Rotation
T0_end = T0_i(:,:,end); % Joint 6 (end effector) pose in 0 frame
% NOTE: since motor 6's axis points outwards through the wrist, must
% negate it to get an outward facing normal for the QR code
R0_end = T0_end(1:3,1:3);
Xhat0_end =  R0_end(:,1);
Yhat0_end =  R0_end(:,2);
Zhat0_end =  R0_end(:,3);

Xhat0_QR = -R0_end(:,1);
Yhat0_QR =  R0_end(:,2);
Zhat0_QR = -R0_end(:,3);
R0_QR = [Xhat0_QR,Yhat0_QR,Zhat0_QR];

% Position
r0_6QR = zeros(3,1); % relative offset
P0_6 = T0_end(1:3,4);
P0_QR = P0_6+r0_6QR; % Compensate for relative offset

% Combine in homogenous pose matrix
T0_QR = [R0_QR,   P0_QR;
         zeros(1,3), 1];

% % PLOT
plotRobotAxes(T0_i,T0_QR)



