clear all 
close all
format compact
clc

%% NSTAR_ArmSim_createSymbolic.m

%{
TODO

() + Offload functionality into a function

%}

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
q_fwd = sym('q_fwd',[1,6]);
N = numel(q_fwd);
r0_6QR = zeros(3,1); % What would be more valuable is offset in the end effector's coordinates
% lee=44.5;  % Measured to endpoint of end effector 
%            % (not grabbing centroid)
q_types = zeros(1,N); % specify 0 for each joint (revolute)

T0_i = forwardKin(q_fwd, q_types, D_H, "mod","abs");
% matlabFunction(T0_i,"File","forwardKin_explicit")

%% INVERSE KINEMATICS
% T0_QR = sym('T0_QR',[4,4]);
% r0_6QR = zeros(3,1); % relative offset
% q_types = zeros(1,6); % specify 0 for each joint (revolute)
% [theta_invs,T0_i_invs,eaxes,thetasErr,delPs,magPs] = pieperInvKin(T0_QR, r0_6QR, q_types, D_H, "mod");
