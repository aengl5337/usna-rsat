clear all 
close all
format compact
clc

%% RSAT_ISSopsScript.m
%{
REV0 - 08JAN24, Alec Engl

TODO:
() = incomplete, (X) = complete
+ = addition, - = fix/debug, !=test

() + Add input filtering
() ! Ensure that conversion from unit vectors to rotation matrix works
%}

%% INIT
path = "G:\My Drive\RSat master\softwareWorkingDir\USNA_NSTAR_RSat\robotics\archive\manual";
addpath(genpath(path)); % Add path and all subdirectories


rsat = RSAT; % Create object of RSAT class

%% INPUT
% *NOTE: these vectors MUST be specified in the MSG frame, as 3x1 arrays (column vectors)

% SPECIFY DESIRED COORDINATE FRAME OF END EFFECTOR*
zhat_EE = input('Input desired z axis vector of end effector:\n');
xhat_EE = input('Input desired x axis vector of end effector:\n');

% SPECIFY DESIRED POSITION OF END EFFECTOR TIP*
PMSG_EE = input('Input desired position vector of end effector tip:\n');

%% PREPROCESS INPUT
% NORMALIZE AND ORTHOGONALIZE AXES (use z axis as reference)
yhat_EE = cross(zhat_EE,xhat_EE);
xhat_EE = cross(yhat_EE,zhat_EE); % This x axis is now orthogonal in the plane of the originally specified x and z

% Normalize to get rid of any numerical imprecision
xhat_EE = normalizeVector(xhat_EE);
yhat_EE = normalizeVector(yhat_EE);
zhat_EE = normalizeVector(zhat_EE);

% CREATE HOMOGENOUS TRANSFORM
xhat_EE = reshape(xhat_EE,3,1);
yhat_EE = reshape(yhat_EE,3,1);
zhat_EE = reshape(zhat_EE,3,1);
RMSG_EE = [xhat_EE yhat_EE zhat_EE];

PMSG_EE = reshape(PMSG_EE,3,1);
TMSG_EE = [RMSG_EE,    PMSG_EE;
           zeros(1,3), 1];
       
% TRANSFORM TO MOTOR 6 FRAME
TMSG_6 = TMSG_EE; % Trivial to test
%% FIND SOLUTIONS (and qualify validity)
theta_invs = rsat.invKin(TMSG_6);
