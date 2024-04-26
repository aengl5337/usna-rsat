%% manualPilot.m
% Plots experiment based on given motor angles

%{
TODO:
%}

%% Clean up
clc
clear all
close all

%% Add path and all subdirs
path = "G:\My Drive\RSat master\softwareWorkingDir\USNA_NSTAR_RSat\robotics\archive\manual";
addpath(genpath(path));
%% Initiate MSG object (and RSAT for invKin)
experiment = MSG();
RECS = RSAT();

%% Pull target pose from .ply
zmax = .62;  % maximum depth displayed in m (D435 has depth noise out to infinity depth)
% (good values: zmax = .55 for ISS 10JAN24, .62 for Houston test)
CANNED = true; % MUST BE false TO ACTUALLY USE VALUES
TMSG_tgt = experiment.MSGplyGrabPose(zmax, CANNED);
% NOTE: the particular choice of x,y axes in the above does not matter --
% this will be accounted for in the later code

%% Input motor position if other than stowed
% NOTE: all motor angles are relative to the stowed position
theta_relstowed = input("Input vector of current motor angles relative to the stowed position:");
theta_relstowed = reshape(theta_relstowed,6,1); % Ensure is a column vector

%% PLOT
theta_armI_relstowed = zeros(6,1);
theta_armII_relstowed = theta_relstowed;
experiment.plotExperiment(theta_armI_relstowed,theta_armII_relstowed,TMSG_tgt)

%% Display displacement of end effector from target
THETAREF = "stow";
TMSG_armII_is = experiment.MSGfwdKin(theta_armII_relstowed,THETAREF,"II");
TMSG_armII_6 = TMSG_armII_is(:,:,6);
PMSG_tgt = TMSG_tgt(1:3,4);
PMSG_armII_6 = TMSG_armII_6(1:3,4);

rMSG = PMSG_tgt-PMSG_armII_6;
r = norm(rMSG);

disp("Displacement vector (in MSG coordinates) from functioning arm's (arm II's) end effector to cube target:")
disp(rMSG)
disp("The magnitide of this vector (in mm) is:")
disp(r)
