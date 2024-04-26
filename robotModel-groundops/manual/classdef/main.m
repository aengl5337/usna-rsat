%% main.m
% DEPRECATED AS OF 14MAR24
% Performs inverse kinematics to determine motor angles based on a target
% and desired task

%{
TODO:
() + Consolidate invKin() within the MSG functionality
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
CANNED = true; % MUST BE false TO ACTUALLY USE VALUES
TMSG_tgt = experiment.MSGplyGrabPose(CANNED);
% NOTE: the particular choice of x,y axes in the above does not matter --
% this will be accounted for in the later code


%% Define relationship between target and goal frame of frame (6)
% Removes dependence on end effector
whichArm = "II"; % Only functioning arm
[T0_6s,Nsol] = experiment.MSGtgtWtask2goal(TMSG_tgt,whichArm,"move2QR");

%% Determine possible invKin sol'ns
% Initial reference is the stowed position, so we must change from the model reference 
theta_invss_relstowed = zeros(6,8,Nsol); % would additional solutions be trivial, i.e. just rotations of theta6?
THETAREF = "stow";
for nn = 1:Nsol
    T0_6 = T0_6s(:,:,nn);
    theta_invs_relstowed = RECS.invKin(T0_6,THETAREF);
    theta_invss_relstowed(:,:,nn) = theta_invs_relstowed;
end



%% Input motor position if other than stowed
% NOTE: all motor angles are relative to the stowed position
thetainit_restowed = input("Input vector of current motor angles relative to the stowed position:");
thetainit_restowed = reshape(thetainit_restowed,6,1); % Ensure is a column vector

% Flatten index
Ntot = 8*Nsol;
theta_invss_relinit = zeros(6,Ntot);
for ii = 1:Nsol
    for jj = 1:8
        ind = jj+(ii-1)*8; % This will match the figure number
        theta_invss_relinit(:,ind) = theta_invss_relstowed(:,jj,ii) - thetainit_restowed;
    end
end

%% PLOT
% % experiment.plotMSG(false)
% % Test stowed
% theta_armI_relstowed = zeros(6,1);
% theta_armII_relstowed = zeros(6,1);
% experiment.plotExperiment(theta_armI_relstowed,theta_armII_relstowed,TMSG_tgt)

for ind = 1:Ntot
    theta_invs_relinit = theta_invss_relinit(:,ind);
    chknan = sum(isnan(theta_invs_relinit));
    if chknan == 0
        theta_armII_relstowed = theta_invs_relinit; % Only functioning arm
        theta_armI_relstowed = zeros(6,1);
        experiment.plotExperiment(theta_armI_relstowed,theta_armII_relstowed,TMSG_tgt)
    else
        txt = strcat("Soln ", num2str(ind)," DNE");
        disp(txt)
    end
end
    

