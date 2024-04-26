%% autoPilot.m
% Performs inverse kinematics to determine motor angles based on a target
% and desired task

%{
TODO:
() + Consolidate invKin() within the MSG functionality
() -Indicate need to change to current directory for addpath to work
properly
() -Change 'Nsol' to a more appropriate name, i.e. Norientations


%}

%% Clean up
clc
clear all
close all

%% Add path and all subdirs
path = "..";
addpath(genpath(path));
% path = ".";
% addpath(genpath(path));
%% Initiate MSG object (and RSAT for invKin)
experiment = MSG();
RECS = RSAT();

%% Pull target pose from .ply
% CANNED = true; % MUST BE false TO ACTUALLY USE VALUES
% TMSG_tgt = experiment.MSGplyGrabPose(CANNED);
% NOTE: the particular choice of x,y axes in the above does not matter --
% this will be accounted for in the later code

%% Find a reachable target pose for arm II (RECSB ARM2)
MA_fwd_relstowed = [45 90 -40 0 0 0];
theta_fwd_relstowed = RECS.motorAngle2theta(MA_fwd_relstowed);
T0_i = RECS.fwdKin(theta_fwd_relstowed, "stow"); % Absolute references for theta
T0_6 = T0_i(:,:,6);

% whichArm == "II";
whichRECSarm = 2;
TMSG_RECS = experiment.TMSG_RECSB;
TRECS_0 = RECS.arm2RECS(whichRECSarm);

% The below puts the tgt a little in front of the tip of the end effector
% [T0_6s,Nsol,T6_tgts] = tgtWtask2goal(obj,TRECS_tgt,whichArm,TASK)
gar3 = eye(4);
gar4 = 1;
[gar1,gar2,T6_tgts] = RECS.tgtWtask2goal(gar3,gar4,"move2pt"); % exploiting this function for now ***
% T6_tgt = [         0   -1.0000         0         0;
%                -1.0000         0         0         0;
%                      0         0   -1.0000   66.7200;
%                      0         0         0    1.0000];

% Simply take the first option for demo purposes
T6_tgt = T6_tgts(:,:,1);
% T6_tgt = eye(4);
TMSG_tgt = TMSG_RECS*TRECS_0*T0_6*T6_tgt;

% Display starting configuration
experiment.plotExperiment(RECS.theta0abs,theta_fwd_relstowed,TMSG_tgt)

%% Define relationship between target and goal frame of frame (6)
% Removes dependence on end effector
whichArm = "II"; % Only functioning arm
[T0_6s,Nsol] = experiment.MSGtgtWtask2goal(TMSG_tgt,whichArm,"move2pt");

%% Determine possible invKin sol'ns
% Initial reference is the stowed position, so we must change from the model reference 
theta_invss_relstowed = zeros(6,8,Nsol); % would additional solutions be trivial, i.e. just rotations of theta6?
THETAREF = "stow";
for nn = 1:Nsol
    T0_6 = T0_6s(:,:,nn);
    theta_invs_relstowed = RECS.invKin(T0_6,THETAREF);
    theta_invss_relstowed(:,:,nn) = theta_invs_relstowed;
end

%% Input initial motor position if other than stowed
% NOTE: all motor angles are relative to the stowed position
thetainit_relstowed = input("Input vector of current motor angles of arm II relative to the stowed position:");
thetainit_relstowed = reshape(thetainit_relstowed,6,1); % Ensure is a column vector

% Flatten index
Ntot = 8*Nsol;
theta_invss_relinit = zeros(6,Ntot);
for ii = 1:Nsol
    for jj = 1:8
        ind = jj+(ii-1)*8; % This will match the figure number
        theta_invss_relinit(:,ind) = theta_invss_relstowed(:,jj,ii) - thetainit_relstowed;
    end
end

%% PLOT
% Also prints which solutions DNE
% % experiment.plotMSG(false)
% % Test stowed
% theta_armI_relstowed = zeros(6,1);
% theta_armII_relstowed = zeros(6,1);
% experiment.plotExperiment(theta_armI_relstowed,theta_armII_relstowed,TMSG_tgt)

for ind = 1:Ntot
    theta_invs_relinit = theta_invss_relinit(:,ind);
    chknan = sum(isnan(theta_invs_relinit));
    if chknan == 0
        theta_armII_relstowed = theta_invs_relinit; % Only functioning arm %%%%%%%%%%%
        theta_armI_relstowed = zeros(6,1);
        experiment.plotExperiment(theta_armI_relstowed,theta_armII_relstowed,TMSG_tgt)
    else
        txt = strcat("Soln ", num2str(ind)," DNE");
        disp(txt)
    end
end
    

