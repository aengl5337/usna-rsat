clear all 
close all
format compact
clc

%{
modDHtest.m

Demonstrator script

%}

path = "G:\My Drive\RSat master\softwareWorkingDir\USNA_NSTAR_RSat\robotics\archive\manual";
addpath(path);
%% Test 1 -- fig 3.13right pg82
% Denavit-Hartenberg Parameters (Modified -- Craig*)
% Link twist angles (alpha, deg)
alpha = [0 90 0]'; % i-1
% Link Lengths (a, mm)
a = [0 0 100]'; % i-1
% Link twist angles (theta, deg)
theta = [0 90 0]'; % i
% Link twist angles (d, mm)
d = [0 -100 0]'; % i

D_H1 = [alpha,a,theta,d];


% FORWARD KINEMATICS
q_fwd = [0 0 0]';
N = numel(q_fwd);
% lee=44.5;  % Measured to endpoint of end effector 
%            % (not grabbing centroid)

T0_i = forwardKin(q_fwd, zeros(N,1), D_H1, "mod");
plotRobotAxes(T0_i)
