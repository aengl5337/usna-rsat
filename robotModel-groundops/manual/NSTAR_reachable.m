clear all 
close all
format compact
clc

%% NSTAR_reachable.m
%{

TODO:
() = incomplete, (X) = complete
+ = addition, - = fix/debug

() -Offload functionality into plotReachable
%}

%% Some constants
path = "G:\My Drive\RSat master\softwareWorkingDir\USNA_NSTAR_RSat\robotics\archive\manual";
addpath(path);

% Lengths (mm)
l2 = 312.24;
l4 = 182.02; % l3
l4p = 7.99; % l3p
l6 = 76.6; %l5
% lee=44.5;  % Measured to endpoint of end effector 
%            % (not grabbing centroid)

% Denavit-Hartenberg Parameters (Modified -- Craig*)
% Link twist angles (alpha, deg)
alpha = 90*[0 1 1 1 1 -1]'; % i-1
% Link Lengths (a, mm)
a = [0 0 0 l4p 0 0]'; % i-1
% (Intrinsic) Joint angles (theta, deg)
theta0ext = 90*[0 1 0 2 0 0]'; % i; these correspond to the 'extended' position
theta0 = theta0ext;
% Theta limits as measured from the extended position
delThetaLimRng = [-255,0;        
               -180,180;
               -180,0;
               -180,180;
               -123.95,34.29; % referenced from previous arm's centerline (i.e. extended position)
               -180,180;
               -45,135];       % **guess**, note this 7th entry is for the end effector motor!

% Link offsets (d, mm)
d = [0 -l2 0 l4 0 l6]'; % i

D_H = [alpha,a,theta0,d];

D_Hwrist = D_H;
D_Hwrist(1:3,:) = zeros(3,4); % zeroed out to ensure wrist is at origin of plot display


%% SAMPLE WRIST POSE VIA FORWARD KINEMATICS
% % PROPAGATE ROBOT ARM OUT TO ORIGIN OF (4) ONLY
inputMode ="abs";
q_types = zeros(6,1); % specify 0 for each revolute joint, and 1 for each prismatic one
sc = 100;
% COLOR CODE AXES
c1 = '#EDB120'; % Marigold
c2 = '#D95319'; % Orange
c3 = '#A2142F'; % Red?
c4 = '#0072BD'; % Dark blue
c5 = '#4DBEEE'; % Light blue
c6 = '#00FF00'; % Lime green
ctgt = '#777777'; % Gray
ctgt2 = '#888888';

% Sample wrist position
% Only need to sample across theta1 first, theta3 last
narm = 3;
Nper = 5; % is cubed 
Ntot = Nper^3;
thetaSamps = zeros(narm,Nper);
for nn = 1:narm
    thetaSamps(nn,:) = linspace(delThetaLimRng(nn,1),delThetaLimRng(nn,2),Nper);
end

% Propagate sample and plot
theta1s = thetaSamps(1,:);
theta2s = thetaSamps(2,:);
theta3s = thetaSamps(3,:);

figure()
hold on
for ii = 1:Nper
    theta1 = theta1s(ii);
    for jj = 1:Nper
        theta2 = theta2s(jj);
        for kk = 1:Nper
            ind = kk + (jj-1)*Nper + (ii-1)*Nper^2;
            display(ind)
            theta3 = theta3s(kk);
            thetaSamp = [theta1,theta2,theta3,0,0,0]; % set 4,5,6 to zero
            
            T0_i = forwardKin(thetaSamp, q_types, D_H, "mod",inputMode); % Using "abs" for consistency
            T0_4 = T0_i(:,:,4);
            zhat0_4 = T0_4(1:3,3); % Axis of 4th motor
            P0_4 = T0_4(1:3,4); % origin of 4th frame
            % SLOW OPTION:
            plotRobotAxes(T0_i(:,:,1:4),NaN,true) % Plot the whole robot arm
            % MUCH FASTER OPTION:
%             plotAxes(T0_4,sc/2,c4,false,false,true) % Plot only the origin and z axis of the 4th frame origin
        end
    end
end

% ***Specify labelling, legend here
hold off

%% SAMPLE END EFFECTOR POSE VIA FORWARD KINEMATICS
% % PROPAGATE ROBOT ARM OUT FROM (4) TO (6) ONLY
% Sample wrist position
% Only need to sample across theta4 first, theta6 last
nwrist = 3;
Nper = 5; % is cubed 
Ntot = Nper^3;
thetaSamps2 = zeros(nwrist,Nper);
for nn = 1:nwrist
    thetaSamps2(nn,:) = linspace(delThetaLimRng(nn,1),delThetaLimRng(nn,2),Nper);
end

% Propagate sample and plot
theta4s = thetaSamps2(1,:);
theta5s = thetaSamps2(2,:);
theta6s = thetaSamps2(3,:);

figure()
hold on
for ii = 1:Nper
    theta4 = theta4s(ii);
    for jj = 1:Nper
        theta5 = theta5s(jj);
        for kk = 1:Nper
            ind = kk + (jj-1)*Nper + (ii-1)*Nper^2;
            display(ind)
            theta6 = theta6s(kk);
            thetaSamp = [0,0,0,theta4,theta5,theta6]; % set 1,2,3 to zero
            
            T0_i = forwardKin(thetaSamp, q_types, D_Hwrist, "mod",inputMode); % Using "abs" for consistency
            T0_6 = T0_i(:,:,6);
            zhat0_6 = T0_6(1:3,3); % Axis of 6th motor
            P0_6 = T0_6(1:3,4); % origin of 6th frame
            % SLOW OPTION:
            plotRobotAxes(T0_i(:,:,4:6),NaN,true) % Plot the whole robot arm
            % MUCH FASTER OPTION:
%             plotAxes(T0_6,sc/2,c6,false,false,true) % Plot only the origin and z axis of the 6th frame origin
        end
    end
end
% ***Specify labelling, legend here
hold off