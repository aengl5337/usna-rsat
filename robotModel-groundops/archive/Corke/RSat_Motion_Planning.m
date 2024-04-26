clear all
close all
format compact
clc
figind = 2;

%% RSat_NASAdemo_DEC21
% rev0: (Alec Engl, 05DEC21)
%      a. borrowed everything from NSTAR_ArmSim_script_Corke_rev3.m
%      b. Initial draft of the code for the NASA demo (Riley, there are commented instructions for finishing it near the top)
%      c. Has the RSat robot w non-spherical wrist D-H
%      d. Note to Riley: still need to finish up the part of the code where
%      the end effector moves from the central pose to two other poses a
%      small distance away (start at point annotated by '***')
% rev1: (Riley Cushing, 05DEC21)
%      a. It has the option of either hardcoding the transform for commanded pose 1 and 2 or allowing the user to enter the coordinates and pose into the command line.
% 	   b. The commented Transforms for commanded pose 1 and 2 have been confirmed to work, as have a variety of positions and poses near that range.
% 	   c. Current end effector pose in the hard coded values is aligned with the base coordinate frame.
% 	   d. Recording in armSim.mp4
% rev2: (Riley Cushing, 06DEC21)
% 			a. Altered rejected step limit value of ikine, allowing for larger movements to be accomplished w/ one ikine command instead of having to break it up into sequential movements.
% 			b. Also changed reference thetas of all ikine to correspond to the home position. Makes more sense now that large movements are more likely to succeed.
% rev3: (???, ???)
%           a. renamed from RSat_NASAdemo_DEC21 to RSat_Motion_Planning
%           b. Some other untracked changes
% rev4: (???, ???)
%           a. Some untracked changes
% Author(s):
% Alec Engl
% Riley Cushing

% Replace this with the directory that houses rtb, common, and smtb folders
corkeTBpath = 'C:\Users\Alec';
% corkeTBpath = 'C:\Users\m231362\Desktop\School\RSAT';
addpath(corkeTBpath);
addpath rtb common smtb

%% Set up robot
% Structure dimensions (mm)
l2 = 312.24;
l3 = 182.02;
l3_perp = 7.99;
l5 = 76.6;
lee=50;

% Angle limits for each joint (deg)
thetasLimRng = [0,255;        % **guess**
               -180,180;
               -180,0;
               -180,180;
               -123.95,34.29; % **revise**
               -180,180;
               -45,135];       % **guess**

% Link Lengths (mm)
a1=0;
a2=0;
a3=0;
a4=0;
a5=0;
a6=0;

% Link twist angles (deg)
alpha1=90;
alpha2=-90;
alpha3=-90;
alpha4=-90;
alpha5=90;
alpha6=0;

% Link offsets (mm)
d1=0;
d2=-l2;
d3=l3_perp;
d4=l3;
d5=0;
d6=l5+lee;

% Set initial thetas
theta1=0;
theta2=-90;
theta3=0;
theta4=0;
theta5=0;
theta6=0;

a=[a1, a2, a3, a4, a5, a6]';
d=[d1, d2, d3, d4, d5, d6]'; 
alpha=[alpha1,alpha2,alpha3,alpha4,alpha5,alpha6]';
theta_i = [theta1,theta2,theta3,theta4,theta5,theta6]';
sigma = zeros(6,1); % Indicating that all joints are '0', revolute

% Create D-H Table array
D_Hc = [(pi/180)*theta_i, d, a, (pi/180)*alpha, sigma]; % Where d is NOT offset (offset assumed zero)

% Loop for building robot
nLinks = 6;
for k=1:nLinks % NOTE: Must start with at least 3 joints
    for i=1:nLinks
        L(i) = Link(D_Hc(i,:),'standard'); % Setting params as standard
    end
    robot = SerialLink(L);
end

%% Move to a 'central' pose from start, then between 2 specified poses a small distance away
% NOTES: 
% 1. all theta vectors must be row vectors
% 2. all angles in degrees, distances in mm
numruns=input("Number of runs: ");
X=zeros([numruns,1]);
Y=zeros([numruns,1]);
Z=zeros([numruns,1]);
a=zeros([numruns,1]);
b=zeros([numruns,1]);
c=zeros([numruns,1]);
% Starting pose (once arm unfolds)
thetas_i(1,:) = [60 0 -120 0 0 0]; % (all angles in degrees), **Deploy position**

% thetas_i = [0 0 90 0 0 0];
% Tf_i = robot.fkine((pi/180)*thetas_i);\

figure(figind)
robot.plot((pi/180)*thetas_i(1,:))
hold on


% Find a 'Center' pose (Tf_c) using fwd kin of an arbitrary theta vector
% That way, we know we can reach this point
thetas_fwdc = [60 0 -120 0 0 0]; %Deployed Pose
Tf_c = robot.fkine((pi/180)*thetas_fwdc);
thetas_invcentral = 180/pi*robot.ikine(Tf_c,thetas_i(1,:));
thetas_invcentral=wrapToPi(thetas_invcentral*pi/180)*180/pi; %Wrap to avoid large rotation and previous link colision, also helps improve ikin preformance

% %% Unnecessary Code block, ignore for now
% % Animated plotting
% % Have to make a separate loop, once robot constructed to animate
% l=20; % Number of discrete motion steps
% thetas_invc_j = thetas_i; % Incrementally increasing theta vector
% for k=1:nLinks
%     del = (thetas_invc(k)-thetas_i(k))/l;  % Take overall delta between starting and end angles and divide them up into l steps
%     for j=1:l
%         thetas_invc_j(k) = thetas_invc_j(k) + del;
%         robot.plot((pi/180)*thetas_invc_j)
%     end
% end

% *** At this point, the robot is in its Deployed pose
clear Tf_cmd1;
Tf_cmd1=diag([1 1 1 1]); %init cmd transfrom cell array
Tf_cmd1(:,:,numruns)=reshape(zeros(4),4,4);
Tf_cmd=num2cell(Tf_cmd1,[1 2]);
Tf_cmd{1}=robot.fkine((pi*180)*thetas_i(1,:));

for i=1:numruns
    X(i)=input("Input X Cord (mm): "); %User entered position and pose Xcam=-XRobot
    Y(i)=input("Input Y Cord (mm): "); % Ycam=-ZRobot
    Z(i)=input("Input Z Cord (mm): "); % ZCam =-YRobot
    a(i)=input("Input X Angle (deg): ");
    b(i)=input("Input Y Angle (deg): ");
    c(i)=input("Input Z Angle (deg): ");
    Tf_cmd{i}=Tx(-X(i))*Ty(-Z(i))*Tz(-Y(i))*Rx(a(i)*pi/180)*Ry(b(i)*pi/180)*Rz(c(i)*pi/180); %Transform calculator
    thetas_invcmd1(i,:)=180/pi*robot.ikine(Tf_cmd{i},thetas_invcentral,'ilimit',1000,'rlimit',500); %rlimit increases the rejected step limit, allowing for larger movements to be accomplished
    thetas_invcmd1(i,:)=wrapToPi(thetas_invcmd1(i,:)*pi/180)*180/pi %Prevents colision with previous link by avoiding rotation larger than 180 deg in either direction
    % Insert error check, break if thetas is empty
    l=20; % Number of discrete motion steps
    thetas_invcmd1_j = thetas_invcentral; % Incrementally increasing theta vector
    for k=1:nLinks
        del = (thetas_invcmd1(k)-thetas_invcentral(k))/l;  % Take overall delta between starting and end angles and divide them up into l steps
        for j=1:l
            thetas_invcmd1_j(k) = thetas_invcmd1_j(k) + del;
            robot.plot((pi/180)*thetas_invcmd1_j)
        end
    end
end
%%
%Next 4 runs done with 0 X and 0 Z angle
%Arm 1 0 deg Y = Point to front glass
%Arm 1 90 deg Y = Point to right
%Arm 1 -90 deg Y = Point to left
%Arm 1 180 deg Y = Point to back wall

%Next 4 runs done with 0 Z and 0 Y angle
%Arm 1 90 deg X = Point to robot
%Arm 1 -90 deg X = Point away from robot








thetatest=[0 90 0 90 0 0];
robot.plot((pi/180)*thetatest)






% X=input("Input X Cord (mm): "); %User entered cordinate values
% Y=input("Input Y Cord (mm): ");
% Z=input("Input Z Cord (mm): ");
% a=input("Input X Angle (rad): ");
% b=input("Input Y Angle (rad): ");
% c=input("Input Z Angle (rad): ");
% Tf_cmd2=Tx(X)*Ty(Y)*Tz(Z)*Rx(a)*Ry(b)*Rz(c);
% thetas_invcmd2=180/pi*robot.ikine(Tf_cmd2,thetas_invc,'rlimit',500); %rlimit increases the rejected step limit, allowing for larger movements to be accomplished
% thetas_invcmd2=wrapToPi(thetas_invcmd2*pi/180)*180/pi;
% 
% l=20; % Number of discrete motion steps
% thetas_invcmd2_j = thetas_invcmd1; % Incrementally increasing theta vector
% for k=1:nLinks
%     del = (thetas_invcmd2(k)-thetas_invcmd1(k))/l;  % Take overall delta between starting and end angles and divide them up into l steps
%     for j=1:l
%         thetas_invcmd2_j(k) = thetas_invcmd2_j(k) + del;
%         robot.plot((pi/180)*thetas_invcmd2_j)
%     end
% end
% Robot is in commanded Pose 2
%% TRASH:


% 
% 
% 
% % Below I started to write some stuff on trajectory generation but stopped
% 
% % Make trajectory between two points over a specified number of fixed intervals
% a = 200*ones(3,1);
% b = a + [0 0 100]';
% 
% Ta = transl(a);
% Tb = transl(b);
% 
% q0 = zeros(1,6); % Specify (arb) initial position for inv kinematics to work on
% qa = 180/pi*robot.ikunc(Ta,q0); % 'mask',[1 1 1 0 0 0]Masking allows us to ignore error at certain DoFs
% qb = 180/pi*robot.ikunc(Ta,q0);
% 
% n=100 % Number of intervals
% s = linspace(0,1,n); % Parameterizes along trajectory via s, think of this as a pseudo-time variable
%                        % Since we are using our own scheme for actually
%                        % controlling the joints, we are ignoring the
%                        % scale of the velocity/acceleration outputs from this function
% 
% Q = jtraj(pi/180*qa,pi/180*qb,s); % Generates trajectory in joint angles
% Ttraj = robot.fkine(Q); % Converts trajectory into series of transforms 
% 
% for i=1:n
%     Ti = Ttraj(i);
%     pi = transl(Ti); % Pulls translation component out of matrix
%     x(i) = pi(1);
%     y(i) = pi(2);
%     z(i) = pi(3);
% end
% 
% figure(3)
% robot.plot(Q)
% hold on
% plot3(x,y,z,'Color',[1 0 0],'LineWidth',3)