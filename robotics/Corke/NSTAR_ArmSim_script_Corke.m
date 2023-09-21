clear all 
close all
format compact
clc

% Replace this with the directory that houses rtb, common, and smtb folders
corkeTBpath = 'C:\Users\Alec';
addpath(corkeTBpath);
addpath rtb common smtb

% Link Lengths (mm)
a1=0;
a2=0;
a3=0;
a4=-7.99;
a5=0;
a6=0;

% Link twist angles (deg)
alpha1=0;
alpha2=90;
alpha3=-90;
alpha4=-90;
alpha5=90;
alpha6=-90;

% Link offsets (mm)
d1=0;
d2=0;
d3=-312.24;
d4=0;
d5=182.02;
d6=0;

% Set initial thetas
theta1=0;
theta2=0;
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

% Inner loop is for building robot, outer loop is for plotting the motion
% sequentially
l=20; % Number of discrete motion steps
thetas_fwd = [90 45 30 30 -30 0];
thetas_fwd_j = theta_i;
nLinks = 6;
for k=1:nLinks % NOTE: Must start with at least 3 joints
    for i=1:nLinks
        L(i) = Link(D_Hc(i,:),'standard'); % Setting params as standard
    end
    robot = SerialLink(L);
    
    % **Will make motion planning more complicated than simply iterating
    % through from shoulder to wrist**
    figure(1)
    hold on
    del = thetas_fwd(k)/l;
    for j=1:l
        thetas_fwd_j(k) = thetas_fwd_j(k) + del;
        robot.plot((pi/180)*thetas_fwd_j)
    end
end

% Forward Kinematics (plotting above)
Tf_cmd = robot.fkine((pi/180)*thetas_fwd);

% Inverse Kinematics
% Below is the graveyard of previous attempts, most of which won't converge
% % thetas_inv = 180/pi*robot.ikine6s(Tf_cmd,thetas_fwd) % For analytical solution
% % of spherical wristed robots
% % thetas_inv = 180/pi*robot.ikine(Tf_cmd);
% % thetas_inv = 180/pi*robot.ikine(Tf_cmd,(pi/180)*thetas_fwd); % Specify starting thetas
% thetas_inv = 180/pi*robot.ikine(Tf_cmd,thetas_fwd,[1 1 1 0 0 0]); 
% % thetas_inv = 180/pi*robot.ikine(Tf_cmd,[0 0 0 0 0 0],[0 0 1 0 0 0]);
% % thetas_inv = 180/pi*robot.ikunc(Tf_cmd,thetas_fwd);

thetas_inv = 180/pi*robot.ikunc(Tf_cmd,theta_i);  % This one works! 
% Note that we specify 'starting thetas' (theta_i)
% Syntax:
%SerialLink.IKUNC Inverse manipulator by optimization without joint limits
%
% Q = R.ikunc(T, OPTIONS) are the joint coordinates (1xN) corresponding to
% the robot end-effector pose T which is an SE3 object or homogenenous
% transform matrix (4x4), and N is the number of robot joints. OPTIONS is
% an optional list of name/value pairs than can be passed to fminunc.
%
% Q = robot.ikunc(T, Q0, OPTIONS) as above but specify the
% initial joint coordinates Q0 used for the minimisation.
%
% [Q,ERR] = robot.ikunc(T,...) as above but also returns ERR which is the
% scalar final value of the objective function.
%
% [Q,ERR,EXITFLAG] = robot.ikunc(T,...) as above but also returns the
% status EXITFLAG from fminunc.
%
% [Q,ERR,EXITFLAG,OUTPUT] = robot.ikunc(T,...) as above but also returns the
% structure OUTPUT from fminunc which contains details about the optimization.
%
% Trajectory operation::
%
% In all cases if T is a vector of SE3 objects (1xM) or a homogeneous transform
% sequence (4x4xM) then returns the joint coordinates corresponding to
% each of the transforms in the sequence.  Q is MxN where N is the number
% of robot joints. The initial estimate of Q for each time step is taken as
% the solution from the previous time step.
%
% ERR and EXITFLAG are also Mx1 and indicate the results of optimisation
% for the corresponding trajectory step.
%
% Notes::
% - Requires fminunc from the MATLAB Optimization Toolbox.
% - Joint limits are not considered in this solution.
% - Can be used for robots with arbitrary degrees of freedom.
% - In the case of multiple feasible solutions, the solution returned
%   depends on the initial choice of Q0
% - Works by minimizing the error between the forward kinematics of the
%   joint angle solution and the end-effector frame as an optimisation.
%   The objective function (error) is described as:
%           sumsqr( (inv(T)*robot.fkine(q) - eye(4)) * omega )
%   Where omega is some gain matrix, currently not modifiable.
%
% Author::
% Bryan Moutrie

figure(2)
robot.plot((pi/180)*thetas_inv)

% Check solution
Ti_check = robot.fkine((pi/180)*thetas_inv);
T_error = abs(Tf_cmd - Ti_check);


% Below I started to write some stuff on trajectory generation but stopped

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

