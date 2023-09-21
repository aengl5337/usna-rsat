clear all 
close all
format compact
clc

addpath('C:\Users\Alec');
addpath rtb common smtb
% cd G:\My Drive\NSTAR\NSTAR_IEEE_Paper\MATLAB

% Link Lengths (mm)
a1=0;
a2=0;
a3=0;
a4=-7.99;
a5=0;
a6=0;

% Link twist angles (deg)
%gamma1=-90; % This one rotates around Y axis to be consistent with the illustrations' base frame
alpha1=0;
alpha2=90;
alpha3=-90;
alpha4=-90;
alpha5=90;
alpha6=-90;
% alpha6=90;

% Link offsets (mm)
d1=0;
d2=0;
d3=-312.24;
d4=0;
d5=182.02;
d6=0;

theta1=0;
theta2=0;
theta3=-90;
theta4=0;
theta5=0;
theta6=0;

a=[a1, a2, a3, a4, a5, a6]';
d=[d1, d2, d3, d4, d5, d6]'; 
% a = [0,ones(1,5),NaN]';
% d = [NaN,1,zeros(1,5)]';
alpha=[alpha1,alpha2,alpha3,alpha4,alpha5,alpha6]';
theta_i = [theta1,theta2,theta3,theta4,theta5,theta6]';

sigma = zeros(6,1); % Indicating that all joints are '0', revolute
D_Hc = [(pi/180)*theta_i, d, a, (pi/180)*alpha, sigma]; % Where d is NOT offset (offset assumed zero)


nLinks = 6;

% Forward Kinematics
l=20;
del = 90/l;
thetas_fwd = zeros(1,6);
for k=1:nLinks % Must start with at least 3 joints
    for i=1:nLinks
        L(i) = Link(D_Hc(i,:),'standard');
    end
    robot = SerialLink(L);
    
    figure(1)
% %         zlim([-10,10000])
%     hold on
%     for j=1:l
%         thetas_fwd(k) = thetas_fwd(k) + del;
%         robot.plot((pi/180)*thetas_fwd)
%     end
end

% thetas_fwd = [90 45 30 30 -30 0];
% robot.plot((pi/180)*thetas_fwd)
% Tf_cmd = robot.fkine((pi/180)*thetas_fwd);
% 
% % Inverse Kinematics
% % thetas_inv = 180/pi*robot.ikine6s(Tf_cmd,thetas_fwd) % For analytical solution
% % of spherical wristed robots
% % thetas_inv = 180/pi*robot.ikine(Tf_cmd);
% % thetas_inv = 180/pi*robot.ikine(Tf_cmd,(pi/180)*thetas_fwd); % Specify starting thetas
% thetas_inv = 180/pi*robot.ikine(Tf_cmd,thetas_fwd,[1 1 1 0 0 0]); 
% % thetas_inv = 180/pi*robot.ikine(Tf_cmd,[0 0 0 0 0 0],[0 0 1 0 0 0]);
% % thetas_inv = 180/pi*robot.ikunc(Tf_cmd,thetas_fwd);
% thetas_inv = 180/pi*robot.ikunc(Tf_cmd,[0 0 0 0 0 0]);
% 
% % Specify starting thetas
% % and mask to ensure that only position error is considered in solution
% % (first three elements of state vector)
% 
% figure(2)
% robot.plot((pi/180)*thetas_inv)
% 
% % Check solution
% Ti_check = robot.fkine((pi/180)*thetas_inv);
% T_error = abs(Tf_cmd - Ti_check);
% 

% Make trajectory between two points over a specified number of fixed intervals
a = 200*ones(3,1);
b = a + [0 0 100]';

Ta = transl(a);
Tb = transl(b);

q0 = zeros(1,6); % Specify (arb) initial position for inv kinematics to work on
qa = 180/pi*robot.ikunc(Ta,q0); % 'mask',[1 1 1 0 0 0]Masking allows us to ignore error at certain DoFs
qb = 180/pi*robot.ikunc(Ta,q0);

n=100 % Number of intervals
s = linspace(0,1,n); % Parameterizes along trajectory via s, think of this as a pseudo-time variable
                       % Since we are using our own scheme for actually
                       % controlling the joints, we are ignoring the
                       % scale of the velocity/acceleration outputs from this function

Q = jtraj(pi/180*qa,pi/180*qb,s); % Generates trajectory in joint angles
Ttraj = robot.fkine(Q); % Converts trajectory into series of transforms 

for i=1:n
    Ti = Ttraj(i);
    pi = transl(Ti); % Pulls translation component out of matrix
    x(i) = pi(1);
    y(i) = pi(2);
    z(i) = pi(3);
end

figure(3)
robot.plot(Q)
hold on
plot3(x,y,z,'Color',[1 0 0],'LineWidth',3)

