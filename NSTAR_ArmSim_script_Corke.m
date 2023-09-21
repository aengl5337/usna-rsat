clear all 
close all
format compact
clc

% Link Lengths (mm) from i to i+1
a0=0;
a1=0;
a2=0;
a3=-7.99;
a4=0;
a5=0;
a6=NaN;
% a6=0;

% Link twist angles (deg) **make sure rotation CW/CCW sense is correct
gamma0=-90; % This one rotates around Y axis to be consistent with the illustrations' base frame
alpha1=90;
alpha2=-90;
alpha3=-90;
alpha4=90;
alpha5=-90;
alpha6=NaN;
% alpha6=90;

% Link offsets (mm)* from i-1 to i
d0=NaN;
d1=0;
d2=312.24;
d3=0;
d4=-182.02;
d5=0;
d6=-76.6;

theta0=NaN;
theta1=0;
theta2=-90;
theta3=0;
theta4=0;
theta5=0;
theta6=0;

a=[a0, a1, a2, a3, a4, a5, a6]';
d=[d0, d1, d2, d3, d4, d5, d6]'; 
% a = [0,ones(1,5),NaN]';
% d = [NaN,1,zeros(1,5)]';
alpha=[gamma0, alpha1,alpha2,alpha3,alpha4,alpha5,alpha6]';
theta_i = [theta0,theta1,theta2,theta3,theta4,theta5,theta6]';

sigma = zeros(7,1); % Indicating that all joints are '0', revolute
D = zeros(7,1); % Artifact for prismatic joints, ignore
% D_Hc = [(pi/180)*theta_i(2:7), D(1:6), 100*a(1:6), (pi/180)*alpha(1:6), sigma(1:6), 100*d(2:7)]; % Arranged specially for Link() syntax, joint assumed revolute and OFFSET=0
% D_Hc = [(pi/180)*theta_i(2:7), D(2:7), 100*a(2:7), (pi/180)*alpha(2:7), sigma(2:7), 100*d(2:7)]; % Where d is offset, and D_pris is a placeholder
D_Hc(:,:,1) = [(pi/180)*theta_i(2:7), 10*d(2:7), a(1:6), (pi/180)*alpha(1:6), sigma(2:7)]; % Where d is NOT offset (offset assumed zero)
D_Hc(:,:,2) = [(pi/180)*theta_i(2:7), 10*d(2:7), a(2:7), (pi/180)*alpha(2:7), sigma(2:7)]; % Where d is NOT offset (offset assumed zero)
% D_Hc = [(pi/180)*theta_i, D, 100*a, (pi/180)*alpha, sigma, 100*d]; % Where d is offset, and D_pris is a placeholder

nLinks = 6;
l=20;
del = 90/l;
thetas_fwd = zeros(1,7);
for n = 1:1
    for k=3:nLinks % Must start with at least 3 joints
        for i=1:(nLinks-(n-1))
%             L(i) = Link(D_Hc(i,:,n),'modified');
            L(i) = Link(D_Hc(i,:,n),'standard');
        end
        figure(n)
%         zlim([-10,10000])
        hold on
        robot = SerialLink(L);
        for j=1:l
            thetas_fwd(k) = thetas_fwd(k) + del;
            robot.plot((pi/180)*thetas_fwd(2:(8-n)))
        end
    end
    hold off
    clear L
end

% Inverse Kinematics
% p0_6 = [
% T = [ eye(3,3) p0_6;
%       0 0 0    1];

  

% thetas_rev = 