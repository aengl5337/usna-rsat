clear all 
close all
format compact
clc

thetas_fwd = [NaN -90 0 45 0 0 0]; % index similarly to the rest of the D_H params
r0_6QR = zeros(3,1);
% lee=44.5;  % Measured to endpoint of end effector 
%            % (not grabbing centroid)


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

D_H = [a d alpha];

[T0_6,T] = forwardKin(thetas_fwd, D_H);

nMotors = 6;
figure(1)
hold on
colors = {'#EDB120','#D95319','#A2142F','#0072BD','#4DBEEE','#00FF00'};
T0_i = eye(4,4); % Initialize gradual transformation matrix from {i} to {0}
for joint=1:nMotors % Incrementally matrix-multiply all transformations together, from joint 6 to 1
    i = joint;
    T(:,:,i)
    T0_i = T0_i*T(:,:,i)
        
    P0_iorg = T0_i(1:3,4)  % Origin of {ith} frame in zeroth frame coordinates
    
    P0_ix = P0_iorg+T0_i(1:3,1);
    P0_iy = P0_iorg+T0_i(1:3,2);
    P0_iz = P0_iorg+T0_i(1:3,3);
    
    plot3(P0_iorg(1),P0_iorg(2),P0_iorg(3),'o','MarkerEdgeColor',colors{i},'MarkerFaceColor',colors{i})
    arrow(P0_iorg,P0_ix,'MarkerFaceColor',colors{i},'MarkerEdgeColor',colors{i})
    arrow(P0_iorg,P0_iy,'MarkerFaceColor',colors{i},'MarkerEdgeColor',colors{i})
    arrow(P0_iorg,P0_iz,'MarkerFaceColor',colors{i},'MarkerEdgeColor',colors{i})
    figure(1)
    pause(1)
end

%%DEBUG%%
targetPose = T0_6; % for the time being

% NOTE:
% This is a special-case inverse kinematics solution solution from D.
% Pieper as per Craig (pp129, ch4.6 in 2nd Edition) that is pertinent to
% the NSTAR robot arm.  A special-case solution for theta4 is used, playing
% on the fact that a1 equals ZERO.
% 
% INPUTS:
%
% targetPose = 
% [ [R0_QR] [P0_QR];
%   [0 0 0]  1      ]
%
% D_H = Denavit-Hartenberg link parameters as follows:
% [ [a] [d] [alpha] ]
% NOTE: the first row are the 0 frame parameters, so be careful when
% indexing
%
% r0_6QR = Can be thought of the grab margin [vector] from motor 6 origin to
% that of the QR code, i.e. special positioning before final approach (in
% base coords)
%
%
% OUTPUTS:
%
% thetas = vector of each joint angle corresponding to input points
%
    
    % GIVENS
    a = D_H(:,1);
a0 = a(1);
a1 = a(2);
a2 = a(3);
a3 = a(4);
a4 = a(5);
a5 = a(6);
a6 = a(7);


    d = D_H(:,2);
d0 = d(1);
d1 = d(2);
d2 = d(3);
d3 = d(4);
d4 = d(5);
d5 = d(6);
d6 = d(7);

%     d7 = d(7);
    
    alpha = D_H(:,3);
gamma0 = alpha(1);
alpha1 = alpha(2);
alpha2 = alpha(3);
alpha3 = alpha(4);
alpha4 = alpha(5);
alpha5 = alpha(6);
alpha6 = alpha(7);
    
%     ca0 = cosd(alpha0);
    ca1 = cosd(alpha1);
    ca2 = cosd(alpha2);
    ca3 = cosd(alpha3);
    ca4 = cosd(alpha4);
    ca5 = cosd(alpha5);
    ca6 = cosd(alpha6);

%     sa0 = sind(alpha0);
    sa1 = sind(alpha1);
    sa2 = sind(alpha2);
    sa3 = sind(alpha3);
    sa4 = sind(alpha4);
    sa5 = sind(alpha5);
    sa6 = sind(alpha6);
    
    
    % Extract the 4 possible orientations of the 6 frame from given pose
    R0_QR = targetPose(1:3,1:3);
    Xhat0_QR = R0_QR(:,1);
    Yhat0_QR = R0_QR(:,2); 
    Zhat0_QR = R0_QR(:,3);
    
    R0_6s = zeros(3,3,4);
%     R0_6s(:,:,1) = [ Yhat0_QR  Xhat0_QR -Zhat0_QR];
%     R0_6s(:,:,2) = [-Yhat0_QR -Xhat0_QR -Zhat0_QR];
%     R0_6s(:,:,3) = [ Xhat0_QR -Yhat0_QR -Zhat0_QR];
%     R0_6s(:,:,4) = [-Xhat0_QR  Yhat0_QR -Zhat0_QR];
    R0_6s(:,:,1) = [ Xhat0_QR Yhat0_QR  Zhat0_QR];
    R0_6s(:,:,2) = [-Xhat0_QR -Yhat0_QR Zhat0_QR];
    R0_6s(:,:,3) = [ Yhat0_QR -Xhat0_QR Zhat0_QR];
    R0_6s(:,:,4) = [-Yhat0_QR  Xhat0_QR Zhat0_QR];
    
    % Extract origins of motors 4 and 6 frame from the given pose
    P0_QR = targetPose(1:3,4);
    % P0_QR = float array, 3 element column vector of the cartesian coordinates
    % of the desired location of the *end effector/grabbing point*, with
    % respect to the base frame
    
    P0_6 = P0_QR-r0_6QR; % Location of the 6 origin relative to the QR code
                         % This concept could be generalized to include
                         % relative pose offsets too but will ignore for
                         % now*
    
    r0_46 = d6*(-Zhat0_QR); % This is the vector in base coords between the origins of motors 4 and 6
                            % (-Zhat0_QR) = Zhat0_6
    
    P0_4 = P0_6-r0_46; % Subtracting this from motor 6 origin yields motor 4 origin (in base coords)
    % P0_4 = float array, 3 element column vector of the cartesian coordinates
    % of the desired location of the "4ORG" as prescribed by Pieper in Craig, with
    % respect to the base frame

    r = norm(P0_4);
    
    x = P0_4(1);
    y = P0_4(2);
    z = P0_4(3);
    
    % ASSUMPTIONS
    theta4 = 0;               % This method makes this assumption
    
    % SOLUTION
    thetas = zeros(2,2,2,4,6);  % Make a container for all of the 192 potential solutions
                              % indexed by order of solution, (i,j,k,l,ntheta) or
                              % (theta3 no., theta2 no., theta1 no., R0_6 no., ntheta) (see loop indices below) 
    % ***CHECK THESE***
    A3 = (2*a2*a3 + 2*d2*(-d4*(sa2)*(sa3)));
    B3 = (2*a2*d4*(sa3) + 2*d2*(a3*(sa2)));
    C3 = -r + (a1^2+a2^2+a3^2) + (d2^2+d3^2+d4^2) + 2*d3*d4*(ca3) + 2*d2*(d4*(ca2)*(ca3)+d3*(ca2));
    
    theta3p = 2*atan2d((B3+sqrt(B3^2+(A3^2-C3^2))),(A3+C3));
    theta3n = 2*atan2d((B3+sqrt(B3^2-(A3^2-C3^2))),(A3+C3));
    
    theta3s = [theta3n,theta3p];
    %***ADD ERROR CONDITIONS***
    for i=1:numel(theta3s)
        theta3 = theta3s(i);
        s3 = sind(theta3);
        c3 = cosd(theta3);
        
        f1 = a3*(c3) + d4*(sa3)*(s3) + a2;
        f2 = a3*(ca2)*(s3) - d4*(ca2)*(sa3)*(c3) - d4*(sa2)*(ca3) - d3*(sa2);
        f3 = a3*(sa2)*(s3) - d4*(sa2)*(sa3)*(c3) + d4*(ca2)*(ca3) + d3*(ca2);
        
        k1 = f1;
        k2 = -f2;
        %k3 = f1^2 + f2^2 + f3^2 + a1^2 + d23^2 + 2*d23*f3;
        k4 = (f3+d2)*(ca1); %%TYPO?? no sin term**
        
        A2 = -k2;
        B2 = k1;
        C2 = (z-k4)/(sa1);
        theta2p = 2*atan2d((B2+sqrt(B2^2+(A2^2-C2^2))),(A2+C2));
        theta2n = 2*atan2d((B2+sqrt(B2^2-(A2^2-C2^2))),(A2+C2));
        theta2s = [theta2n,theta2p];
        %% do something with theta2s**
        for j=1:numel(theta2s)
            theta2 = theta2s(j);
            s2 = sind(theta2);
            c2 = cosd(theta2);
            
            g1 = f1*(c2) - f2*(s2) + a1;
            g2 = f1*(ca1)*(s2) + f2*(ca1)*(c2) - f3*(sa1) - d2*(sa1);
            g3 = f1*(sa1)*(s2) + f2*(sa1)*(c2) + f3*(ca1) + d2*(ca1);
            
            % **Using x coordinate
            A1 = g1;
            B1 = -g2;
            C1 = x;
            
            theta1p = 2*atan2d((B1+sqrt(B1^2+(A1^2-C1^2))),(A1+C1));
            theta1n = 2*atan2d((B1+sqrt(B1^2-(A1^2-C1^2))),(A1+C1));
            theta1s = [theta1n,theta1p];
            
            for k=1:numel(theta1s)
                theta1 = theta1s(k);
                
                for l=1:size(R0_6s,3)
                    R0_6 = R0_6s(:,:,l);
                    
                    R0_4 = planarRotation(theta4,'xy')*planarRotation(alpha3, 'yz')*planarRotation(theta3,'xy')*planarRotation(alpha2, 'yz')*planarRotation(theta2,'xy')*planarRotation(alpha1, 'yz')*planarRotation(theta1,'xy')*planarRotation(alpha0, 'yz');
                    Ra4 = planarRotation(alpha4,'yz');
                    R0_a4 = R0_4*Ra4;
                    B = (R0_a4)'*R0_6; % Orthogonal: inverse is transpose
                                    
                    theta5 = atan2d(B(1,3),-B(2,3));
                    theta6 = atan2d(-B(3,1),B(3,2));
                    
                    thetas(i,j,k,:) = [theta1, theta2, theta3, theta4, theta5, theta6];
                
                end
            end
        end
    end
% 
% % Print error between two methods
% thetas_residual = thetas_fwd - thetas_inv