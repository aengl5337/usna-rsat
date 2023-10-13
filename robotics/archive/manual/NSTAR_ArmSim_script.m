clear all 
close all
format compact
clc

%{
NSTAR_ArmSim_script.m

REV0-????
REV1-30SEP-12OCT23, Alec Engl (aengl5337@gmail.com)
Revamped&checked math, D-H parameters,etc
Arm now plots correctly (i.e. D-H params correct)
Inverse Kinematics now provides a seemingly-meaningful answer, but still differs from expected 


TODO:
() = incomplete, (X) = complete
+ = addition, - = fix/debug

() - Ensure frame zero transform input correctly (and ensure changes
according to which arm) -- for now this assumes 0 offset or rotation
() - CHANGE NAME
() + Write instructions (modified D-H params, etc)
(~) + Display sim
() + Migrate into python/speed up the math
() - Compare with other methods
() + Cite sources (*)
(X) - arrow display is super wonky... color arrows
(X) - why is arm plotting in x direction now?
() -how to work with git checkout?
(X) + Draw out rudimentary RSat geometry in plotter to better visualize
(X) + Make legend for each joint
() - Ensure intrinsic thetas are being subtracted from inverse kinematics
() - ***ADD ERROR CONDITIONS*** (NOTE: atan2d(3,0) = +90deg)
() + OOP function for theta sub sol
() + general joint frame transformation functions
() + Generalize end effector transformation as another fixed transformation
-- refine conventions
() - Clean up notation (i.e. is P a 3d vector or 4d one?
() - Is using atan2 for theta4,5 actually useful?

%}

%% Some constants
path = "G:\My Drive\RSat master\softwareWorkingDir\USNA_NSTAR_RSat\robotics\archive\manual";
addpath(path);

% Lengths (mm)
l2 = 312.24;
l4 = 182.02; % l3
l4p = 7.99; % l3p
l6 = 76.6; %l5

% Denavit-Hartenberg Parameters (Modified -- Craig*)
% Link twist angles (alpha, deg)
alpha = 90*[0 1 1 1 1 -1]'; % i-1
% Link Lengths (a, mm)
a = [0 0 0 l4p 0 0]'; % i-1
% Link twist angles (theta, deg)
theta = 90*[0 1 0 2 0 0]'; % i
% Link twist angles (d, mm)
d = [0 -l2 0 l4 0 l6]'; % i

D_H = [alpha,a,theta,d];


%% FORWARD KINEMATICS
% q_fwd = [-90 0 45 0 0 0]'; % index similarly to the rest of the D_H params, add 0 entry
q_fwd = [0 0 0 0 0 0]';
N = numel(q_fwd);
r0_6QR = zeros(3,1); % What would be more valuable is offset in the end effector's coordinates
% lee=44.5;  % Measured to endpoint of end effector 
%            % (not grabbing centroid)
q_types = zeros(N,1); % specify 0 for each joint (revolute)
T0_i = forwardKin(q_fwd, q_types, D_H, "mod");

% Plot
T0_6 = T0_i(:,:,6); % Joint 6 (end effector) pose in 0 frame
r0_6QR = zeros(3,1); % relative offset

% NOTE: since motor 6's axis points outwards through the wrist, must
% negate it to get an outward facing normal for the QR code
R0_6 = T0_6(1:3,1:3); % (eventually this will be reversed, as QR pose will be the input)
Xhat0_6 =  R0_6(:,1);
Yhat0_6 =  R0_6(:,2);
Zhat0_6 =  R0_6(:,3);

Xhat0_QR = -R0_6(:,1);
Yhat0_QR =  R0_6(:,2);
Zhat0_QR = -R0_6(:,3);
R0_QR = [Xhat0_QR,Yhat0_QR,Zhat0_QR];

P0_6 = T0_6(1:3,4);
P0_QR = P0_6+r0_6QR; % Compensate for relative offset


T0_QR = [R0_QR,   P0_QR;
         zeros(1,3), 1];

plotRobotAxes(T0_i,T0_QR)

%% INVERSE KINEMATICS

% INPUT: D_H, T0_tgt, offset, q_types,
T0_tgt = T0_QR;
r0_6tgt = r0_6QR;

% Transform from target frame to end effector frame (i.e. frame (6))
% (The following code needs generalization)
P0_tgt = T0_tgt(1:3,4);
    % P0_QR = float array, 3 element column vector of the cartesian coordinates
    % of the desired location of the *end effector/grabbing point*, with
    % respect to the base frame
P0_6 = P0_tgt-r0_6tgt; % Location of the 6 origin relative to the QR code
                         % This concept could be generalized to include
                         % relative pose offsets too but will ignore for
                         % now*
                         
% *Redefine R0_6 here when migrate to function*
% R0_6 = T0_tgt(1:3,1:3); % No rotational offset
% % Pull (6) unit vectors in coordinates of (0)
% Xhat0_6 = R0_6(:,1);
% % Yhat0_6 = R0_6(:,2);
% Zhat0_6 = R0_6(:,3);


% %%DEBUG%%
    % GIVENS
    alpha = D_H(:,1);
    alpha0 = alpha(1);
    alpha1 = alpha(2);
    alpha2 = alpha(3);
    alpha3 = alpha(4);
    alpha4 = alpha(5);
    alpha5 = alpha(6);
    
    a = D_H(:,2);
    a0 = a(1);
    a1 = a(2);
    a2 = a(3);
    a3 = a(4);
    a4 = a(5);
    a5 = a(6);

    d = D_H(:,4);
    d1 = d(1);
    d2 = d(2);
    d3 = d(3);
    d4 = d(4);
    d5 = d(5);
    d6 = d(6);
    
%     ca0 = cosd(alpha0);
    ca1 = cosd(alpha1);
    ca2 = cosd(alpha2);
    ca3 = cosd(alpha3);
    ca4 = cosd(alpha4);
    ca5 = cosd(alpha5);

%     sa0 = sind(alpha0);
    sa1 = sind(alpha1);
    sa2 = sind(alpha2);
    sa3 = sind(alpha3);
    sa4 = sind(alpha4);
    sa5 = sind(alpha5);
    
%     % Extract the 4 possible orientations of the 6 frame from given pose
%     % NOTE: since motor 6's axis points outwards through the wrist, mist
%     % negate it to get an outward facing normal for the QR code
%     R0_QR = targetPose(1:3,1:3); % (eventually this will be reversed, as QR pose will be the input)
%     Xhat0_QR = -R0_QR(:,1);
%     Yhat0_QR =  R0_QR(:,2);
%     Zhat0_QR = -R0_QR(:,3);
%     
%     R0_6s = zeros(3,3,4);
% %     R0_6s(:,:,1) = [ Yhat0_QR  Xhat0_QR -Zhat0_QR];
% %     R0_6s(:,:,2) = [-Yhat0_QR -Xhat0_QR -Zhat0_QR];
% %     R0_6s(:,:,3) = [ Xhat0_QR -Yhat0_QR -Zhat0_QR];
% %     R0_6s(:,:,4) = [-Xhat0_QR  Yhat0_QR -Zhat0_QR];
%     R0_6s(:,:,1) = [ Xhat0_QR Yhat0_QR  Zhat0_QR];
%     R0_6s(:,:,2) = [-Xhat0_QR -Yhat0_QR Zhat0_QR];
%     R0_6s(:,:,3) = [ Yhat0_QR -Xhat0_QR Zhat0_QR];
%     R0_6s(:,:,4) = [-Yhat0_QR  Xhat0_QR Zhat0_QR];
    
    % Extract origins of motors 4 and 6 frame from the given pose
    r0_46 = d6*(Zhat0_6); % This is the vector in base coords between the origins of motors 4 and 6
                            % (-Zhat0_QR) = Zhat0_6
                            % *Generalize this to the caseswhere Zhat0_QR != Zhat0_6, don't cheat and use d6!
    
    P0_4 = P0_6-r0_46; % Subtracting this from motor 6 origin yields motor 4 origin (in base coords)
    % P0_4 = float array, 3 element column vector of the cartesian coordinates
    % of the desired location of the "4ORG" as prescribed by Pieper in Craig, with
    % respect to the base frame

    r = norm(P0_4)^2; % The norm^2 of P0_4ORG (p130 Craig)
    
    % Decompose components of wrist position
    xw = P0_4(1);
    yw = P0_4(2);
    zw = P0_4(3);
    
    % ASSUMPTIONS
%     theta4 = 0;               % This method makes this assumption
    
    % SOLUTION
    thetas = zeros(2,2,2,4,6);  % Make a container for all of the 192 potential solutions
                              % indexed by order of solution, (i,j,k,l,ntheta) or
                              % (theta3 no., theta2 no., theta1 no., R0_6 no., ntheta) (see loop indices below) 
    % CHECKED with pieperInvKin_symwork.m
    A3 = (2*a2*a3 - 2*d2*d4*(sa2)*(sa3));
    B3 = (2*a2*d4*(sa3) + 2*a3*d2*(sa2));
    C3 = -r + (a1^2+a2^2+a3^2) + (d2^2+d3^2+d4^2)+ d2*d3*(ca2) + 2*d3*d4*(ca3) + 2*d2*d4*(ca2)*(ca3);
    
    % NOTE: typo in book corrected here
    theta3p = 2*atan2d((B3+sqrt(B3^2+(A3^2-C3^2))),(A3+C3));
%     theta3n = 2*atan2d((B3+sqrt(B3^2-(A3^2-C3^2))),(A3+C3));
    theta3n = 2*atan2d((B3-sqrt(B3^2+(A3^2-C3^2))),(A3+C3));
    
    theta3s = [theta3n,theta3p];
    
    % ITERATE THROUGH 2 POSSIBLE THETA3 SOLUTIONS
    for i=1:numel(theta3s)
        theta3 = theta3s(i);
        s3 = sind(theta3);
        c3 = cosd(theta3);
        
        % p129 Craig, checked 06OCT23
        f1 = a3*(c3) + d4*(sa3)*(s3) + a2;
        f2 = a3*(ca2)*(s3) - d4*(ca2)*(sa3)*(c3) - d4*(sa2)*(ca3) - d3*(sa2);
        f3 = a3*(sa2)*(s3) - d4*(sa2)*(sa3)*(c3) + d4*(ca2)*(ca3) + d3*(ca2);
        
        k1 = f1;
        k2 = -f2;
        %k3 = f1^2 + f2^2 + f3^2 + a1^2 + d23^2 + 2*d23*f3;
        k4 = (f3+d2)*(ca1); %%TYPO?? no sin term**
        
        A2 = -k2;
        B2 = k1;
        C2 = (zw-k4)/(sa1);
        theta2p = 2*atan2d((B2+sqrt(B2^2+(A2^2-C2^2))),(A2+C2));
        theta2n = 2*atan2d((B2-sqrt(B2^2+(A2^2-C2^2))),(A2+C2));
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
            C1 = xw;
            
            theta1p = 2*atan2d((B1+sqrt(B1^2+(A1^2-C1^2))),(A1+C1));
            theta1n = 2*atan2d((B1-sqrt(B1^2+(A1^2-C1^2))),(A1+C1));
            theta1s = [theta1n,theta1p];
            
            for k=1:numel(theta1s)
                theta1 = theta1s(k);
                
                for l=1 % Test
%                 for l=1:size(R0_6s,3) % For every possible solution
%                     R0_6 = R0_6s(:,:,l); Commented since defined earlier
%                     in code
                    
                    theta4p = 0; % To obtain unrotated theta4, i.e. (4') frame
                    q_wrist = [theta1 theta2 theta3 theta4p 0 0]';
                    T0_i_wrist = forwardKin(q_wrist, q_types, D_H, "mod");
                    
                    % Pull out T0_4p; this is transformation to unrotated
                    % wrist frame
                    
                    T0_4p = T0_i_wrist(:,:,4);
                    R0_4p = T0_4p(1:3,1:3);
                    
                    % Determine coordinates of target in (4')
                    R4p_6 = R0_4p'*R0_6;
                    P4p_6 = R0_4p'*r0_46; % This works since it is a relative (3D) vector
                    
                    T4p_6 = [R4p_6,   P4p_6;
                             zeros(1,3), 1];
                    
                    P6_tgt = zeros(3,1); % This is the position of the target relative to the end effector ((6) frame) ****
                    P6_tgt = [P6_tgt;1]; % Homogenize
                    P4p_tgt = T4p_6*P6_tgt; % Equivalent notation as r4p_4p,6 = r4p_4,6
                    
                    % Use basic spherical trig to solve for theta4,5
                    xt = P4p_tgt(1);
                    yt = P4p_tgt(2);
                    zt = P4p_tgt(3); % also equals cos(theta5)
                    theta4 = atan2d(yt,xt);
                    s5 = sqrt(xt^2 + yt^2);
                    theta5 = atan2d(s5,zt);
                    
                    q_ee = [theta1 theta2 theta3 theta4 theta5 0]';
                    T0_i_ee = forwardKin(q_ee, q_types, D_H, "mod");
                    
                    % Pull out T0_6p; this is transformation to unrotated
                    % wrist frame
                    
                    T0_6p = T0_i_ee(:,:,6);
                    R0_6p = T0_6p(1:3,1:3);
                    
                    Xhat0_6p = R0_6p(:,1);
                    Yhat0_6p = R0_6p(:,2);
                    c6 = Xhat0_6p'*Xhat0_6;
                    s6 = Yhat0_6p'*Xhat0_6;
                    theta6 = atan2d(s6,c6);
                    
%                     % Since only considering rotations, we do not need to consider the translations
%                     % the below is technically not correct since would want to be
%                     % multiplying consecutive 3x3 matrices as opposed to
%                     % 4x4's, however the result is the same because the
%                     % additional diagonal one does nothing and we pull out
%                     % the useful data anyhow.
%                     % Note that the same would not work for consecutive
%                     % translations, as each consecutive one needs to be
%                     % multiplied by the accumulative rotation matrix before
%                     % being added on to the next.
%                     R0_4p = planarRotation(theta4p,'z')*planarRotation(alpha3, 'x')*planarRotation(theta3,'z')*planarRotation(alpha2, 'x')*planarRotation(theta2,'z')*planarRotation(alpha1, 'x')*planarRotation(theta1,'z')*planarRotation(alpha0, 'x'); % NOTE: could simply subst 
%                     R0_4p = R0_4p(1:3,1:3); % ... now a 3x3
%                     B = (R0_4p)'*R0_6; % Orthogonal: inverse is transpose
                                    
%                     theta5 = atan2d(B(1,3),-B(2,3));
%                     theta6 = atan2d(-B(3,1),B(3,2));
                    
%                     thetas(i,j,k,:) = [theta1, theta2, theta3, theta4, theta5, theta6];
                
                      display([theta1, theta2, theta3, theta4, theta5, theta6])
                end
            end
        end
    end

% % Print error between two methods
% thetas_residual = thetas_fwd - thetas_inv
