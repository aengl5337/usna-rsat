%% pieperInvKin.m
% This is a special-case inverse kinematics solution solution from D.
% Pieper as per Craig (pp129, ch4.6 in 2nd Edition) that is geared towards
% the NSTAR robot arm.  A special-case solution for theta4 is used, playing
% on the fact that a1 equals ZERO.

%{
TODO
() + OOP out the theta1-3 solving! 
()-flesh out documentation/variable naming better
()+enable multiple possible target frames:
a. trying 4 possible orientations that are "square" to each other
b. searching for an arbitrary rotation between the (6) and target frames, 

()-pull out reliance on q_types -- its given that this is a 6R robot
() - Ensure syms are handled properly by enforceJointLims
() - modify to accept r0_6tgt as symbolic
() + *** fix if generalizing for Ntgt ~=1
%}

function [theta_invs,T0_i_invs] = pieperInvKin(T0_6goal, q_types, D_H, D_H_type) %,eaxes,thetasErr,delPs,magPs
% INPUTS:
%
% T0_6goal = goal pose of (6) frame in 4x4 transformation matrix form
% [ [R0_6] [P0_6];
%   [0 0 0]  1  ]
%   NOTE: can be specified as double or sym!
%
% D_H = (modified) Denavit-Hartenberg link parameters as follows:
% [[alpha_(i-1)] [a_(i-1)] [theta_(i)] [d_(i)] ]
%
% OUTPUTS:
%
% thetas = vector of each joint angle (absolute D_H parameters, not 
% referenced to any arm position) corresponding to input points
%
%% CONSTANTS/SWITCHES
c = class(T0_6goal);
inputMode = "abs"; % Forward kinematics joint variable input mode (should be "abs")

% PLOT FUNCTIONS DEPRECATED
PLOT_SOLN = false; % cannot plot a symbolic solution
PLOT_ERR = false; % Plot error as per resid4x4() in 3D

if D_H_type=="mod"
    %% INVERSE KINEMATICS

    % Transform from target frame to 6th motor frame (i.e. frame (6))
    % (The following code needs generalization)
    P0_6 = T0_6goal(1:3,4);
    R0_6 = T0_6goal(1:3,1:3);
    Xhat0_6 = R0_6(:,1);
    Yhat0_6 = R0_6(:,2);
    Zhat0_6 = R0_6(:,3);

    % Number of potential solutions for theta_{3,2,1} resp(will have 2 each)
    N3 = 2;
    N2 = 2;
    N1 = 2;
    Ntgt = 1; % Number of possible targets ***
    Njoints = 6; % Number of serial actuators

    Ntot = N3*N2*N1*Ntgt; % Total number of solutions

    % Storage containers, made flexible with double or sym class
    if c=="sym"
        theta_invs = sym('theta_invs',[Njoints,Ntot]);
        T0_i_invs = sym('T0_i_invs',[4,4,Njoints,Ntot]);
%         eaxes = sym('eaxes',[3,Ntot]);
%         thetasErr = sym('thetasErr',[1,Ntot]);
%         delPs = sym('delPs',[3,Ntot]); 
%         magPs = sym('magPs',[1,Ntot]);
    else
        % Make storage container for all of Ntot possible solutions 
        % (see flattened indexing scheme below, variable 'ind')
        theta_invs = zeros(Njoints,Ntot); % Joint angles, absolute
        T0_i_invs = zeros(4,4,Njoints,Ntot); % Transform matrices

%         % Make storage containers for error metrics
%         eaxes = zeros(3,Ntot); % Eigenaxis of rotation error
%         thetasErr = zeros(1,Ntot); % Rotation error angle
%         delPs = zeros(3,Ntot); % Position error vector
%         magPs = zeros(1,Ntot); % Position error magnitude
    end

    % GIVENS
    % D_H is indexed as:
    % [alpha_i-1, a_i-1, theta
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

%     ca0 = cosd(alpha0); % Not used
    ca1 = cosd(alpha1);
    ca2 = cosd(alpha2);
    ca3 = cosd(alpha3);
    ca4 = cosd(alpha4);
    ca5 = cosd(alpha5);

%     sa0 = sind(alpha0); % Not used
    sa1 = sind(alpha1);
    sa2 = sind(alpha2);
    sa3 = sind(alpha3);
    sa4 = sind(alpha4);
    sa5 = sind(alpha5);

        % Extract origins of motors 4 and 6 frame from the given goal pose
        r0_46 = d6*(Zhat0_6); % This is the vector in base coords between the origins of motors 4 and 6
                                % (-Zhat0_QR) = Zhat0_6
                                % *Generalize this to the caseswhere Zhat0_QR != Zhat0_6, don't cheat and use d6!

        P0_4 = P0_6-r0_46; % Subtracting this from motor 6 origin yields motor 4 origin (in frame (0) coords)
        % P0_4 = float array, 3 element column vector of the cartesian coordinates
        % of the desired location of the "4ORG" as prescribed by Pieper in Craig, with
        % respect to the base frame

        r = norm(P0_4)^2; % The norm^2 of P0_4ORG (p130 Craig)

        % Decompose components of wrist position
        xw = P0_4(1);
%         yw = P0_4(2); % Unused
        zw = P0_4(3);

        % Solve for theta3 using a nice little trig sub (DETAIL THIS ***)
        % k3 = A3*cos(theta3) + B3*sin(theta3)+D3 = r =>
        % A3*cos(theta3) + B3*sin(theta3) = C3
        
        % Derived from text:
        A3 = (2*a2*a3 - 2*d2*d4*(sa2)*(sa3));
        B3 = (2*a2*d4*(sa3) + 2*a3*d2*(sa2));
        C3 = -r + (a1^2+a2^2+a3^2) + (d2^2+d3^2+d4^2)+ d2*d3*(ca2) + 2*d3*d4*(ca3) + 2*d2*d4*(ca2)*(ca3);
%         % Corrected:
%         
%         % C3 coeff
%         A3 = (2*a2*a3 - 2*d2*d4*ca3*ca2);
% 
%         % S3 coeff
%         B3 = (2*a2*d4*ca3 + 2*a3*d2*sa2);
%         
%         % Constant coeff
%         D3 = (a2^2 + a3^2) + (d2^2 + d3^2 + d4^2) + 2*d2*d3*ca2 + 2*d3*d4*sa3 + 2*d2*d4*ca2*sa3;
%         C3 = r-D3;
        
        % NOTE: typo in book corrected here
        discr3 = sqrt(B3^2+(A3^2-C3^2));
        if imag(discr3)==0
            theta3p = 2*atan2d((B3+discr3),(A3+C3));
        %   book says:  theta3 = 2*atan2d((B3 +/- sqrt(B3^2-A3^2-C3^2)),(A3+C3));
            theta3n = 2*atan2d((B3-discr3),(A3+C3));
            theta3s = [theta3n,theta3p];
        else % NO VALID SOLUTIONS
            disp("Goal pose unreachable, no solutions")
            theta_invs(:,:) = NaN(6,8); % Each column corresponds to solution
            T0_i_invs(:,:,:,:) = NaN(4,4,6,8);
        end
        
        % ITERATE THROUGH 2 POSSIBLE THETA3 SOLUTIONS
        for i=1:N3
            theta3 = theta3s(i)+180; %*************************DEBUG
%             theta3 = theta3s(i);
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
            discr2 = sqrt(B2^2+(A2^2-C2^2));
            if imag(discr2)==0
                theta2p = 2*atan2d((B2+discr2),(A2+C2));
                theta2n = 2*atan2d((B2-discr2),(A2+C2));
                theta2s = [theta2n,theta2p];
            else
                disp("This theta3 yields no theta2 solutions")
                ind_nosol_2 = 4*(i-1) + [1:4];
                for h = ind_nosol_2
                    theta_invs(:,h) = NaN(6,1); % Each column corresponds to solution
                    T0_i_invs(:,:,:,h) = NaN(4,4,6,1);
                end
                continue
            end
            %% do something with theta2s**
            for j=1:N2 % 2 possible solutions
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

                discr1 = sqrt(B1^2+(A1^2-C1^2));
                if imag(discr1)==0
                    theta1p = 2*atan2d((B1+discr1),(A1+C1));
                    theta1n = 2*atan2d((B1-discr1),(A1+C1));
                    theta1s = [theta1n,theta1p];
                else
                    disp("This theta2 yields no theta1 solutions")
                    ind_nosol_1 = 4*(i-1) + 2*(j-1) + [1:2]; % ***
                    for h = ind_nosol_1
                        theta_invs(:,h) = NaN(6,1); % Each column corresponds to solution
                        T0_i_invs(:,:,:,h) = NaN(4,4,6,1);
                    end
                    continue
                end
                for k=1:N1 
                    theta1 = theta1s(k);

                    for l=1:Ntgt % Test (Ntgt will be generalized to number of possible targets)
    %                 for l=1:size(R0_6s,3) % For every possible solution
    %                     R0_6 = R0_6s(:,:,l); Commented since defined earlier
    %                     in code

                        theta4p = 0; % To obtain unrotated theta4, i.e. (4') frame
                        theta_wrist = [theta1 theta2 theta3 theta4p 0 0]';
                        T0_i_wrist = forwardKin(theta_wrist, q_types, D_H, D_H_type,inputMode);

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
                        theta5 = -atan2d(s5,zt);

                        q_ee = [theta1 theta2 theta3 theta4 theta5 0]';
                        T0_i_ee = forwardKin(q_ee, q_types, D_H, D_H_type,inputMode);

                        % Pull out T0_6p; this is transformation to unrotated
                        % end effector frame

                        T0_6p = T0_i_ee(:,:,6);
                        R0_6p = T0_6p(1:3,1:3);

                        Xhat0_6p = R0_6p(:,1);
                        Yhat0_6p = R0_6p(:,2);
                        % Pull angular info from dot products to the axes
                        % we determined earlier
                        c6 = Xhat0_6p'*Xhat0_6;
                        s6 = Yhat0_6p'*Xhat0_6;
                        theta6 = atan2d(s6,c6);

    %                     % Since only considering rotations, we do not need to consider the translations
    %                     % the below is technically not correct since would want to be
    %                     % multiplying consecutive 3x3 matrices as opposed to
    %                     % 4x4's, however the result is the same because the
    %                     % additional diagonal 1 does nothing and we pull out
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

                          theta_inv = [theta1, theta2, theta3, theta4, theta5, theta6];
                          % Measure distance, angle from target
                          T0_i_inv = forwardKin(theta_inv, q_types, D_H, D_H_type, inputMode);
                          
                          % Eval and plot error metrics, except if sym
                          % input
%                           if c=="sym"
%                             eaxis = NaN(3,1);
%                             thetaErr = NaN;
%                             delP = NaN(3,1);                           
%                           else
%                             T0_EE = T0_i_inv(:,:,end); % ** Generalize this to more
%                             [eaxis,thetaErr,delP] = resid4x4(T0_tgt,T0_EE,PLOT_ERR);
%                           end

                          % Store
                          ind = k + (j-1)*N1 + (i-1)*N2*N1 + (l-1)*N3*N2*N1; % Flattened index 
                          theta_invs(:,ind) = theta_inv; % Each column corresponds to solution
                          T0_i_invs(:,:,:,ind) = T0_i_inv;
                          
%                           eaxes(:,ind) = eaxis;
%                           thetasErr(ind) = thetaErr;
%                           delPs(:,ind) = delP;
%                           magPs(:,ind) = norm(delP);
                    end
                end
            end
        end
% elseif D_H_type...     Code solution to other D-H formalisms
else
    disp("Only solves given modified D-H parameters")
    return
end
end