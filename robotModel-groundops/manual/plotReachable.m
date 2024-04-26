%% plotReachable.m
% Plots reachable sets for the specified robot
% Assumes 6R with spherical wrist (e.g. the RSat robot), so plots possible 
% wrist positions and orientations separately

%{
TODO:
() = incomplete, (X) = complete
+ = addition, - = fix/debug

() - Make extended position and delThetaLimRng 
%}

function plotReachable(thetaInit,delThetaLimRng,Nsamp)
% INPUTS:
% Nsamp = number of samples to be taken, evenly-spaced, across the
% specified range for each joint angle
% OUTPUTS:
    %% FORWARD KINEMATICS
    % % FIND JOINT POSES GIVEN AN INPUT THETA
    inputMode ="abs";
    q_fwd = [-90 0 69 0 0 0]'; % index similarly to the rest of the D_H params, add 0 entry
    % q_fwd = [0 -90 -180 0 180 -90]'; %***
    % q_fwd = [0 0 0 0 0 0];
    if inputMode == "rel"
        theta_fwd = q_fwd + theta0;
    elseif inputMode == "abs"
        theta_fwd = q_fwd;
    end

    N = numel(theta_fwd);
    q_types = zeros(N,1); % specify 0 for each revolute joint, and 1 for each prismatic one
    T0_i = forwardKin(theta_fwd, q_types, D_H, "mod",inputMode); % Using "abs" for consistency

    % % DEVISE A CANNED QR TARGET THAT MATCHES (6) OF CURRENT ARM ORIENTATION
    % Rotation
    T0_6 = T0_i(:,:,end); % Joint 6 (end effector) pose in 0 frame
    % NOTE: since motor 6's axis points outwards through the wrist, must
    % negate it to get an outward facing normal for the QR code
    R0_6 = T0_6(1:3,1:3);
    Xhat0_6 =  R0_6(:,1);
    Yhat0_6 =  R0_6(:,2);
    Zhat0_6 =  R0_6(:,3);

    Xhat0_QR = -R0_6(:,1);
    Yhat0_QR =  R0_6(:,2);
    Zhat0_QR = -R0_6(:,3);
    R0_QR = [Xhat0_QR,Yhat0_QR,Zhat0_QR];

    % Position
    r0_6QR = zeros(3,1); % relative offset
    P0_6 = T0_6(1:3,4);
    P0_QR = P0_6+r0_6QR; % Compensate for relative offset

    % Combine in homogenous pose matrix
    T0_QR = [R0_QR,   P0_QR;
             zeros(1,3), 1];

    % % PLOT
    plotRobotAxes(T0_i,T0_QR)

end

