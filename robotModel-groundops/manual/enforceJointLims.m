%% FILTER BASED ON ROTATION RESTRICTIONS FOR EACH JOINT
% DEPRECATED AS OF 28JAN24 -- migrated to RSAT.m
% Only takes single vector of joint angles (single solution)

%{
TODO
() -Perhaps loop can be replaced with more efficient array-based methods!
() + Introduce plotting feature that shows where solution is relative to
what is reachable
%}

function logic = enforceJointLims(absTheta)
% INPUTS:

% 
% OUTPUTS:
% logic = vector of booleans indexed by joint (true = within lims, 
% false = violation)
% 
    % Angle limits for each joint, relative to the extended position (INITPOS="EXT") (deg)
    theta0ext = 90*[0 1 0 2 0 0]'; % i; these correspond to the 'extended' position; 
                                   % Remove definition when turn into a class!
    delThetaLimRng = [-255,0;        % **guess**
                   -180,180;
                   -180,0;
                   -180,180;
                   -123.95,34.29; % referenced from previous arm's centerline (i.e. extended position)
                   -180,180;
                   -45,135];       % **guess**, note this 7th entry is for the end effector motor!
    absThetaLimRng = delThetaLimRng + theta0ext;
    
    % Enforce
    N =numel(absTheta);
    logic = zeros(1,N);
    for ii = 1:N
        negLim = absThetaLimRng(ii,1);
        posLim = absThetaLimRng(ii,2);
        absTheta_i = absTheta(ii);
        
        logic(ii) = (absTheta_i>negLim)&(absTheta_i<posLim); %https://stackoverflow.com/questions/1379415/whats-the-difference-between-and-in-matlab
    end
end