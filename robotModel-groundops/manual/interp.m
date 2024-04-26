%% interp.m

%{
TODO:
() + add functionality to consider max angle rate, etc?

%}

function Qfcn = interp(Q,t,TYPE) %thdd,tb
% INPUTS
% Q = array of joint variables, row = joint index, column = point index
%     NOTE: first and last points will be start point and end point,
%     respectively -- these will be assigned zero velocit
% TYPE = {'lin','parabBlendLin','cubic'} = type of interpolation
% thdd = acceleration for blend segments; if NaN then specifies linear
% interpolation
% tb = duration of parabolic blend at either end

% OUTPUTS
% Qfcn = array of joint variable functions, row = joint index, column = spline index
%     NOTE: jth spline span
%     as a result will have one fewer column than
    
    dims = size(Q);
    N = dims(1); % N joints
    M = dims(2); % M total points, including 2 end points and M-2 vias
    
    %% INTERPOLATION
    
    % Index over M-1 splines spanning M points
    for kk = 1:(M-1)
        % Index over each joint angle
        for ii = 1:N
            qi_0 = Q(ii,(kk-1)); % Initial joint angle
            qi_1 = Q(ii,kk); % Final joint angle
            
            
            
            
%             qi_fcn = 
            
        end
    end



end