%{ 
TODO:
() + Enable inputting multiple solutions
() + Transfer over error handling to this function
%}

%% FILTER SOLUTIONS
% DEPRECATED 28JAN24, MIGRATED TO RSAT.M

function filterInvKinSols(absThetas)
    % for each solution in array...
        % % FILTER BASED ON PROXIMITY***
        % % Print error between two methods
        % thetas_residual = thetas_fwd - thetas_inv


        % % FILTER BASED ON ROTATION RESTRICTIONS FOR EACH JOINT
        logic = enforceJointLims(


end