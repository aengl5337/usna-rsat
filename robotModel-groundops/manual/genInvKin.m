function genInvKin()
%     [configSol,solInfo] = gik(initialguess,constraintObj,...,constraintObjN) finds a joint configuration, configSol, based on the initial guess and a comma-separated list of constraint description objects. The number of constraint descriptions depends on the ConstraintInputs property.
    robot = mDH2rigidBodyTree(); %***
    [configSol,solInfo] = gik(initialguess,constraintObj,...,constraintObjN)
%     Initial guess of robot configuration, specified as a structure array or vector. The value of initialguess depends on the DataFormat property of the object specified in the RigidBodyTree property specified in gik.
%     Constraint descriptions defined by the ConstraintInputs property of gik, specified as one or more of these constraint objects:
% 
% constraintAiming
% 
% constraintCartesianBounds
% 
% constraintJointBounds
% 
% constraintOrientationTarget
% 
% constraintPoseTarget
% 
% constraintPositionTarget

% Robot configuration solution, returned as a structure array or vector, depends on the DataFormat property of the object specified in the RigidBodyTree property specified in gik.

end