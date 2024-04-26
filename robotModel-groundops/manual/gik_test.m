%% Clean up
clc
clear all
close all

%% Add path and all subdirs
path = "..";
addpath(genpath(path));
% path = ".";
% addpath(genpath(path));
%% Initiate MSG object (and RSAT for invKin)
experiment = MSG();
RECS = RSAT();

%% rigidBodyTree
robot = RECS.robot;
showdetails(robot)
show(robot) % displays robot in home config (all joint variables = 0)
% DataFormat property, 
% Input/output data format for kinematics and dynamics functions, specified as "struct", "row", or "column"
% i.e. specified in rigidBodyTree, determines how 'initialguess' should be formatted

% %% Test changing config
% figure()
% config = homeConfiguration(robot);
% config(3).JointPosition = pi/2;
% show(robot,config)

%% IF version>=R2019b, use
% inverseKinematics
% generalizedInverseKinematics
%% IF version<R2019b, use
% robotics.InverseKinematics
% robotics.GeneralizedInverseKinematics
% robotics.PoseTarget

%% BUILD THE SOLVER
% gik = robotics.GeneralizedInverseKinematics('RigidBodyTree',rigidbodytree,'ConstraintInputs',inputTypes);
gik = robotics.GeneralizedInverseKinematics(...
					'RigidBodyTree',robot,...
					'ConstraintInputs',{'aiming','joint'});

%% SPECIFY THE CONSTRAINTS
                    
% % Create the corresponding constraint objects.
% posTgt = robotics.PositionTarget('ee');
% posTgt.TargetPosition = [0.0 0.5 0.5];

% use joint6 frame since aiming constrains based on z axis
% z axis of joint 6 points in same direction as end effector camera
aimCon = robotics.AimingConstraint('body6'); 
aimCon.TargetPoint = [0.0 0.0 0.0];

jointConst = robotics.JointPositionBounds(robot);
jointConst.Bounds = RECS.thetaLimRng_relabs;

% ALternately, 
% posTgt = constraintPositionTarget("iiwa_link_ee_kuka");
% posTgt.TargetPosition = [0.0 0.5 0.5];
% 
% aimCon = constraintAiming("iiwa_link_ee_kuka");
% aimCon.TargetPoint = [0.0 0.0 0.0]; (This 


% poseConst = constraintPoseTarget(endeffector);
% Name of the end effector, specified as a string scalar or character vector. 
% When using this constraint with generalizedInverseKinematics, the name must match a body specified in the robot model (rigidBodyTree).
% 
% Example: "left_palm"
% 
% Data Types: char | string


%% QUERY THE SOLVER
% [configSol,solInfo] = gik(initialguess,constraintObj,...,constraintObjN);

% Constraint input types, specified as a cell array of character vectors. The possible constraint input types with their associated constraint objects are

% Pass the constraint objects into the solver object with an initial guess
q0 = homeConfiguration(robot);
configSol = gik(q0,aimCon,jointConst);
figure()
show(robot,configSol)