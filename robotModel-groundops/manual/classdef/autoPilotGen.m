%% autoPilotGen.m
% Performs inverse kinematics to determine motor angles based on a target
% and desired task
% SOURCES:
% https://www.mathworks.com/help/robotics/ref/generalizedinversekinematics-system-object.html

%{
TODO:
() + Consolidate invKin() within the MSG functionality
() -Indicate need to change to current directory for addpath to work
properly
() -Change 'Nsol' to a more appropriate name, i.e. Norientations
() ?Are they constraints or relaxed constraints/targets?
() - remove entirely reliance on ee frame in exchange for 6 frame?  or
other way around?

%}

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

%% Pull target pose from .ply
CANNED = true; % MUST BE false TO ACTUALLY USE VALUES
zmax = .55;
TMSG_tgt = experiment.MSGplyGrabPose(zmax, CANNED);

% NOTE: the particular choice of x,y axes in the above does not matter --
% this will be accounted for in the later code

%% Define relationship between target and goal frame of frame (6)
% Removes dependence on end effector
whichArm = "II"; % Only functioning arm
TMSG_0 = experiment.arm2MSG(whichArm);
[T0_6s,Nsol,T6_tgts] = experiment.MSGtgtWtask2goal(TMSG_tgt,whichArm,"move2pt");

T0_tgt = TMSG_0\TMSG_tgt;
P0_tgt = T0_tgt(1:3,4);
R0_tgt = T0_tgt(1:3,1:3);

%% rigidBodyTree
robot = RECS.robot;
% showdetails(robot)
% show(robot) % displays robot in home config (all joint variables = 0)
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
					'ConstraintInputs',{'position','aiming','orientation','joint'});

%% SPECIFY THE CONSTRAINTS
% Recall whichArm = "II";
% Create the corresponding constraint objects.

% POSITION OF END EFFECTOR
posTgt = robotics.PositionTarget('ee');
posTgt.TargetPosition = P0_tgt;

% AIMING OF END EFFECTOR
% use joint6 frame since aiming constrains based on z axis
% z axis of joint 6 points in same direction as end effector camera
aimCon = robotics.AimingConstraint('body6'); 
aimCon.TargetPoint = P0_tgt;

% ORIENTATION OF END EFFECTOR
% requires the orientation of one body (the end effector) to match a target orientation within an angular tolerance in any direction
squareIndex = 1;
degTol = 1;
T0_6_t = T0_6s(:,:,squareIndex); % ***
T0_ee_t = T0_6_t*RECS.T6_ee; % ***
orCon = robotics.OrientationTarget('ee');
orCon.TargetOrientation = tform2quat(T0_ee_t); % translational components of tform are ignored
orCon.OrientationTolerance = deg2rad(degTol);

% JOINT LIMITS
jointCon = robotics.JointPositionBounds(robot);
jointCon.Bounds = RECS.thetaLimRng_relabs;


% Set constraint weights
posTgt.Weights = 0;

%% QUERY THE SOLVER
% [configSol,solInfo] = gik(initialguess,constraintObj,...,constraintObjN);

% Constraint input types, specified as a cell array of character vectors. The possible constraint input types with their associated constraint objects are

% Pass the constraint objects into the solver object with an initial guess
q0 = homeConfiguration(robot);
configSol = gik(q0,posTgt,aimCon,orCon,jointCon);
figure()
show(robot,configSol)

%% IF NOT GETTING A SOLUTION:
% Change squareIndex (possible values 1-4 reflecting each quadrant)
% 

%% PLOT SOLUTION
theta_relabs = zeros(6,1);
for ii = 1:6
    theta_relabs(ii) = (180/pi)*configSol(ii).JointPosition;
end

theta_relstowed = theta_relabs - RECS.theta0stow;
theta_armI_relstowed = zeros(6,1);
theta_armII_relstowed = theta_relstowed;
experiment.plotExperiment(theta_armI_relstowed,theta_armII_relstowed,TMSG_tgt)


