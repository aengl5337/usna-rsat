clear all 
close all
format compact
clc

%% NSTAR_ArmSim_script.m
%{
REV0-????
REV1-30SEP-12OCT23, Alec Engl (aengl5337@gmail.com)
Revamped&checked math, D-H parameters,etc
Arm now plots correctly (i.e. D-H params correct)
Inverse Kinematics now provides several correct solutions, but still differs from expected
REV2-11DEC23, Alec Engl
Inverse kinematics debugged and offloaded to pieperInvKin.m
(deprecated as of 08JAN24 -- see RSAT.m and ops script)

TODO:
() = incomplete, (X) = complete
+ = addition, - = fix/debug

() + CONVERT WITH ALL FUNCTIONS INTO A LARGE CLASS, PREVENT NEED TO
CROSS-DEFINE VARIABLES (E.G. THETA0EXT IN ENFORCEJOINTLIMS())
() + Figure out if arms collide with one another
() + Add FOV of D435 to the spatial orientation 

() - Ensure frame zero transform input correctly (and ensure changes
according to which arm) -- for now this assumes 0 offset or rotation
() - CHANGE SCRIPT NAME TO "DEMO"
() - RETURN INVERSE KINEMATICS CODE TO RESPECTIVE FUNCTION!
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
(X) - Ensure intrinsic thetas are being subtracted from inverse kinematics
() - ***ADD ERROR CONDITIONS*** (NOTE: atan2d(3,0) = +90deg)
() + OOP function for theta sub sol
() + general joint frame transformation functions
() + Generalize end effector transformation as another fixed transformation
-- refine conventions
() - Clean up notation (i.e. is P a 3d vector or 4d one?
() - Is using atan2 for theta4,5 actually useful?
(X) + filter out imaginary solutions as soon as they are calculated or allow
to propagate through to the end of the loop then filter? -- let them
propagate for now, since if one solution is imaginary both will be (both
share the same discriminant sqrt() term)
(X) +- finish indexing solution storage container
() - change the coordinate frame that the eigenaxis is measured in to that
of the QR code -- that way, get a 2D vector each time -- this can even be
reduced to an angle using atan2d
() - Do the same with delP vectors -- more useful in QR frame
() - Clean up/label error displays (resid4x4())
() - Tune error metrics to -zQR being the target as opposed to +zQR
() - What is the best order to place solution filter layers?
() + Map out feasibility field
(~) + Set up arguments, and display graphically (arm pos, which arm are we
mapping, etc)
() + Output both absolute and relative thetas (but we would need to track
each and every position change to make true relative, so maybe skip it? Thus theta0 would simply be for display purposes)
(DEFINE ABS VS REL THETAS)
() - Debug stowed position shenanigans
() + Explain D_H parameter table indexing!
() + generalize invKin to include relative pose offsets too
() -Rename thetasErr
%}

%% User input
INITPOS = "EXT"; % Options: {"EXT","STOW"}


%% Some constants
path = "G:\My Drive\RSat master\softwareWorkingDir\USNA_NSTAR_RSat\robotics\archive\manual";
addpath(path);

% Lengths (mm)
l2 = 312.24;
l4 = 182.02; % l3
l4p = 7.99; % l3p
l6 = 76.6; %l5
% lee=44.5;  % Measured to endpoint of end effector 
%            % (not grabbing centroid)

% Denavit-Hartenberg Parameters (Modified -- Craig*)
% Link twist angles (alpha, deg)
alpha = 90*[0 1 1 1 1 -1]'; % i-1
% Link Lengths (a, mm)
a = [0 0 0 l4p 0 0]'; % i-1
% (Intrinsic) Joint angles (theta, deg)
theta0ext = 90*[0 1 0 2 0 0]'; % i; these correspond to the 'extended' position
theta0stow = theta0ext + [0 0 -180 0 +7.2 0];

if INITPOS == "EXT"
    theta0 = theta0ext;
elseif INITPOS == "STOW"
    theta0 = theta0stow;
else % Catch all misinputs
    disp("Bad input for INITPOS")
    return;
end

% Link offsets (d, mm)
d = [0 -l2 0 l4 0 l6]'; % i

D_H = [alpha,a,theta0,d];


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
plotRobotAxes(T0_i,T0_QR,false)

%% INVERSE KINEMATICS
[theta_invs,T0_i_invs,eaxes,thetasErr,delPs,magPs] = pieperInvKin(T0_QR, r0_6QR, q_types, D_H, "mod");

%TEMPORARY*** rotate axes result into QR frame
Ntot = 8;
axesQR = zeros(2,Ntot);
axesQR(1,:) = eaxes(3,:);
axesQR(2,:) =-eaxes(1,:);
etasQR = atan2d(axesQR(2,:),axesQR(1,:));

display(axesQR)
display(etasQR)
display(thetasErr)
display(delPs)
display(magPs)

%% DISPLAY DIFFS
delthetas = theta_invs - theta_fwd;

%% FILTER SOLUTIONS
% filterInvKinSols(



