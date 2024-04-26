% REF: https://www.mathworks.com/help/robotics/ref/rigidbodyjoint.setfixedtransform.html?s_tid=doc_ta

function robot = DH2rigidBodyTree(dhparams,dhType,Njoints,jointCodes)
% Function to convert to MATLAB Robotic Systems Toolbox's default
% notation for robots

% INPUTS
% dhparams = [a alpha d theta]
% dhType = {"std","mod"} if standard or modified DH params, respectively
% jointCodes = vector of of booleans indexed by joint specifying {revolute = 0, prismatic = 1}
        
    %% Shift DH flag naming convention
    if dhType == "std"
        dhType = "dh";
%         These parameters are used to set the ChildToJointTransform property. 
%         The JointToParentTransform property is set to an identity matrix.
    elseif dhType == "mod"
        dhType = "mdh";
%         These parameters are used to set the JointToParentTransform property. 
%         The ChildToJointTransform is set to an identity matrix.
    end
    
    %% For each joint, convert the DH parameters
%     robot = rigidBodyTree; % IF version>=R2019b
    robot = robotics.RigidBodyTree; % IF version<R2019b
    
    bodies = cell(Njoints,1);
    joints = cell(Njoints,1);
    for i = 1:Njoints
        if jointCodes(i) == 0
            jointType = "revolute";
        elseif jointCodes(i) == 1
            jointType = "prismatic"; % Check this is the valid input
%         else
%             % ***Push back an error
        end
        
%         %% IF version>=R2019b
%         bodies{i} = rigidBody(['body' num2str(i)]);
%         joints{i} = rigidBodyJoint(['jnt' num2str(i)],jointType);
%         setFixedTransform(joints{i},dhparams(i,:),dhType);
%         bodies{i}.Joint = joints{i};
%         if i == 1 % Add first body to base
%             addBody(robot,bodies{i},"base")
%         else % Add current body to previous body by name
%             addBody(robot,bodies{i},bodies{i-1}.Name)
%         end
        %% IF version<R2019b
        
%         Create and add rigid bodies to the robot. 
%         Specify the previous body name when calling addBody to attach it. 
%         Each fixed transform is relative to the previous joint coordinate frame
        bodies{i} = robotics.RigidBody(['body' num2str(i)]);
        joints{i} = robotics.Joint(['jnt' num2str(i)],jointType);
        setFixedTransform(joints{i},dhparams(i,:),dhType); % sets the transformation relationship parameters of the specified Joint object        
        bodies{i}.Joint = joints{i};
        if i == 1 % Add first body to base
            addBody(robot,bodies{i},"base") % connect robot.Body{1,1} to robot.Base 
        else % Add current body to previous body by name
            addBody(robot,bodies{i},bodies{i-1}.Name) % connect robot.Body{1,i} to robot.Body{1,i-1}
        end
        
    end
end