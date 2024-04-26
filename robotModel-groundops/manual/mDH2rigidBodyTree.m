

function robot = DH2rigidBodyTree(dhparams,dhType,Njoints,jointCodes) % *** add to class
% Temporary function to convert to MATLAB Robotic Systems Toolbox's default
% notation for robots

% INPUTS
% dhType = {"dh","mdh"} if standard or modified DH params, respectively
% jointCodes = vector of of booleans indexed by joint specifying {revolute = 0, prismatic = 1}
        
    
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
        
        bodies{i} = rigidBody(['body' num2str(i)]);
        joints{i} = rigidBodyJoint(['jnt' num2str(i)],jointType);
        setFixedTransform(joints{i},dhparams(i,:),dhType);
        bodies{i}.Joint = joints{i};
        if i == 1 % Add first body to base
            addBody(robot,bodies{i},"base")
        else % Add current body to previous body by name
            addBody(robot,bodies{i},bodies{i-1}.Name)
        end
    end
%     showdetails(robot)
end