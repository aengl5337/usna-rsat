%% plyGrabPose.m
% GUI-search for, open&view, manually select pose points, and return pose
% of a (rectangular) target

%{
TODO:
(X) + Wrap in MSG.m
() + better separate functionality
() + Add redundancy/averaging of normal (i.e. compute normal at every
vertex and average
() - How to deal with incomplete cube surface showing?
%}

function Td_tgt = plyGrabPose(zmax,CANNED)
%% INPUTS
% zmax = .62 maximum depth displayed in m (D435 has depth noise out to
% infinity depth)
% (good values: zmax = .55 for ISS 10JAN24, .62 for Houston test)
% CANNED = if true, will use demo values and skip the point cloud file selection
%% OUTPUTS
% Td_tgt = 4x4 pose matrix of target, referenced in the D435 frame in
% units of (mm)
    %% Constants
    CENTERMANUAL = false; % manually select center
    
    %% Select and view 3D Camera point cloud (.ply format)
    if ~CANNED
        %clear, clc, format compact
        % ptCloud = pcread('Test_1.ply');
        ptCloud = pcread(uigetfile('.ply')); % Edit the file name to match 
        % the .ply that you want to use
        testloc=ptCloud.Location;
        testcol=ptCloud.Color;
        testnorm=ptCloud.Normal;


        for ii=1:length(testloc)
            if testloc(ii,3)<= -zmax % Negative since camera normal is negative z axis
                testloc(ii,:)=NaN;
                testcol(ii,:)=NaN;
                testnorm(ii,:)=NaN;
            end
        end
        figure(2)
        clf
        % testout=pointCloud(testloc,'Color', testcol,'Normal',testnorm,...
        %     'Intensity',ptCloud.Intensity);
        testout=pointCloud(testloc); % Solves weird norm problem
        pcshow(testout);
        pause
    end
    %% Pick points on cube
    disp("Ensure the following when selecting points:")
    disp("1. select p1-4 in a counterclockwise fashion to ensure z axis is the outward normal")
    disp("2. select the points to be the vertices of a rectangle if possible (e.g. the visible face of the cube target")
    disp("3. Units are meters/(m) (which are native to the D435 .ply so don't need to change)")
    if CANNED
        % EG for dark bar with cube.ply:
        p2 = [.017719,.022797,-.28];
        p3 = [-.011528,.031038,-.282];
        p1 = [.011077,-.004875,-.278];
    else
        p1 = input("Input [x y z] of p1");
        p2 = input("Input [x y z] of p2");
        p3 = input("Input [x y z] of p3");
    end
    % Ensure are column vectors
    p1 = reshape(p1,3,1);
    p2 = reshape(p2,3,1);
    p3 = reshape(p3,3,1);
    
    
    %% Determine centroid/origin
    if CENTERMANUAL
       c = input("Input [x y z] of tgt center");
       c = reshape(c,3,1);
    else       
       disp("NOTE: p1-3 only are used to compute axes/orientation of target")
       disp("p1-4 are averaged to determine centerpoint")
       if CANNED
            p4 = [-.016728, -.00054106,-.283];
       else
            p4 = input("Input [x y z] of p4");
       end
       p4 = reshape(p4,3,1);
       c = 1000*(p1+p2+p3+p4)/4; % Convert from m to mm
    end
    
    %% Determine and orthogonalize axes
    x = p3-p2;
    y = p1-p2;
    z = cross(x,y);
    z = normalizeVector(z);
    x = normalizeVector(x);
    y = normalizeVector(y);
    
    %% Format output as homogenous
    
    R = [x y z];
    Td_tgt = [R          c;
              zeros(1,3) 1];
    
end



