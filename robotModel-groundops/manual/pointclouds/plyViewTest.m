%% plyViewTest.m
% This is deprecated -- moved to function plyGrabPose.m

%{
TODO:
() + Transfer to RSAT.m
() + Add redundancy/averaging
() - How to deal with incomplete cube surface showing?
%}

%% Test - view 3D Camera point cloud
%clear, clc, format compact
% ptCloud = pcread('Test_1.ply');
ptCloud = pcread(uigetfile('.ply')); % Edit the file name to match 
% the .ply that you want to use
testloc=ptCloud.Location;
testcol=ptCloud.Color;
testnorm=ptCloud.Normal;
for ii=1:length(testloc)
    if testloc(ii,3)<= -.62 %-.55 for ISS 10JAN24, -.62 for Houston test
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
%% Pick points on cube
% ENSURE you select p1 at a vertex, and p2 and p3 on both of its two
% flanking edges
p1 = input("Input [x y z] of p1");
p2 = input("Input [x y z] of p2");
p3 = input("Input [x y z] of p3");
% p4 = input("Input [x y z] of p4");

% Infer or manually select center?

%{ 
EG for dark bar with cube.ply:
Input [x y z] of p1[.017719,.022797,-.28]
Input [x y z] of p2[-.011528,.031038,-.282]
Input [x y z] of p3[.011077,-.004875,-.278]
%}

%% Determine axes
x = p2-p1;
y = p3-p1;
z = cross(x,y);
z = normalizeVector(z);
x = normalizeVector(x);




