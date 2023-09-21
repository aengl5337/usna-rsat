thetas_fwd = [NaN 0 0 0 0 0 0]; % index similarly to the rest of the D_H params
r0_6QR = zeros(3,1);
% lee=44.5;  % Measured to endpoint of end effector 
%            % (not grabbing centroid)


% Link twist angles (deg) **make sure rotation CW/CCW sense is correct
gamma0=-90; % This one rotates around Y axis to be consistent with the illustrations' base frame
alpha1=90;
alpha2=90;
alpha3=-90;
alpha4=90;
alpha5=-90;
alpha6=NaN;
% alpha6=90;

% Link Lengths (mm) from i to i+1
a0=0;
a1=0;
a2=0;
a3=7.99;
a4=0;
a5=0;
a6=NaN;
% a6=0;

% Link offsets (mm)* from i-1 to i
d0=NaN;
d1=0;
d2=312.24;
d3=0;
d4=182.02;
d5=0;
d6=76.6;

a=[a0, a1, a2, a3, a4, a5, a6]';
d=[d0, d1, d2, d3, d4, d5, d6]'; 
alpha=[gamma0, alpha1,alpha2,alpha3,alpha4,alpha5,alpha6]';

D_H = [a d alpha];

[T0_6] = forwardKin(thetas_fwd, D_H);

thetas_inv = pieperInvKin(T0_6, D_H, r0_6QR);
% 
% % Print error between two methods
% thetas_residual = thetas_fwd - thetas_inv