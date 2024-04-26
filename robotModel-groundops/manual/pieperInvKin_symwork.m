% Working out the math symbollically, Craig 2ed pg130

% syms a1 a2 a3 d2 d3 d4 c3 s3 ca2 ca3 sa2 sa3
% 
% f1 = a3*c3 + d4*sa3*s3 + a2;
% f2 = a3*ca2*s3 - d4*sa3*ca2*c3 - d4*sa2*ca3 - d3*sa2;
% f3 = a3*sa2*s3 - d4*sa3*sa2*c3 + d4*ca2*ca3 + d3*ca2;

syms a0 a1 a2 a3 d1 d2 d3 d4 theta1 theta2 theta3 alpha0 alpha1 alpha2 alpha3

%% FROM TEXT
% f1 = a3*cos(theta3) + d4*sin(alpha3)*sin(theta3) + a2;
% f2 = a3*cos(alpha2)*sin(theta3) - d4*sin(alpha3)*cos(alpha2)*cos(theta3) - d4*sin(alpha2)*cos(alpha3) - d3*sin(alpha2);
% f3 = a3*sin(alpha2)*sin(theta3) - d4*sin(alpha3)*sin(alpha2)*cos(theta3) + d4*cos(alpha2)*cos(alpha3) + d3*cos(alpha2);

% k3 = f1^2 + f2^2 + f3^2 + a1^2 + d2^2 + 2*d2*f3;

% k3 = expand(k3);
% k3 = simplify(k3)

% However there were significant typos in the math, so we need to do it
% ourselves

%% MANUAL method 1
P3_4org = [a3; -d4*cos(alpha3); d4*sin(alpha3); 1];
% R_alpha = planarRotation(alpha2,'x');
% D_a = axialTranslation(a2,'x');
% R_theta = planarRotation(theta3,'z');
% D_d = axialTranslation(d3,'z');
% 
% T2_3 = R_alpha*D_a*R_theta*D_d;
T0_1 = [cos(theta1)                -sin(theta1)                 0                a0;                   
        sin(theta1)*cos(alpha0)  cos(theta1)*cos(alpha0) -sin(alpha0)  -d1*sin(alpha0);
        sin(theta1)*sin(alpha0)  cos(theta1)*sin(alpha0)  cos(alpha0)   d1*cos(alpha0);
        0                              0                              0                1];
T1_2 = [cos(theta2)                -sin(theta2)                 0                a1;                   
        sin(theta2)*cos(alpha1)  cos(theta2)*cos(alpha1) -sin(alpha1)  -d2*sin(alpha1);
        sin(theta2)*sin(alpha1)  cos(theta2)*sin(alpha1)  cos(alpha1)   d2*cos(alpha1);
        0                              0                              0                1];

T2_3 = [cos(theta3)                -sin(theta3)                 0                a2;                   
        sin(theta3)*cos(alpha2)  cos(theta3)*cos(alpha2) -sin(alpha2)  -d3*sin(alpha2);
        sin(theta3)*sin(alpha2)  cos(theta3)*sin(alpha2)  cos(alpha2)   d3*cos(alpha2);
        0                              0                              0                1];

P2_4org = T2_3*P3_4org;

f1 = P2_4org(1);
f2 = P2_4org(2);
f3 = P2_4org(3);
% New defs:
% f1 = a2 + a3*cos(theta3) + d4*cos(alpha3)*sin(theta3);
% f2 = a3*cos(alpha2)*sin(theta3) - d4*sin(alpha2)*sin(alpha3) - d3*sin(alpha2) - d4*cos(alpha2)*cos(alpha3)*cos(theta3)
% f3 = d3*cos(alpha2) + d4*cos(alpha2)*sin(alpha3) + a3*sin(alpha2)*sin(theta3) - d4*cos(alpha3)*sin(alpha2)*cos(theta3)

P0_4org = T0_1*T1_2*P2_4org;
% How to pull out g1, g2 from first two entries?
% g3 = P0_4org(3);

% r = norm(P0_4org(1:3))^2; % Yields some weird abs() insertions
r = P0_4org(1)^2+P0_4org(2)^2+P0_4org(3)^2;

% Substituting the fact that alpha0,a0,d1 are zero (this seemed assumed in the
% text, because (base) = (1) frame)
r = subs(r,a0,0);
r = subs(r,alpha0,0);
r = subs(r,d1,0);
% Result:
% r = a1^2 + a2^2 + a3^2 + d2^2 + d3^2 + d4^2 + 2*d2*d3*cos(alpha2) + 2*a1*a2*cos(theta2) + 2*a2*a3*cos(theta3) + 2*d3*d4*sin(alpha3) + 2*d2*d4*cos(alpha2)*sin(alpha3) + 2*a2*d4*cos(alpha3)*sin(theta3) + 2*a1*a3*cos(theta2)*cos(theta3) + 2*a1*d3*sin(alpha2)*sin(theta2) + 2*a3*d2*sin(alpha2)*sin(theta3) - 2*d2*d4*cos(alpha3)*sin(alpha2)*cos(theta3) + 2*a1*d4*cos(alpha3)*cos(theta2)*sin(theta3) + 2*a1*d4*sin(alpha2)*sin(alpha3)*sin(theta2) - 2*a1*a3*cos(alpha2)*sin(theta2)*sin(theta3) + 2*a1*d4*cos(alpha2)*cos(alpha3)*cos(theta3)*sin(theta2)


% Substituting the fact that a1 is zero, leads to k3=r
k3 = subs(r,a1,0); 


% % Result: omitting abs()
% k31 = (d1 + d2*cos(alpha1) + d3*cos(alpha1)*cos(alpha2) + a2*sin(alpha1)*sin(theta2) + d4*cos(alpha1)*cos(alpha2)*sin(alpha3) + a3*cos(alpha1)*sin(alpha2)*sin(theta3) - d3*sin(alpha1)*sin(alpha2)*cos(theta2) + a3*sin(alpha1)*cos(theta3)*sin(theta2) - d4*sin(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta2) + a3*cos(alpha2)*sin(alpha1)*cos(theta2)*sin(theta3) + d4*cos(alpha3)*sin(alpha1)*sin(theta2)*sin(theta3) - d4*cos(alpha1)*cos(alpha3)*sin(alpha2)*cos(theta3) - d4*cos(alpha2)*cos(alpha3)*sin(alpha1)*cos(theta2)*cos(theta3))^2 ;
% k32 = (a2*cos(theta2)*sin(theta1) - d2*sin(alpha1)*cos(theta1) - d3*cos(alpha2)*sin(alpha1)*cos(theta1) + a2*cos(alpha1)*cos(theta1)*sin(theta2) + a3*cos(theta2)*cos(theta3)*sin(theta1) + d3*sin(alpha2)*sin(theta1)*sin(theta2) - d4*cos(alpha2)*sin(alpha1)*sin(alpha3)*cos(theta1) - d3*cos(alpha1)*sin(alpha2)*cos(theta1)*cos(theta2) + a3*cos(alpha1)*cos(theta1)*cos(theta3)*sin(theta2) - a3*sin(alpha1)*sin(alpha2)*cos(theta1)*sin(theta3) + d4*cos(alpha3)*cos(theta2)*sin(theta1)*sin(theta3) + d4*sin(alpha2)*sin(alpha3)*sin(theta1)*sin(theta2) - a3*cos(alpha2)*sin(theta1)*sin(theta2)*sin(theta3) - d4*cos(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta1)*cos(theta2) + d4*cos(alpha3)*sin(alpha1)*sin(alpha2)*cos(theta1)*cos(theta3) + a3*cos(alpha1)*cos(alpha2)*cos(theta1)*cos(theta2)*sin(theta3) + d4*cos(alpha1)*cos(alpha3)*cos(theta1)*sin(theta2)*sin(theta3) + d4*cos(alpha2)*cos(alpha3)*cos(theta3)*sin(theta1)*sin(theta2) - d4*cos(alpha1)*cos(alpha2)*cos(alpha3)*cos(theta1)*cos(theta2)*cos(theta3))^2;
% k33 = (a2*cos(theta1)*cos(theta2) + d2*sin(alpha1)*sin(theta1) + d3*cos(alpha2)*sin(alpha1)*sin(theta1) + a3*cos(theta1)*cos(theta2)*cos(theta3) - a2*cos(alpha1)*sin(theta1)*sin(theta2) + d3*sin(alpha2)*cos(theta1)*sin(theta2) + d4*cos(alpha2)*sin(alpha1)*sin(alpha3)*sin(theta1) + d3*cos(alpha1)*sin(alpha2)*cos(theta2)*sin(theta1) + d4*cos(alpha3)*cos(theta1)*cos(theta2)*sin(theta3) + d4*sin(alpha2)*sin(alpha3)*cos(theta1)*sin(theta2) - a3*cos(alpha1)*cos(theta3)*sin(theta1)*sin(theta2) - a3*cos(alpha2)*cos(theta1)*sin(theta2)*sin(theta3) + a3*sin(alpha1)*sin(alpha2)*sin(theta1)*sin(theta3) - d4*cos(alpha1)*cos(alpha3)*sin(theta1)*sin(theta2)*sin(theta3) + d4*cos(alpha2)*cos(alpha3)*cos(theta1)*cos(theta3)*sin(theta2) + d4*cos(alpha1)*sin(alpha2)*sin(alpha3)*cos(theta2)*sin(theta1) - d4*cos(alpha3)*sin(alpha1)*sin(alpha2)*cos(theta3)*sin(theta1) - a3*cos(alpha1)*cos(alpha2)*cos(theta2)*sin(theta1)*sin(theta3) + d4*cos(alpha1)*cos(alpha2)*cos(alpha3)*cos(theta2)*cos(theta3)*sin(theta1))^2;
% 
% k3 = k31+k32+k33;
k3 = expand(k3);
k3 = simplify(k3)


% %% Finalized: 
% k3 = A3*cos(theta3) + B3*sin(theta3)+D3 = r =>
% A3*cos(theta3) + B3*sin(theta3) = C3
% 	• Constant coeff
% 	D3 = (a2^2 + a3^2 + d2^2 + d3^2 + d4^2 + 2*d2*d3*cos(alpha2) + 2*d3*d4*sin(alpha3) + 2*d2*d4*cos(alpha2)*sin(alpha3))
% 	C3 = r-D3
% 	• C3 coeff
% 	A3 = (2*a2*a3 - 2*d2*d4*cos(alpha3)*sin(alpha2))
% 	
% 	• S3 coeff
%   B3 = (2*a2*d4*cos(alpha3) + 2*a3*d2*sin(alpha2))
%% MANUAL method 2 (sym fi)
% P3_4org = [a3; -d4*cos(alpha3); d4*sin(alpha3); 1];
% % R_alpha = planarRotation(alpha2,'x');
% % D_a = axialTranslation(a2,'x');
% % R_theta = planarRotation(theta3,'z');
% % D_d = axialTranslation(d3,'z');
% % 
% % T2_3 = R_alpha*D_a*R_theta*D_d;
% T0_1 = [cos(theta1)                -sin(theta1)                 0                a0;                   
%         sin(theta1)*cos(alpha0)  cos(theta1)*cos(alpha0) -sin(alpha0)  -d1*sin(alpha0);
%         sin(theta1)*sin(alpha0)  cos(theta1)*sin(alpha0)  cos(alpha0)   d1*cos(alpha0);
%         0                              0                              0                1];
% T1_2 = [cos(theta2)                -sin(theta2)                 0                a1;                   
%         sin(theta2)*cos(alpha1)  cos(theta2)*cos(alpha1) -sin(alpha1)  -d2*sin(alpha1);
%         sin(theta2)*sin(alpha1)  cos(theta2)*sin(alpha1)  cos(alpha1)   d2*cos(alpha1);
%         0                              0                              0                1];
% 
% T2_3 = [cos(theta3)                -sin(theta3)                 0                a2;                   
%         sin(theta3)*cos(alpha2)  cos(theta3)*cos(alpha2) -sin(alpha2)  -d3*sin(alpha2);
%         sin(theta3)*sin(alpha2)  cos(theta3)*sin(alpha2)  cos(alpha2)   d3*cos(alpha2);
%         0                              0                              0                1];
% 
% P2_4org = T2_3*P3_4org;
% 
% f1 = P2_4org(1);
% f2 = P2_4org(2);
% f3 = P2_4org(3);
% % New defs:
% % f1 = a2 + a3*cos(theta3) + d4*cos(alpha3)*sin(theta3);
% % f2 = a3*cos(alpha2)*sin(theta3) - d4*sin(alpha2)*sin(alpha3) - d3*sin(alpha2) - d4*cos(alpha2)*cos(alpha3)*cos(theta3)
% % f3 = d3*cos(alpha2) + d4*cos(alpha2)*sin(alpha3) + a3*sin(alpha2)*sin(theta3) - d4*cos(alpha3)*sin(alpha2)*cos(theta3)
% 
% 
% syms f1s f2s f3s
% P2_4org_sym = [f1s; f2s; f3s; 1];
% P0_4org = T0_1*T1_2*P2_4org_sym;
% r = P0_4org(1)^2+P0_4org(2)^2+P0_4org(3)^2;
% r = subs(r,a0,0);
% r = subs(r,alpha0,0);
% r = expand(r);
% r = simplify(r)
% 

%% Method 3 (sym gis)
% syms f1s f2s f3s theta2 d2 alpha1 a1
% g1 = cos(theta2)*f1s - sin(theta2)*f2 + a1;
% g2 = sin(theta2)*cos(alpha1)*f1 + cos(theta2)*cos(alpha1)*f2 - sin(alpha1)*f3 - d2*sin(alpha1);
% g3 = sin(theta2)*sin(alpha1)*f1 + cos(theta2)*sin(alpha1)*f2 + cos(alpha1)*f3 + d2*cos(alpha1);
% 
% r = g1^2 + g2^2 + g3^2;
% r = expand(r);
% r = simplify(r)


%% Checking transformation matrix math
% syms a sa ca d st ct
% 
% Rx = [1 0 0 0;
%       0 ca -sa 0;
%       0 sa ca 0;
%       0 0 0 1]; % alpha
% Dx = [1 0 0 a;
%       0 1 0 0;
%       0 0 1 0;
%       0 0 0 1]; % a
% Rz = [ct -st 0 0;
%       st ct 0 0;
%       0 0 1 0;
%       0 0 0 1]; % theta
% Dz = [1 0 0 0;
%       0 1 0 0;
%       0 0 1 d;
%       0 0 0 1]; % d
% 
% T = Rx*Dx*Rz*Dz
% % RESULT:
% % T =
% % [    ct,   -st,   0,     a]
% % [ ca*st, ca*ct, -sa, -d*sa]
% % [ sa*st, ct*sa,  ca,  ca*d]
% % [     0,     0,   0,     1]
