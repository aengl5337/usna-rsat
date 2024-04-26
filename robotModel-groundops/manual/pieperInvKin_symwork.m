% Working out the math symbollically, Craig 2ed pg130

% syms a1 a2 a3 d2 d3 d4 c3 s3 ca2 ca3 sa2 sa3
% 
% f1 = a3*c3 + d4*sa3*s3 + a2;
% f2 = a3*ca2*s3 - d4*sa3*ca2*c3 - d4*sa2*ca3 - d3*sa2;
% f3 = a3*sa2*s3 - d4*sa3*sa2*c3 + d4*ca2*ca3 + d3*ca2;

syms a1 a2 a3 d2 d3 d4 theta3 alpha2 alpha3

f1 = a3*cos(theta3) + d4*sin(alpha3)*sin(theta3) + a2;
f2 = a3*cos(alpha2)*sin(theta3) - d4*sin(alpha3)*cos(alpha2)*cos(theta3) - d4*sin(alpha2)*cos(alpha3) - d3*sin(alpha2);
f3 = a3*sin(alpha2)*sin(theta3) - d4*sin(alpha3)*sin(alpha2)*cos(theta3) + d4*cos(alpha2)*cos(alpha3) + d3*cos(alpha2);

k3 = f1^2 + f2^2 + f3^2 + a1^2 + d2^2 + 2*d2*f3;
k3 = expand(k3);
k3 = simplify(k3)