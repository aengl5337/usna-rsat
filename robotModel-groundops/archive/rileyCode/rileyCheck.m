close all
clear all
format compact
clc

libPath = 'G:\My Drive\RSat master\GDriveSoftwareTransfer\Software\MATLAB';
addpath(libPath);

%%
% Comparing symbolically Riley's math (which flips motors 4,5) to the
% standard approach
syms th4 th5 x y z

l3 = 182.02;

Rth4 = planarRotation(th4,'z');
Rth5 = planarRotation(th5,'z');
Tl3 = axialTranslation(l3,'z');
Rxp = planarRotation(180,'x');
Rxn = Rxp';

v5 = [x; y; z; 1];

v3_riley = Rth5*Tl3*Rxn*Rth4*Rxp*v5;

v3_std = Rth4*Tl3*Rxn*Rth5*Rxp*v5;


%% 
O5r = subs(v3_riley, {x,y,z},[0,0,0]);
O5s = subs(v3_std, {x,y,z},[0,0,0]);