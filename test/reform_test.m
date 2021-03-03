%% the 2D plane with 2D link experiments
clc
clear
% define the auxiliary variables 
syms lmd y1 y2 real;
nlink = 2;

% robot configurations
theta1 = pi/3;
theta2 = pi/6;
xdist = 0.09;
ydist = 0.05;

% forward kinematics 
xpos = cos(theta1) + cos(theta2);
ypos = sin(theta1) + sin(theta2);
xwall = xpos + xdist;
ywall = ypos + ydist;

x0 = [theta1, theta2];
%% SOS formulation to formulate the nonlinear constraints
options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
obj = @(x)norm(x-x0,inf);

A = [];
b = [];

% rng(1);
% xref = rand(1,2);%[0 0]; % lmdb b c d
xref = [0 0];

% non-normalized formulation 
LB = [];
UB = [];
[x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_reform(x,xwall),options);
lmd = norm(x-x0,inf)


