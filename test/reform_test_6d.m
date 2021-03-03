%% the 2D plane with 2D link experiments
clc
clear
% define the auxiliary variables 
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
theta_ini = [pi/20   -pi/2    pi/20   pi/20    pi/20   pi/20]';
seed = 0;
% no lmd forward kinematics
% c_pre = ForKine_sym_nolmd(theta_ini, robot.DH, robot.base, robot.cap, ys);

x0 = theta_ini;
%% SOS formulation to formulate the nonlinear constraints
options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
obj = @(x)norm(x-x0,inf);

A = [];
b = [];

% rng();
% xref = 2*pi*rand(1,6);%[0 0]; % lmdb b c d
xwall = 1.8;
xref = x0;

% non-normalized formulation 
LB = [];
UB = [];
[x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_reform_6d(x,xwall),options);
lmd = norm(x-x0,inf)


