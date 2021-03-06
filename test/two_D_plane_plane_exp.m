%% the 2D plane with 2D link experiments
clc
clear
% define the auxiliary variables 
syms lmd y1 y2 real;
nlink = 2;

% robot configurations
theta1 = pi/3;
theta2 = pi/6;

% forward kinematics 
xpos = cos(theta1) + cos(theta2);
ypos = sin(theta1) + sin(theta2);

% arbitrary wall: y <= x + bias
% max_b = 2*(sqrt(2)/2  + sqrt(2)/2); % 2.8284
% min_b = xpos + ypos; % 2.7321
bias = 2.8; % 2.7321 < 2.8 < 2.8284

%% automatic decomposition tool 
% Y vector = [1 y1 y2];
% forward kinematics within joint toleranc, and without lmd 
xfk = cossym(theta1, y1) + cossym(theta2, y2);
yfk = sinsym(theta1, y1) + sinsym(theta2, y2);

% refute set polynomials
% construct the coefficient and monomoials
% f1 = xfk - xwall;
f1 = xfk + yfk - bias;
[c,t] = coeffs(f1);
deci_coe = vpa(c,3);
% decompose to Y^T Q Y = f1
% decide which term
% differentiation to determine the order 
tic
for i = 1:2
    eval(['gys' num2str(i) '= diff(t,y' num2str(i) ');'])
end
% convert to real number
for i = 1:2
    eval(['gys' num2str(i) '= double(subs(gys' num2str(i) ', {y1,y2}, {1,1}));']);
end
% assign the Q matrix
for num = 1:size(t,2) 
    % determine the order sequence 
    weight = eval(deci_coe(num));
    order_seq = [];
    for i =  1:nlink
        eval(['order_seq = [order_seq gys' num2str(i) '(num)];'])
    end
    
    % get the row and column and weight, directly using the switch case
    % since it is too simple, Y = [1,y1,y2]
    disp(order_seq);
    if order_seq(2) == 2
        % y2^2
        Q(3,3) = weight;
        continue;
    end
    if order_seq(1) == 2
        % y1^2
        Q(2,2) = weight;
        continue;
    end
    if order_seq(2) == 1
        % y2
        Q(1,3) = weight/2;
        continue;
    end
    if order_seq(1) == 1
        % y1
        Q(1,2) = weight/2;
        continue;
    end
    if order_seq(1) == 0 && order_seq(2) == 0
        % y1
        Q(1,1) = weight;
        continue;
    end
end

% make the Q matrix as symmetric 
dim = 3; % due to Y = [1 y1 y2];
for i = 1:dim
    for j = 1:i-1
        assert(j < i);
        Q(i,j) = Q(j,i);
    end
end

% auxiliary decomposition of the constraints for y1, y2
Q1 = [1 0 0; 0 -1 0; 0 0 0];
Q2 = [1 0 0; 0 0 0; 0 0 -1];
Q_cons = [1 0 0; 0 0 0; 0 0 0];


%% SOS formulation to formulate the nonlinear constraints
options = optimoptions('fmincon','Display','iter','Algorithm','SQP');
% options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
obj = @(x)-x(1);

A = [];
b = [];
normalize = 0; % whether to use the normalization trick

rng(1);
xref = [0 100*rand(1,3)]; % lmdb b c d

% non-normalized formulation 
if normalize == 0
    LB = [0, 1, 1, 1];
    UB = [];
    [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy_2D(x,Q,Q1,Q2,Q_cons),options);
end

% normalized trick formulation 
if normalize == 1
    LB = [0, 0, 0, 0];
    UB = [];
    [x, fval, exitflag,output] = fmincon(obj,xref,[],[],A,b,LB,UB,@(x)nonlcon_hierarchy_2D_normalize(x,Q,Q1,Q2,Q_cons),options);
end

toc

function c = cossym(t, y)
    c = cos(t)*(1 - y^2/2) - sin(t)*y;
end

function s = sinsym(t, y)
    s = sin(t)*(1 - y^2/2) + cos(t)*y;
end