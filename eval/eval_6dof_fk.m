%% evaluation of the 6dof forward kinematics problem solution
% forward kinematics for original configuration

% robot definition 
ROBOT = 'GP50';
robot=robotproperty(ROBOT);
theta_ini = [pi/20   -pi/2    pi/20   pi/20    pi/20   pi/20]';

cpre = ForKine(theta_ini, robot.DH, robot.base, robot.cap);

%% sampling to verify the joint bound exact forward kinematics
% made sure cpre.x = 1.7421 

% precomputed lmd as the joint bound 
% lmd = 0.0332;
lmd = 0.0809;
sample_num = 10000;
xpos_samples = zeros(10000,1);
% sampling a y vector within [-1,1]

for i = 1:sample_num
    ys = -1 + 2*rand(6,1);
    epos = ForKine_jointbound(theta_ini, robot.DH, robot.base, robot.cap,ys,lmd);
    xpos_samples(i) = epos(1);
end

figure
plot(xpos_samples,'o','lineWidth',2);
hold on 
% plot the solidline to demonstrate 1.8
yline = 1.8 * ones(10000,1);
plot(yline,'-','lineWidth',2);
hold on 
% limitation 
ylim([0 1.9]);


%% sampling to verify the joint bound approximated forward kinematics
% precomputed lmd as the joint bound 
% safe distance is 1.8 m 
% dist = 1.8;
% lmd = 0.3283; % feasibility = e-3
% % lmd = 0.0607; % feasibility = e-12
% % lmd = 0.0328; % feasibility = e-14

% safe distance is 1.83 m 
dist = 1.83;
% lmd = 0.3595; % feasibility = e-3
% lmd = 0.0844; % feasibility = 5.992e-12
lmd = 0.0639; % feasibility = 3.479e-14

sample_num = 10000;
xpos_approx_samples = zeros(10000,1);
% sampling a y vector within [-1,1]

for i = 1:sample_num
    ys = -1 + 2*rand(6,1);
    epos = ForKine_jointbound_approx(theta_ini, robot.DH, robot.base, robot.cap,ys,lmd);
    xpos_approx_samples(i) = epos(1);
end

figure
plot(xpos_approx_samples,'o','lineWidth',2);
hold on 
% plot the solidline to demonstrate 1.8
yline = dist * ones(10000,1);
plot(yline,'-','lineWidth',2);
hold on 
% limitation 
ylim([0 1.9]);