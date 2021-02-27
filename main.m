clear all
close all
clc

%% Redundancy:

l = [340 200 200 200 200 126 4];

% Joints' limits for KUKA IIWA robot
jointsRange = deg2rad([-170 170; -120 120; -170 170; -120 120; -170 170; -120 120; -175 175]);

q = [50 40 20 30 40 10 10]'; % initial configuration
% final_config = [90 50 40 20 80 70 20]';

%% Trajectory:
nPoints = 40;
radius = sum(l) - 350;
angle1 = linspace(-15,15,nPoints);
angle2 = linspace(0,360*10,nPoints);
x = radius*cosd(angle1);
y = radius*sind(angle1);
z = ones(1, nPoints)*340; %  + 10*sind(angle2);

% current_p = FK(l, q, 0);
% [x, y] = generateSquare(current_p(1), current_p(2), 200, nPoints/4);
% z = 340*ones(1,nPoints);

trajectory = [x; y; z; zeros(1, nPoints); ones(1, nPoints)*pi/2; atan2(y,x)];

%% Weighted Pseudo Inverse Solution
W = diag([0.5 0.5 0.5 0.75 0.2 0.2 0.2]);
[all_configs, all_positions] = PseudoInverseIK(q, trajectory, l, 1, W);

%% Damped Least Square:
% myo = 1.5;
% all_configs = DampedLeastSquares(q, trajectory, l, myo)

%% Null Space Solution:
% objective = "man";
% [all_configs, all_positions] = NullSpaceIK(q, trajectory, l, 1, objective, jointsRange);

%% Task Augmentation:
% current_p = FK(l, q, 0);
% [x, y] = generateSquare(current_p(1), current_p(2), 200, nPoints/4);
% z = 340*ones(1,nPoints);
% TaskAugmentation(q, l, x, y, z, nPoints/4)

%% Comparison:
% [all_configs, all_positions] = Comparison(q, trajectory, l, 1, objective, jointsRange);

nConfigs = 4;

% plot3(all_positions(1,end-10:end), all_positions(2,end-10:end), all_positions(3,end-10:end), '.','Color', '1 0 0' ,'MarkerSize',7.5)

for i=1:7
    FK(l, cell2mat(all_configs(end-i*15)), 1, '1 0 1', '1 0 0');
end

FK(l, cell2mat(all_configs(end)), 1, '0 0 0', '1 0 0');