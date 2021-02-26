clear all
close all
clc

%% Redundancy:

l = [340 200 200 200 200 126 4];

% Joints' limits for KUKA IIWA robot
jointsRange = deg2rad([-170 170; -120 120; -170 170; -120 120; -170 170; -120 120; -175 175]);

q = [20 10 10 20 10 10 10]'; % initial configuration
% final_config = [90 50 40 20 80 70 20]';

%% Trajectory:
nPoints = 40;
radius = 300;
angle1 = linspace(0,360,nPoints);
angle2 = linspace(0,360*10,nPoints);
% x = radius*sind(angle1);
% y = radius*cosd(angle1);
% z = 750 + 10*sind(angle2);

current_p = FK(l, q, 0);
[x, y] = generateSquare(current_p(1), current_p(2), 400, nPoints/4);
z = 750*ones(1,nPoints);

trajectory = [x; y; z; ones(1, nPoints)*current_p(4); ones(1, nPoints)*current_p(5); ones(1, nPoints)*current_p(6)];

%% Pseudo Inverse Solution
% [all_configs, all_positions] = PseudoInverseIK(q, trajectory, l, 1);

%% Null Space Solution:
objective = "man";
% [all_configs, all_positions] = NullSpaceIK(q, trajectory, l, 1, objective, jointsRange);

%% Comparison:
[all_configs, all_positions] = Comparison(q, trajectory, l, 1, objective, jointsRange);