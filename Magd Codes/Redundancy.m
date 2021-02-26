clear all
close all
clc

%% Redundancy:

l = [340 200 200 200 200 126 4];
jointsRange = deg2rad([30 90; 0 360; 0 360; 0 360; 0 360; 0 360; 0 360]);

q = [20 10 10 20 10 10 10]'; % initial configuration
final_config = [90 50 40 20 80 70 20]';

%% Pseudo Inverse Solution
% [all_configs, all_positions] = PseudoInverseIK(q, final_config, l);

%% Null Space Solution:
objective = "range";
[all_configs, all_positions] = NullSpaceIK(q, final_config, l, objective, jointsRange);