clear all
close all
clc

%% Redundancy:

l = [1 1 1 1];
jointsRange = [pi/6 pi/2; -pi/2 pi/2; -pi/2 pi/2; -pi/2 pi/2];
q = [50 0 1 1]'; % initial configuration
final_config = [50 50 50 50]';

%% Pseudo Inverse Solution
[all_configs, all_positions] = PseudoInverseIK(q, final_config, 100, l);

%% Null Space Solution:
% objective = "range";
% [all_configs, all_positions] = NullSpaceIK(q, final_config, 100, l, objective, jointsRange);