%% Setup path to have no conflict
addpath(fullfile('src'));
addpath(fullfile('Deliverable_4_2'),"-begin")
warning('off','MATLAB:rmpath:DirNotFound')
rmpath(fullfile('Deliverable_3_1'))
rmpath(fullfile('Deliverable_3_2'))
rmpath(fullfile('Deliverable_4_1'))
rmpath(fullfile('Deliverable_4_2'))
rmpath(fullfile('Deliverable_5_1'))

clear; close all;

%% This file should produce all the plots for the deliverable