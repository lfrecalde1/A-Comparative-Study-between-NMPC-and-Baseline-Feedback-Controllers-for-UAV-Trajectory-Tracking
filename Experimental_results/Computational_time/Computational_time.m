%% Code to compare the controllers
% Clean variables
clc, clear all, close all;
 
% Load variables of the system
load("Kinematic_time.mat");
load("Dynamic_time.mat");
load("NMPC_time.mat");

% Resize variable
dt_dynamic = dt_dynamic(1:length(dt_NMPC));
dt_kinematic = dt_kinematic(1:length(dt_NMPC));
t = t(1:length(dt_NMPC));
% Get average value of each sample time

avg_kinematic = mean(dt_kinematic)*1000
avg_dynamic = mean(dt_dynamic)*1000
avg_NMPC = mean(dt_NMPC)*1000


