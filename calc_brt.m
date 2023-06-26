clear all
close all

addpath(genpath('toolbox\helperOC\'));
addpath(genpath('toolbox\ToolboxLS\'));
addpath(genpath('data\'));
addpath(genpath('visualize\'));
addpath(genpath('reset_map_fcns\'));

%% generate target fcn area
R = 2;
M = 40;
params.grid_dx1 = 8 * R / (2 * M);
params.grid_dx3 = 2*pi/(2*M);

params.grid_min = [-4*R + params.grid_dx1; -4*R + params.grid_dx1; -pi];
params.grid_max = [-4*R + params.grid_dx1*2*M; -4*R + params.grid_dx1*2*M; pi];
N = [2*M, 2*M, 2*M]; % M/2 corresponds to pi/2 for 3rd axes
pdDims = 3; 
grid = createGrid(params.grid_min, params.grid_max, N, pdDims);

% set 3d target fcn
g_tmp_min = params.grid_min(1:2);
g_tmp_max = params.grid_max(1:2);
N_tmp = [N(1); N(2)];
g_tmp = createGrid(g_tmp_min, g_tmp_max, N_tmp);

% define target set
target_area_r = 0.5;
target_area_pos = [0;4];

data0 = zeros(N);
for i=1:N(3)
    %ToolboxLS\Kernel\InitialConditions\SetOperations, Get combined shape
    data0(:,:,i) = shapeSphere(g_tmp,target_area_pos,target_area_r);
end

%%
params.v = 2; % forward_velparams.R = R; % cycle_r
params.u_bound = 1; % yaw input u limit
params.R = R; % cycle_r
params.alpha = 1.0;

% other params for state rest condition
params.o_params = [];

%% solver setup
schemeData.grid = grid;
schemeData.uMode = 'min'; % control trying to min the cost fcn, reachable set

schemeData.dynSys = hybird_mod_dubins_car([], [], params); % create dyn model

schemeData.accuracy = 'high';
schemeData.hamFunc = @genericHam;
schemeData.partialFunc = @genericPartial;
[schemeData.dissFunc, ~, schemeData.derivFunc] = ...
    getNumericalFuncs('global', schemeData.accuracy);

HJIextraArgs.stopConverge = 1;
HJIextraArgs.targetFunction = data0;

HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; % set figure number
HJIextraArgs.visualize.deleteLastPlot = true; % delete previous plot as you update

%% sim time
t0 = 0;
dt = 0.1;
t_max = 6;
tau = t0:dt:t_max;

%% get reset map
[schemeData.reset_map, params] = get_reset_map(grid, params);

%% cal brt
[data, tau, extraOuts] = ...
        HJIPDE_solve_with_reset_map(data0, tau, schemeData, 'minVWithL', HJIextraArgs);

%% store the brt
data_file_str = strcat('data\data_with_reset_map_', num2str(100*2));
data_file_str = strcat(data_file_str, '_t_');
data_file_str = strcat(data_file_str, num2str(t_max));
save(strcat(data_file_str, '.mat'), 'grid', 'data0', 'params', 'data', 'tau'); % save data
%%
function [dissFunc, integratorFunc, derivFunc] = getNumericalFuncs(dissType, accuracy)
% Dissipation
switch(dissType)
    case 'global'
        dissFunc = @artificialDissipationGLF;
    case 'local'
        dissFunc = @artificialDissipationLLF;
    case 'locallocal'
        dissFunc = @artificialDissipationLLLF;
    otherwise
        error('Unknown dissipation function %s', dissType);
end

% Accuracy
switch(accuracy)
    case 'low'
        derivFunc = @upwindFirstFirst;
        integratorFunc = @odeCFL1;
    case 'medium'
        derivFunc = @upwindFirstENO2;
        integratorFunc = @odeCFL2;
    case 'high'
        derivFunc = @upwindFirstENO3;
        integratorFunc = @odeCFL3;
    case 'veryHigh'
        derivFunc = @upwindFirstWENO5;
        integratorFunc = @odeCFL3;
    otherwise
        error('Unknown accuracy level %s', accuracy);
end
end