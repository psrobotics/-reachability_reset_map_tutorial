%% get reset map mode ver
function [reset_map, params] = get_reset_map(grid, params)

% define state jump rules for all states, modify this based on your system
x1_post_f = @(x_t,o_params) -x_t(1)/2;
x2_post_f = @(x_t,o_params) x_t(2);
x3_post_f = @x_f3;
state_fcn_arr = {x1_post_f; x2_post_f; x3_post_f};
params.state_fcn_arr = state_fcn_arr;

N = grid.N; % grid num vector
% how many elements in grid, = N(1)*N(2)*N(3)
ind = 1:prod(N); 
% generate 3d indices https://www.mathworks.com/help/matlab/ref/ind2sub.html
% If It's a n-dims system, change to [I1, I2, I3, ... In], 
% and change following
[I1, I2, I3] = ind2sub(N, ind); 

I1_reset = I1; % indectors for reset map dim1
I2_reset = I2; % dim2
I3_reset = I3; % dim3

for j = ind % loop through the reset map

    i_tmp = [I1(j); I2(j); I3(j)];
    i_post = i_tmp;

    % check if this state satisify the state reset condition
    if resetmap_trigger_event(grid, i_tmp, params.o_params)
        x_tmp = index2state(grid,i_tmp);
        x_post = x_tmp;
        % if so, calculate states after state jumping
        for k = 1:length(state_fcn_arr)
            x_post(k) = state_fcn_arr{k}(x_tmp,params.o_params);
        end
        % convert back to corresponding grid indices
        i_post = state2index(grid, x_post);
    end

    I1_reset(j) = i_post(1);
    I2_reset(j) = i_post(2);
    I3_reset(j) = i_post(3);
end
% reshape back to 1d indices
reset_map = sub2ind(N, I1_reset, I2_reset, I3_reset); 
end

% state jump rule for 3rd dim, yaw angle
function x3_post = x_f3(x_t,o_params)
% keep range of yaw angle within -pi~pi
if x_t(3) <= 0
   x3_post = x_t(3) + pi;
else
   x3_post = x_t(3) - pi;
end
end

function is_reset = resetmap_trigger_event(grid, i_t, o_params)
% check if the state reset condition has been triggered for current index
% 1 - reset event triggered, 0 - others
is_reset = 0;
eps = 1e-5;
% get state for this index
x_ref = index2state(grid, i_t);
% check based on current state
if abs(x_ref(2))<eps && x_ref(3)<eps
    is_reset = 1;
end
% for multiple reset conditions
% if condition_1 || condition_2 || ...condition_n
%    is_reset = 1;
% end
end

function i_t = state2index(grid, x_t)
% convert current state to grid indices
i_t = ceil((x_t-grid.min)./grid.dx)+1;
i_t=min([max([ones(grid.dim,1) i_t],[],2) grid.N],[],2); %keep inside index bounds
end

function x_t = index2state(grid, i_t)
% convert current grid index to state
% grid_dx = (grid.max-grid.min)./grid.N;
x_t = (i_t-1).*grid.dx + grid.min;
end