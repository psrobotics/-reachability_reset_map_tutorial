function [brs_arr] = get_2d_brt_vis(x_arr, ctr_arr, t_arr, brs_t_ind_arr, g, data, params)

% create simulation amine
size_arr = size(x_arr);
len = size_arr(2)-1;

% get brt overlay
brs_arr = zeros(20,10,len);

ind = 1:20*10;
[I1, I2] = ind2sub([20,10], ind);

% pre-compute all brs at defined timestep
for k = 1:len
for j = 1:ind(end)
    i1 = I1(j); % current element indector
    i2 = I2(j);

    brs_arr(i1,i2,k) = eval_u(g, data(:,:,:,brs_t_ind_arr(k)),...
        [(i1-1)*params.grid_dx1*4+g.min(1), (i2-1)*params.grid_dx1*4, x_arr(3,k)]);
    fprintf('analysis 2D BRT index %d of %d, timestep %d of %d \n',j,ind(end),k,len);
end
end