function [x_arr, ctr_arr, t_arr, brs_t_ind_arr] = car_sim_single_traj(car_sys, x_init, grid, data, tau, params)
% sim opti traj of the car
x_arr = x_init;
t_arr = [];
ctr_arr = [];
brs_t_ind_arr = [];
dt = tau(2)-tau(1);

for t_n = tau
    t_arr = [t_arr, t_n];
    x_n = x_arr(:,end);
    % check if reaches target
    tar_state = check_in_target(x_n,params.tar_pos,params.tar_r);
    if tar_state == 1
        break;
    end

    % get current brs
    % determine the earliest time that the current state is in the reachable set
    t_earliest_i = floor(tau(end)/dt);
    for t_i = 1:t_earliest_i
        value_at_x = eval_u(grid, data(:,:,:, t_i), x_n);
        if value_at_x<0
            t_earliest_i = t_i;
            break;
        end
    end
    brs_t_ind_arr = [brs_t_ind_arr, t_earliest_i];
    brs_at_t = data(:,:,:,t_earliest_i);
    deriv_t = computeGradients(grid, brs_at_t);
    deriv_n = eval_u(grid, deriv_t, x_n);
    % get optimal control
    opti_ctr_t = car_sys.optCtrl([],[],deriv_n,'min');
    ctr_arr = [ctr_arr, opti_ctr_t];
    % sim next step
    x_next = car_sys.updateStateWithResetMap(opti_ctr_t, dt, x_n, [], params);
    x_arr = [x_arr, x_next];

    fprintf('simulating timestep %f\n', t_n);
end

end

%% check if the car reaches target area
function [in_target] = check_in_target(car_state, tar_pos, tar_r)
    x = car_state(1);
    y = car_state(2);
    dist = sqrt((x-tar_pos(1))^2+(y-tar_pos(2))^2) - tar_r;

    if dist<0
        in_target = 1;
    else
        in_target = 0;
    end
end