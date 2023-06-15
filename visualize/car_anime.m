function [] = car_anime(x_arr, ctr_arr, t_arr, brs_t_ind_arr, g, data, params, brs_arr, save_video)
% create simulation amine
size_arr = size(x_arr);
len = size_arr(2)-1;

% video sub folder
x_init = x_arr(:,1);
data_file_str = strcat('video\', 'vid_x_');
data_file_str = strcat(data_file_str, num2str(ceil(100*x_init(1))));
data_file_str = strcat(data_file_str, '_y_');
data_file_str = strcat(data_file_str, num2str(ceil(100*x_init(2))));
data_file_str = strcat(data_file_str, '_a_');
data_file_str = strcat(data_file_str, num2str(ceil(100*x_init(3))));

if save_video == 1
    v = VideoWriter(data_file_str,'MPEG-4');  
    v.FrameRate = 10;
    open(v);
end

for k = 1:len
    clf;
    hold on;
    axis equal;
    grid on;

    x_t = x_arr(:,k);
    r_mat = rot_zyx([0,0,x_t(3)]);

    view(30,40);
    %view(0,90)
    axis([-8,8,0,8,0,3]);

    % plot target area
    for rad = 0:0.2:2*pi
        r_mat_t = rot_zyx([0,0,rad]);
        plot_cube(r_mat_t,params.tar_r/1.414,params.tar_r/1.414,1.6,...
            [params.tar_pos(1);params.tar_pos(2);0.1],[146,36,40]./255,1);
    end

    % plot current car pos
    plot_cube(r_mat,1.1,0.6,0.2,[x_t(1);x_t(2);0.1],'black',2);

    % plot past traj
    for i = 1:k
        x_i = x_arr(:,i);
        r_mat_old = rot_zyx([0,0,x_i(3)]);
        plot_cube(r_mat_old,0.2,0.2,0.2,[x_i(1);x_i(2);0.1],[107,76,154]./255,1);
    end

    [X,Y] = meshgrid(g.min(1) : params.grid_dx1*4: g.max(1) ,...
                      0: params.grid_dx1*4: g.max(2) - params.grid_dx1*3);

    size(X)
    size(Y)

    [Xq,Yq] = meshgrid(g.min(1): params.grid_dx1: g.max(1),...
                      0+params.grid_dx1: params.grid_dx1: g.max(2));
    BRT_2d_layer_high_res = griddata(X,Y,brs_arr(:,:,k)',Xq,Yq,"cubic");

    surf(Xq,Yq,-1*BRT_2d_layer_high_res);
    colormap summer;

    pause(params.dt);
    hold on;

    if save_video == 1
        frame = getframe(gcf);
        writeVideo(v,frame);
    end

end

    if save_video == 1
        close(v);
    end

end