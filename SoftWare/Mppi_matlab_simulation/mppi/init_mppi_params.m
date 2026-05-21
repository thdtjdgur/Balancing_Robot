function params = init_mppi_params()
params.dt = 0.1;          %0.1초 제어주기
params.horizon = 30;      % 미래행동 1.5초 보려면 15
params.num_samples = 64;  %명령줄 64줄 테스트 

params.v_min = -0.5;
params.v_max = 0.5;
params.w_min = -1.2;
params.w_max = 1.2;

params.lambda = 10.0;

%가중치----------
params.weight_goal_x = 1.5;
params.weight_goal_y = 1.5;
params.weight_obstacle = 120.0;

params.weight_smooth_v = 0.3;
params.weight_smooth_w = 0.3;

params.weight_input_v = 0.1;
params.weight_input_w = 0.1;

params.obs_safe_dist = 1.0;%로봇과 장애물 사이 안전거리
%가중치----------
end