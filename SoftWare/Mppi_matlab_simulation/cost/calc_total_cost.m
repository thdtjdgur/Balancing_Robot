function total_cost = calc_total_cost(state, input, prev_input, goal, obstacles, params)
%calc_total_cost(pred_state, sequence(t), last_input, goal, obstacles, params);

goal_cost = calc_goal_cost(state, goal, params);
obstacle_cost = calc_obstacle_cost(state, obstacles, params);
input_cost = calc_input_cost(input, params);
smooth_cost = calc_smooth_cost(input, prev_input, params);

total_cost = goal_cost + obstacle_cost + input_cost + smooth_cost;

end





function cost = calc_goal_cost(state, goal, params)

error_x = goal.x - state.x;
error_y = goal.y - state.y;

cost = params.weight_goal_x * error_x^2 + params.weight_goal_y * error_y^2;

end





function cost = calc_obstacle_cost(state, obstacles, params)

cost = 0.0;

num_sectors = 24;
sector_width = 2*pi / num_sectors;%섹터 하나의 각도폭 계산

sector_min_dist = inf(num_sectors, 1);
%inf함수: 24행 1열 배열을 만들고 전부 inf로 채움. inf는 무한대라는 뜻
%각 섹터에서 가장 가까운 장애물 거리를 저장할 배열

for i = 1:size(obstacles, 1)%장애물 개수만큼 반복문 반복
    dx = obstacles(i, 1) - state.x;
    dy = obstacles(i, 2) - state.y;

    dist_to_obstacle = sqrt(dx^2 + dy^2);
    %로봇 예측위치와 장애물까지의 거리계산

    angle_world = atan2(dy, dx);%월드 좌표계에서 로봇에서 장애물을 바라보는 각도 계산
    angle_robot = wrap_to_pi(angle_world - state.psi);
    %월드 기준 장애물 각도에서 로봇 heading state.psi를 빼서, 로봇 기준 상대각으로 바꿈.
    angle_0_to_2pi = mod(angle_robot, 2*pi);%상대각을 0 ~ 2*pi 범위로 바꿈.

    sector_idx = floor(angle_0_to_2pi / sector_width) + 1;
    %장애물이 로봇 기준으로 봤을때 몇번째 섹터에 있는지 계산

    if sector_idx < 1
        sector_idx = 1;
    elseif sector_idx > num_sectors
        sector_idx = num_sectors;
    end

    if dist_to_obstacle < sector_min_dist(sector_idx)
        sector_min_dist(sector_idx) = dist_to_obstacle;
        %현재 장애물이 그 섹터에 저장된 기존 장애물보다 더 가까우면 그 섹터의 가장 가까운 장애물 거리로 갱신
    end
end

for s = 1:num_sectors
    if isinf(sector_min_dist(s))
        continue;
        %s번째 섹터 값이 아직 inf면, 즉 그 섹터에 장애물이 하나도 없으면 이 섹터 건너뛰고 다음 섹터로 감
    end

    if sector_min_dist(s) < params.obs_safe_dist
        %그 섹터의 가장 가까운 장애물이 안전거리보다 가까우면
        error = params.obs_safe_dist - sector_min_dist(s);
        %안전거리보다 얼마나 침범했는지 침범량 계산.
        cost = cost + params.weight_obstacle * error^2;
        %침범량의 제곱에 장애물 가중치를 곱해서 cost에 추가.
    end
end

end





function cost = calc_input_cost(input, params)

cost_v = params.weight_input_v * input.v_ref^2;
cost_w = params.weight_input_w * input.w_ref^2;

cost = cost_v + cost_w;

end





function cost = calc_smooth_cost(input, prev_input, params)

dv = input.v_ref - prev_input.v_ref;
dw = input.w_ref - prev_input.w_ref;

cost_v = params.weight_smooth_v * dv^2;
cost_w = params.weight_smooth_w * dw^2;

cost = cost_v + cost_w;

end