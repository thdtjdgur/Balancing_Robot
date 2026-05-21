clear; clc; close all;%이전 작업 잔여데이터를 모두 초기화

addpath(genpath(pwd));%현재 폴더(mppi_matlab_sim)와 하위 폴더 전체를 MATLAB 함수 검색 경로에 추가

params = init_mppi_params();%init_mppi_params.m 함수를 실행해서 mppi파라미터 구조체를 받아와서 params에 저장

%로봇이 움직일 공간크기 설정
max.x_min = -10;
max.x_max = 10;
max.y_min = -10;
max.y_max = 10;

%state라는 구조체 생성, 초기위치설정
state.x = -10;
state.y = -10;
state.psi = 0;
state.v = 0;
state.w = 0;

%최종 목적지 위치 설정
goal.x = 10;
goal.y = 10;

%장애물 위치 설정
obstacles = [
    -6.0 -6.0;
    -3.0 -3.0;
     0.0  0.0;
     3.0  3.0;
     6.0  6.0;
    -4.8 -3.7;
    -3.8 -5.0;
    -1.2 -0.4;
    -0.3 -1.6;
     1.1  0.6;
     2.2  1.8;
     5.4  4.8;
    -7.2 -5.2;
    -6.5 -2.0;
    -5.2 -0.6;
    -2.8 -5.4;
    -2.4 -1.8;
    -1.6  2.0;
     0.7 -4.2;
     0.4  2.7;
     2.1 -2.8;
     3.4 -0.8;
     4.8  0.8;
     5.9  2.2;
     6.8 -2.6;
     7.6  4.0;
     8.8  1.1;

    -7.5  8.5;
    -5.0  9.6;
    -2.5  8.8;
     0.0  9.7;
     2.5  8.6;
     5.0  9.4;
     7.0  8.2;

    -6.0  3.0;
    -3.0  1.0;
     1.0  4.0;
     4.5  2.0;
     8.0 -1.5;

    -8.0 -4.0;
    -4.0 -6.5;
     0.0 -3.0;
     3.5 -7.0;
     7.5 -5.0
];

%직전 입력값 초기화
prev_input.v_ref = 0;
prev_input.w_ref = 0;

best_sequence = [];%이전 최적 명령열을 빈 배열로 초기화
sequence_initialized = false;%이전 최적 명령열 유무

history = [];%로봇이 지나온 경로와 입력 기록을 저장할 빈 배열

max_steps = 900;
goal_tolerance = 0.5;
draw_interval = 5;

for k = 1:max_steps %시뮬레이션 루프. k가 1부터 150까지 반복됨. dt=0.1이면 실제 시뮬기산은 15초
    [input, best_sequence, sequence_initialized, debug] = mppi_step( ...
        state, goal, obstacles, prev_input, best_sequence, sequence_initialized, params);
    %현재 로봇상태, 목표점, 장애물, 이전입력, 이전 최적 명령열, 파라미터를 mppi_step함수에 넣고
    %이번주기에 실제 적용할 명령, 새로 계산된 최적 명령열, 다음루프에서 이전 명령열이 있다는 표시등을 저장함
    %즉 cost함수를 쭉 실행해서 이번에 적용할 입력을 계산하는 함수임

    state = predict_next_state(state, input, params);
    %입력이 선정되었으면 실제 로봇위치 이동

    prev_input = input;

    history = [history; state.x, state.y, state.psi, input.v_ref, input.w_ref];
    goal_distance = hypot(goal.x - state.x, goal.y - state.y);
    %x,y,psi,v_ref,w_ref행을 저장
    %x1,y1,psi1,v_ref1,w_ref1
    %x2,y2,psi2,v_ref2,w_ref2

    if mod(k, draw_interval) == 0 || k == 1 || goal_distance < goal_tolerance
        draw_sim(state, goal, obstacles, history, debug, params, k, goal_distance);%로봇이 지나온 자리(history), 목표점. 장애물 들등 표시
        pause(0.001);%0.03초 멈춰서 에니메이션처럼 보이게 함
    end

    if goal_distance < goal_tolerance
        fprintf('Goal reached at step %d, time %.1f s, distance %.3f m\n', ...
            k, k * params.dt, goal_distance);
        break;
    end
end