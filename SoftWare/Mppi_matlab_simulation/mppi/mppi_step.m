function [input, best_sequence, sequence_initialized, debug] = mppi_step( ...
    state, goal, obstacles, prev_input, best_sequence, sequence_initialized, params)

horizon = params.horizon;
num_samples = params.num_samples;

base_sequence = build_base_sequence(best_sequence, sequence_initialized, horizon);
%이번 샘플링의 기준 명령열을 만듬

sampled_sequences = repmat(struct('v_ref', 0, 'w_ref', 0), num_samples, horizon);
%v_ref = 0, w_ref = 0이라는 구조체를 num_samples행, horizon열만큼 만듬. 즉 명령줄 초기화 코드임

sequence_costs = zeros(num_samples, 1);
%num_samples*1크기의 행렬 생성, zeros함수의 특징에 따라 행렬 내부의 모든 데이터는 0으로 채워짐
%즉 각 명령줄의 코스트를 저장할 배열 생성

for i = 1:num_samples%i가 1부터 num_samples까지 증가
    sampled_sequences(i, :) = sample_input_sequence_from_base( ...
        base_sequence, sequence_initialized, params);
    %sample_input_sequence_from_base함수는 기준명령열에 랜덤 노이즈를 섞어서 후보명령줄 하나를 만드는 함수
    %sampled_sequences(i, :)의 의미: i번째 후보 명령줄(행) 전체

    sequence_costs(i) = evaluate_input_sequence( ...
        state, sampled_sequences(i, :), prev_input, goal, obstacles, params);
    %sample_input_sequence_from_base함수에서 계산된 후보명령열 하나에 총 비용 계산
end

sequence_weights = compute_sequence_weights(sequence_costs, params);
%compute_sequence_weights는 각 후보 명령열의 비용을 mppi가중치로 바꾸는 함수

best_sequence = compute_weighted_sequence(sampled_sequences, sequence_weights, params);
%모든 후보 명령열을 weight로 가중평균해서 최종 명령줄 best_sequence를 만드는 함수

input = best_sequence(1);%best_sequence의 첫번째 열을 로봇의 입력으로 사용
sequence_initialized = true;

debug.sampled_sequences = sampled_sequences;
debug.sequence_costs = sequence_costs;
debug.sequence_weights = sequence_weights;
debug.best_sequence = best_sequence;

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function base_sequence = build_base_sequence(best_sequence, sequence_initialized, horizon)

base_sequence = repmat(struct('v_ref', 0, 'w_ref', 0), 1, horizon);

if ~sequence_initialized || isempty(best_sequence)
    return;
    %이전 최적명령열이 없거나 best_sequence가 비어있으면 처음 시작이므로 base_sequence는 전부 0인 상태로 반환
end

for t = 1:horizon-1
    base_sequence(t) = best_sequence(t + 1);
    %이전 최적 명령열을 한칸씩 앞으로 당김
end

base_sequence(horizon) = best_sequence(horizon);
%base_sequence 마지막 칸은 이전 최적 명령열의 마지막 값 그대로 대입
end





function sequence = sample_input_sequence_from_base(base_sequence, sequence_initialized, params)

sequence = base_sequence;

if sequence_initialized%이전 최적 명령열이 있으면 이미 어느 정도 좋은 방향을 알고 있으니까 작은 노이즈만 줌
    v_amp = 0.30;
    w_amp = 0.80;
else%첫 MPPI 실행이면 정보가 없으니까 더 큰 노이즈로 넓게 탐색함
    v_amp = 0.40;
    w_amp = 0.90;
end

noise_v = rand_symmetric(v_amp);
noise_w = rand_symmetric(w_amp);

for t = 1:params.horizon
    noise_v = 0.7 * noise_v + 0.3 * rand_symmetric(v_amp);
    noise_w = 0.7 * noise_w + 0.3 * rand_symmetric(w_amp);

    sequence(t).v_ref = clamp_value( ...
        base_sequence(t).v_ref + noise_v, params.v_min, params.v_max);

    sequence(t).w_ref = clamp_value( ...
        base_sequence(t).w_ref + noise_w, params.w_min, params.w_max);
end

end





function total_cost = evaluate_input_sequence( ...
    start_state, sequence, prev_input, goal, obstacles, params)

total_cost = 0.0;
pred_state = start_state;
last_input = prev_input;

for t = 1:params.horizon
    pred_state = predict_next_state(pred_state, sequence(t), params);%명령줄을 적용했을때 다음 상태 예측

    total_cost = total_cost + calc_total_cost(pred_state, sequence(t), last_input, goal, obstacles, params);
    %명령줄의 특정 열을 적용했을때의 상태에 대한 비용계산

    last_input = sequence(t);
end

end





function weights = compute_sequence_weights(costs, params)

min_cost = min(costs);%각 명령줄 중 가장 작은 값을 min_cost에 대입
weights = exp(-(costs - min_cost) / params.lambda);%각 명령줄에 대한 가중치 계산

weight_sum = sum(weights);%각 명령줄의 가중치 합

if weight_sum > 0
    weights = weights / weight_sum;%weights 합이 1이 되도록 정규화
end

end





function best_sequence = compute_weighted_sequence(sampled_sequences, weights, params)

best_sequence = repmat(struct('v_ref', 0, 'w_ref', 0), 1, params.horizon);%최종 명령열을 0으로 초기화

for t = 1:params.horizon
    v_sum = 0.0;
    w_sum = 0.0;

    for i = 1:params.num_samples
        v_sum = v_sum + weights(i) * sampled_sequences(i, t).v_ref;
        w_sum = w_sum + weights(i) * sampled_sequences(i, t).w_ref;
    end

    best_sequence(t).v_ref = clamp_value(v_sum, params.v_min, params.v_max);
    best_sequence(t).w_ref = clamp_value(w_sum, params.w_min, params.w_max);
end

end