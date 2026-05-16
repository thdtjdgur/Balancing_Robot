#include <math.h>
#include "mppi.h"
#include "variable.h"
#include "lidar.h"
#include "gnss.h"      // GNSS에서 계산한 gnss_x, gnss_y, goal_x, goal_y 사용
#include "esp_log.h"
#include "cost.h"
#include "waypoint.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"


#define MPPI_MAX_HORIZON 10
#define MPPI_MAX_SAMPLES 64

static const char *TAG = "MPPI";

MPPI_Params mppi_params;

static MPPI_State current_state; //로봇 상태를 담아낼 변수

static MPPI_Input prev_applied_input = {0.0f, 0.0f};                     //직전 실제 적용 입력
static MPPI_Input best_sequence[MPPI_MAX_HORIZON];                       //직전 최종 명령표
static MPPI_Input base_sequence[MPPI_MAX_HORIZON];                       //이번 샘플링 기준 명령표
static MPPI_Input sampled_sequences[MPPI_MAX_SAMPLES][MPPI_MAX_HORIZON]; //평가해볼 후보명령표를 저장할 배열
static float sequence_costs[MPPI_MAX_SAMPLES];                           //각 후보 명령줄의 총 비용 계산저장배열
static float sequence_weights[MPPI_MAX_SAMPLES];                         //sequence_costs를 바탕으로 계산한 가중치 저장 배열

static int sequence_initialized = 0;



//값이 최소/최대 범위를 넘지 않게 자르는 함수
static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}


//미래상태 예측 함수
//현재 상태와 현재 명령(v_ref, w_ref)을 바탕으로
//dt 뒤의 다음 상태를 예측하는 함수
static MPPI_State predict_next_state(const MPPI_State *state, const MPPI_Input *input)
{
    MPPI_State next_state = *state;

    // MPPI가 만든 목표 전진속도와 목표 각속도를
    // 로봇이 허용 가능한 범위 안으로 제한
    float v_cmd = clampf_local(input->v_ref, mppi_params.v_min, mppi_params.v_max);
    float w_cmd = clampf_local(input->w_ref, mppi_params.w_min, mppi_params.w_max);

    // 단순 운동모델:
    // 로봇이 명령 속도와 명령 각속도를 즉시 따른다고 가정
    next_state.v = v_cmd;
    next_state.w = w_cmd;

    // 현재 heading 방향을 기준으로 dt 동안 이동한 다음 위치 계산
    next_state.x += next_state.v * cosf(state->psi) * mppi_params.dt;
    next_state.y += next_state.v * sinf(state->psi) * mppi_params.dt;

    // 각속도 * 시간 만큼 heading이 변한다고 보고 다음 heading 계산
    next_state.psi = wrap_to_pi(state->psi + next_state.w * mppi_params.dt);

    return next_state;
}



//직전 최적 명령표를 한 칸 앞으로 당기는 역할
static void build_base_sequence(int horizon)
{
    if (!sequence_initialized) {
        // 아직 이전 최적 명령표가 없으면 현재 목표값으로 전체 명령표를 채움
        for (int t = 0; t < horizon; t++) {
            base_sequence[t].v_ref = targetvel_vel;
            base_sequence[t].w_ref = target_yaw_diff;
        }
        return;
    }

    // 이전 최적 명령표를 한 칸 앞으로 당겨서 새 기준 명령표로 사용
    for (int t = 0; t < horizon - 1; t++) {
        base_sequence[t] = best_sequence[t + 1];
    }

    // 마지막 칸은 이전 최적 명령표의 마지막 입력을 유지
    base_sequence[horizon - 1] = best_sequence[horizon - 1];
}


//명령표에 다양한 후보 시퀸스를 만들기 위한 -a~+a범위의 작은 노이즈를 만드는 함수
static float rand_symmetric(float amplitude)
{
    float r = (float)rand() / (float)RAND_MAX;   // 0 ~ 1
    return (2.0f * r - 1.0f) * amplitude;        // -amplitude ~ +amplitude
}



//기준 명령표 base_sequence에 노이즈를 섞어서 후보 명령표 하나를 만드는 함수
//sample_input_sequence_from_base(sampled_sequences[i], base_sequence, horizon);
static void sample_input_sequence_from_base(MPPI_Input *dst,
                                            const MPPI_Input *base,
                                            int horizon)
{
    float v_amp = sequence_initialized ? 0.08f : 0.30f;
    float w_amp = sequence_initialized ? 0.25f : 0.80f;
    //처음 실행할때 정지 시퀸스 주변만 보지 말고 좀 더 넓게 탐색

    float noise_v = rand_symmetric(v_amp);
    float noise_w = rand_symmetric(w_amp);

    for (int t = 0; t < horizon; t++) {
        // 시간적으로 완전히 독립이 아니라 조금 연속된 노이즈가 되도록 만듦
        noise_v = 0.7f * noise_v + 0.3f * rand_symmetric(v_amp);
        noise_w = 0.7f * noise_w + 0.3f * rand_symmetric(w_amp);

        dst[t].v_ref = clampf_local(base[t].v_ref + noise_v,
                                    mppi_params.v_min,
                                    mppi_params.v_max);

        dst[t].w_ref = clampf_local(base[t].w_ref + noise_w,
                                    mppi_params.w_min,
                                    mppi_params.w_max);
    }
}


//명령줄 하나에 대한 총 비용을 계산하는 함수
//evaluate_input_sequence(&current_state, sampled_sequences[i], horizon);
static float evaluate_input_sequence(const MPPI_State *start_state,
                                     const MPPI_Input *sequence,
                                     int horizon)
{
    float total_cost = 0.0f;
    MPPI_State pred_state = *start_state;
    MPPI_Input prev_input = prev_applied_input;

    for (int t = 0; t < horizon; t++) {
        pred_state = predict_next_state(&pred_state, &sequence[t]);

        total_cost += calc_goal_cost(&pred_state);
        total_cost += calc_sector_obstacle_cost(&pred_state, start_state);
        total_cost += calc_input_cost(&sequence[t]);
        total_cost += calc_smooth_cost(&sequence[t], &prev_input);

        prev_input = sequence[t];
    }

    return total_cost;
}


//각 명령줄의 비용을 가중치로 바꾸는 함수
//compute_sequence_weights(sequence_costs, sequence_weights, num_samples);
static void compute_sequence_weights(const float *costs, float *weights, int num_samples)
{
    float min_cost = costs[0];
    float weight_sum = 0.0f;

    for (int i = 1; i < num_samples; i++) {
        if (costs[i] < min_cost) {
            min_cost = costs[i];
        }
    }

    //exp(-(J_i - J_min)/lambda)
    for (int i = 0; i < num_samples; i++) {
        weights[i] = expf(-(costs[i] - min_cost) / mppi_params.lambda);
        weight_sum += weights[i];
    }

    if (weight_sum > 0.0f) {//0으로 나누는 상황 방지
        for (int i = 0; i < num_samples; i++) {
            weights[i] /= weight_sum;
            //각 가중치를 총합으로 나눔. 그러면 모든 가중치 합은 1이 됨
        }
    }
}


//모든 후보를 가중평균해서 최종 명령줄을 만드는 함수야.
//compute_weighted_sequence(best_sequence, sampled_sequences, sequence_weights, num_samples, horizon);
static void compute_weighted_sequence(MPPI_Input *out_sequence,
                                      MPPI_Input sequences[MPPI_MAX_SAMPLES][MPPI_MAX_HORIZON],
                                      const float *weights,
                                      int num_samples,
                                      int horizon)
{
    for (int t = 0; t < horizon; t++) {
        float v_sum = 0.0f;
        float w_sum = 0.0f;

        for (int i = 0; i < num_samples; i++) {
            v_sum += weights[i] * sequences[i][t].v_ref;
            w_sum += weights[i] * sequences[i][t].w_ref;
        }

        out_sequence[t].v_ref = clampf_local(v_sum, mppi_params.v_min, mppi_params.v_max);
        out_sequence[t].w_ref = clampf_local(w_sum, mppi_params.w_min, mppi_params.w_max);
    }
}


// 현재 센서값을 MPPI_State 형식으로 묶는 함수
// 지금은 GNSS에서 계산된 x,y와 현재 yaw, 속도를 그대로 사용함
static MPPI_State get_current_state(void)
{
    MPPI_State state;

    state.x = gnss_x;         // 현재 로봇의 로컬 x 위치 [m]
    state.y = gnss_y;         // 현재 로봇의 로컬 y 위치 [m]
    state.psi = current_yaw;  // 현재 로봇 heading [rad]
    state.v = current_vel;    // 현재 로봇 전진속도 [m/s]
    state.w = gyro;           // 현재 로봇 회전속도 [rad/s]

    return state;
}

//*****로그 출력 완료하면 지우기
static void send_cost_log_hc06(const MPPI_State *scan_origin_state,//*****
                               const MPPI_State *pred_state,
                               float label_cost)
{
    static int csv_header_sent = 0;
    static const char *csv_header =
        "scan_x,scan_y,scan_psi,pred_x,pred_y,"
        "d0,d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11,d12,d13,d14,d15,d16,d17,d18,d19,d20,d21,d22,d23,"
        "a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,a18,a19,a20,a21,a22,a23,"
        "label_cost\n";

    const float *d = get_sector_distance_array();
    const float *a = get_sector_angle_array();

    char tx_buf[2048];
    int len = 0;

    if (!csv_header_sent) {
        uart_write_bytes(UART_NUM_2, csv_header, strlen(csv_header));
        csv_header_sent = 1;
    }

    len += snprintf(tx_buf + len, sizeof(tx_buf) - len,
                    "%.6f,%.6f,%.6f,%.6f,%.6f,",
                    scan_origin_state->x,
                    scan_origin_state->y,
                    scan_origin_state->psi,
                    pred_state->x,
                    pred_state->y);

    for (int i = 0; i < MPPI_NUM_SECTORS; i++) {
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len,
                        "%.6f,", d[i]);
    }

    for (int i = 0; i < MPPI_NUM_SECTORS; i++) {
        len += snprintf(tx_buf + len, sizeof(tx_buf) - len,
                        "%.6f,", a[i]);
    }

    len += snprintf(tx_buf + len, sizeof(tx_buf) - len,
                    "%.6f\n", label_cost);

    uart_write_bytes(UART_NUM_2, tx_buf, len);
}



float wrap_to_pi(float angle)
{
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}


void init_MPPI(void)
{
    //기본값 세팅
    mppi_params.weight_goal_x = 1.0f;
    mppi_params.weight_goal_y = 1.0f;
    //mppi_params.weight_heading = 0.5f;
    mppi_params.weight_obstacle = 2.0f;

    mppi_params.weight_smooth_v = 0.3f;
    mppi_params.weight_smooth_w = 0.3f;

    mppi_params.weight_input_v = 0.1f;
    mppi_params.weight_input_w = 0.1f;

    mppi_params.dt = 0.1f;
    mppi_params.horizon = 6;//6샘플 앞을 관찰
    mppi_params.num_samples = 64;

    mppi_params.v_min = -0.5f;
    mppi_params.v_max = 0.5f;
    mppi_params.w_min = -1.2f;
    mppi_params.w_max = 1.2f;

    mppi_params.lambda = 10.0f;

    mppi_params.obs_safe_dist = 0.5f; //0.5m

    ESP_LOGI(TAG, "MPPI initialized");
}


void MPPI_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        int horizon = mppi_params.horizon;
        int num_samples = mppi_params.num_samples;

        if (horizon > MPPI_MAX_HORIZON) {
            horizon = MPPI_MAX_HORIZON;
        }

        if (num_samples > MPPI_MAX_SAMPLES) {
            num_samples = MPPI_MAX_SAMPLES;
        }

        waypoint_update();

        if (!waypoint_is_active()) {//처음, 끝났을때 말고는 여기 안들어옴
            targetvel_vel = 0.0f;
            target_yaw_diff = 0.0f;
            sequence_initialized = 0;
            prev_applied_input.v_ref = 0.0f;
            prev_applied_input.w_ref = 0.0f;

            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
            continue;
        }

        // 현재 상태 읽기
        current_state = get_current_state();

        // 현재 라이다를 섹터 정보로 압축
        build_lidar_sectors();

        // 직전 최적 명령표를 한 칸 당겨서 새 기준 명령표 생성
        build_base_sequence(horizon);

        // 후보 명령표 여러 개 생성 및 비용 계산
        for (int i = 0; i < num_samples; i++) {
            sample_input_sequence_from_base(sampled_sequences[i],
                                            base_sequence,
                                            horizon);

            sequence_costs[i] = evaluate_input_sequence(&current_state,
                                                        sampled_sequences[i],
                                                        horizon);
        }

        // 비용 -> 가중치 변환
        compute_sequence_weights(sequence_costs,
                                 sequence_weights,
                                 num_samples);

        // 가중평균으로 최종 명령표 생성
        compute_weighted_sequence(best_sequence,
                                  sampled_sequences,
                                  sequence_weights,
                                  num_samples,
                                  horizon);
        
        MPPI_State log_pred_state = predict_next_state(&current_state, &best_sequence[0]);//*****로그 출력 완료하면 지우기
        float log_cost = calc_sector_obstacle_cost(&log_pred_state, &current_state);//*****로그 출력 완료하면 지우기
        send_cost_log_hc06(&current_state, &log_pred_state, log_cost);//*****로그 출력 완료하면 지우기


        // 최종 명령표의 첫 번째 입력만 실제 적용
        targetvel_vel = best_sequence[0].v_ref;
        target_yaw_diff = best_sequence[0].w_ref;

        prev_applied_input = best_sequence[0];
        sequence_initialized = 1;

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}