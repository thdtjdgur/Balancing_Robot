#include <stdio.h>
#include "encoder.h"
#include "variable.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm.h"
#include <math.h>

// 다른 파일(main.c 등)에서도 사용해야 하는 변수들은 extern으로 선언합니다.
spi_transaction_t t_left;  // 왼쪽 엔코더용 트랜잭션
spi_transaction_t t_right; // 오른쪽 엔코더용 트랜잭션
uint16_t tx_data = 0xFFFF; // 패리티 주소 + 읽기/쓰기(RWn) + (PAR)0x3FFF(각도 주소) 이 정보를 합쳐서 0xFFFF라는 명령어 만듬 [cite: 563, 609]
uint16_t rx_data_left;     // 왼쪽 각도 데이터 빈 상자
uint16_t rx_data_right;    // 오른쪽 각도 데이터 빈 상자
encoder_handles_t enc_ctx;

spi_device_handle_t h_left;  
spi_device_handle_t h_right;

// 통신 완료 동기화를 위한 세마포어 핸들 선언 (WDT 에러 방지용)
SemaphoreHandle_t encoder_sem;

void IRAM_ATTR spi_post_callback(spi_transaction_t *t) {
    //void IRAM_ATTR는 spi_post_callback라는 인터럽트함수가 빠르게 실행될 수 있게 flash메모리에서 ram메모리로 꺼내두는 과정
    // 통신 완료 인터럽트 콜백: 세마포어를 해제하여 app_main의 대기를 풀어줌
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //xHigherPriorityTaskWoken는 요리사b(스레드)가 잠에서 깼는지에 대한 여부의 체크리스트변수. 초기값은 pdFALSE로 할당함.
    xSemaphoreGiveFromISR(encoder_sem, &xHigherPriorityTaskWoken);
    //rtos는 encoder_sem(세마포어)주문서와 imu_sem(세마포어)주문서를 기다리는 요리사(쓰레드) 각각의 우선순위를 확인하고 
    //encoder_sem를 기다리는 요리사의 우선순위가 높기때문에 xHigherPriorityTaskWoken에 true를 적음
    if (xHigherPriorityTaskWoken) {//요리사b(스레드)가 깨어있으면
        portYIELD_FROM_ISR();//rtos가 요리작업대 사용자를 요리사a에서 요리사b로 바로 바꿈
    }
}

static bool timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{//alarm_count가 1000에 도달하면 ESP32의 하드웨어 타이머 컨트롤러가 timer_on_alarm_cb이 함수를 호출함
    encoder_handles_t *handles = (encoder_handles_t *)user_ctx;
    
    // 왼쪽과 오른쪽 엔코더에 동시에 전송 큐를 삽입함
    spi_device_queue_trans(handles->left, &t_left, 0);
    spi_device_queue_trans(handles->right, &t_right, 0);
    
    return false;
}

void encoder_timer_init (void)
{
    gptimer_handle_t gptimer = NULL;//gptimer변수 선언 후 아래 코드부터 쭉 gptimer설정코드임
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Select the default clock source
        .direction = GPTIMER_COUNT_UP,      // Counting direction is up
        .resolution_hz = 1 * 1000 * 1000,   // 1초에 타이머가 몇번 올라갈것인가
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,      // When the alarm event occurs, the timer will automatically reload to 0
        .alarm_count = 1000, // Set the actual alarm period, since the resolution is 1us, 1000은 1ms (1KHz)
        .flags.auto_reload_on_alarm = true, // Enable auto-reload function
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = { .on_alarm = timer_on_alarm_cb };
    //alarm_count가 1000에 도달해 알람이 울리면 timer_on_alarm_cb함수를 실행하겠다라는 내용을 cbs라는 구조체에 적음
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, &enc_ctx));
    //cbs라는 구조체를 이용해 gptimer에게 등록하는 행위
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    // Start the timer
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}

void encoder_init(void)//엔코더 초기설정
{
    // 세마포어라는 신호의 존재를 만들고 초기화
    encoder_sem = xSemaphoreCreateBinary();

    spi_bus_config_t buscfg = {
        .miso_io_num = 13,    
        .mosi_io_num = 11,    
        .sclk_io_num = 12,       
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .mode = 1,
        //엔코더데이터시트 11페이지를 보면 clk는 0부터 시작해서 CPOL = 0,
        //MOSI나 MISO 선에 있는 데이터 비트의 중심이 CLK의 내려가는선에 맞춰져있어서 CPHA = 1, 따라서 모드는 1임
        .clock_speed_hz = 10000000,//10MHz(spi통신박자)(엔코더 데이터시트 12페이지에 TCLK는 최소 100ns = 10MHz라고 되어있음)
        .queue_size = 7,
        .spics_io_num = 9, // 왼쪽 엔코더 CS 핀 [cite: 167]
        .clock_source = SPI_CLK_SRC_DEFAULT, 
        //clock_source 멤버는 SPI 통신 클럭을 생성하기 위해 어떤 뿌리(Source) 전원을 사용할지 결정하는 옵션
        // 시스템이 알아서 최적의 시계를 선택하도록 함
        .cs_ena_pretrans = 4,
        //엔코더 데이터시트 12페이지를 보면 mcu에서 CSn을 내리고난 후 첫번째 클럭신호가 rising이 되기까지의 시간이 최소 350ns라고 되어있음(tl)
        //.cs_ena_pretrans = 4,이 코드의 의미는 현재 10MHz이므로 1클럭당 100ns가 됨. 즉 100ns를 4번 더 보내는거니까 400ns만큼 기다림.
        .post_cb = spi_post_callback,// 통신 완료 인터럽트 콜백
    };

    // 오른쪽 엔코더 설정 (CS 핀만 다르게 설정, 나머지는 왼쪽과 동일)
    spi_device_interface_config_t devcfg_right = devcfg;
    devcfg_right.spics_io_num = 10; // 오른쪽 엔코더 CS 핀 (10번으로 가정)

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &h_left);
    spi_bus_add_device(SPI2_HOST, &devcfg_right, &h_right);

    //전송 구조체 세부 설정 (왼쪽)
    t_left.length = 16;
    t_left.tx_buffer = &tx_data;//0xFFFF라는 변수의 주소값 전달
    t_left.rx_buffer = &rx_data_left;//엔코더가 준 값이 이 주소의 변수에 저장됨

    //전송 구조체 세부 설정 (오른쪽)
    t_right.length = 16;
    t_right.tx_buffer = &tx_data;//0xFFFF라는 변수의 주소값 전달
    t_right.rx_buffer = &rx_data_right;//엔코더가 준 값이 이 주소의 변수에 저장됨

    // 콜백 함수에 두 핸들을 모두 전달하기 위해 구조체에 담음
    enc_ctx.left = h_left;
    enc_ctx.right = h_right;
}

void encoder_to_vcc_cal(void)
{
    // 출력 속도를 조절하기 위한 카운터
    static int print_count = 0;
    spi_transaction_t *ret_l, *ret_r;  // ret_t 실체 추가
    
    spi_device_get_trans_result(h_left, &ret_l, 0);
    spi_device_get_trans_result(h_right, &ret_r, 0);

    // 빅엔디안 센서 데이터를 리틀엔디안 MCU에 맞게 바이트 스왑 (MSB first 대응)
    uint16_t raw_l = (rx_data_left << 8) | (rx_data_left >> 8);
    uint8_t error_flag_l = (raw_l & 0x4000) >> 14;

    //spi_device_get_trans_result는 1khz인터럽트에 의해 spi_device_queue_trans함수가 각도를 요청하고 각도를 받아올때까지 기다리는 함수임 
    float angle_l = raw_l & 0x3FFF; // [수정] 스왑된 데이터에서 14비트 추출
    angle_l = angle_l * 360.0f/16384.0f;//14해상도이기 때문에 360 % 2^14 [cite: 262, 236]
    angle_l = angle_l * 2.0f * M_PI /360.0f;//각도를 라디안으로 변경

    // 오른쪽 엔코더 데이터도 바이트 스왑 적용
    //엔코더에서 빋엔디안방식으로 msb부터 보냄. mcu에서는 데이터를 저장할때 낮은자리부터 저장하는방식을 사용함
    //즉 엔코더에서 0xabcd를 보내면 mcu에서 받았을 경우 0xcdab로 저장됨. 즉 하위 8비트 상위 8비트를 서로 바꿔줘야힘.
    uint16_t raw_r = (rx_data_right << 8) | (rx_data_right >> 8);
    uint8_t error_flag_r = (raw_r & 0x4000) >> 14;

    float angle_r = raw_r & 0x3FFF; // [수정] 스왑된 데이터에서 14비트 추출
    angle_r = angle_r * 360.0f/16384.0f;//원시데이터를 도단위로 변환
    angle_r = angle_r * 2.0f * M_PI /360.0f;


    // 센서 에러 플래그(EF, Bit 14) 확인 시 연산 중단 [cite: 589]
    //if ((raw_l & 0x4000) || (raw_r & 0x4000)) return;

    // 100번 호출될 때마다 1번만 출력 (1ms * 100 = 100ms 간격)
    if (print_count++ >= 10) {
        //printf("L_Angle: %f | R_Angle: %f\n", angle_l, angle_r);//라디안 각도값
        //printf("L_Error: %d, R_Error: %d\n", error_flag_l, error_flag_r);
        //printf("\n");
        print_count = 0;
    }

    /*
    static float prev_angle_l = 0, prev_angle_r = 0;
    // 각속도 계산 (rad/s)
    float omega_l = (angle_l - prev_angle_l) / 0.001f;
    float omega_r = (angle_r - prev_angle_r) / 0.001f;
    // 선속도 계산 (m/s) = 평균 각속도 * 반지름
    current_vel = ((omega_l + omega_r) / 2.0f) * WHEEL_RADIUS; 
    //printf("current_vel: %f\n", current_vel);

    prev_angle_l = angle_l; 
    prev_angle_r = angle_r;
    */
    
    static int vel_calc_counter = 0;
    static float prev_angle_l = 0.0f;
    static float prev_angle_r = 0.0f;
    
    vel_calc_counter++;
    
    if (vel_calc_counter >= 10) { // 1KHz 루프가 10번 돌 때마다(10ms) 딱 1번만 진입
        vel_calc_counter = 0;
        
        // 1. dt를 0.010f(10ms)로 나누어 노이즈를 1/10로 축소시킵니다.
        float diff_l = angle_l - prev_angle_l;
        // 각도 차이가 +180도(+PI)보다 크면, 뒤로 한 바퀴 돈 것!
        if (diff_l > M_PI) diff_l -= 2.0f * M_PI;
        // 각도 차이가 -180도(-PI)보다 작으면, 앞으로 한 바퀴 돈 것!
        else if (diff_l < -M_PI) diff_l += 2.0f * M_PI;
        float omega_l = diff_l / 0.010f;

        float diff_r = angle_r - prev_angle_r;
        if (diff_r > M_PI) diff_r -= 2.0f * M_PI;
        else if (diff_r < -M_PI) diff_r += 2.0f * M_PI;
        float omega_r = diff_r / 0.010f;
        
        // 2. 날것의 선속도(raw_vel) 계산
        float raw_vel = ((omega_l - omega_r) / 2.0f) * WHEEL_RADIUS; 
        
        // 3. 아주 가벼운 로우패스 필터 적용 (과거 70% + 현재 30%)
        // 주기를 늘렸기 때문에 이 정도만 걸어도 지연 없이 노이즈가 완벽히 사라집니다.
        current_vel = (0.5f * current_vel) + (0.5f * raw_vel);
        
        prev_angle_l = angle_l; 
        prev_angle_r = angle_r;

        vel_calc_flag = 1;
    }

    // 테스트용 전압 입력 (Vq_l, Vq_r는 나중에 PID 결과값으로 대체됨)
    float Vq_l = -Vq_left; //-Vq_left
    float Vq_r = Vq_right; //Vq_right
    float Vd = 0.0f; //0.0f

    // GM4108H-120T 모터의 극 쌍(Pole Pairs) 수 적용 --> 기계적 각도를 전기적 각도로 변환
    int pole_pairs = 11;
    //float ele_angle_l = 0;
    //float ele_angle_r = 0;

    float offset_l = 2.7756f;//2.775
    float offset_r = 1.6345f;//1.632

    float ele_angle_l = (angle_l - offset_l) * (float)pole_pairs;
    float ele_angle_r = (angle_r - offset_r) * (float)pole_pairs;

    // [왼쪽 바퀴] 역변환 (Park + Clarke) - 전기적 각도 적용
    float V_alpha_l = Vd * cosf(ele_angle_l) - Vq_l * sinf(ele_angle_l);
    float V_beta_l  = Vd * sinf(ele_angle_l) + Vq_l * cosf(ele_angle_l);

    float Va_l = V_alpha_l;
    float Vb_l = -0.5f * V_alpha_l + (sqrtf(3.0f) / 2.0f) * V_beta_l;
    float Vc_l = -0.5f * V_alpha_l - (sqrtf(3.0f) / 2.0f) * V_beta_l;

    // [오른쪽 바퀴] 역변환 (Park + Clarke) - 전기적 각도 적용
    float V_alpha_r = Vd * cosf(ele_angle_r) - Vq_r * sinf(ele_angle_r);
    float V_beta_r  = Vd * sinf(ele_angle_r) + Vq_r * cosf(ele_angle_r);

    float Va_r = V_alpha_r;
    float Vb_r = -0.5f * V_alpha_r + (sqrtf(3.0f) / 2.0f) * V_beta_r;
    float Vc_r = -0.5f * V_alpha_r - (sqrtf(3.0f) / 2.0f) * V_beta_r;

    mcpwm_set_voltage(0, Va_l);
    mcpwm_set_voltage(1, Vb_l);
    mcpwm_set_voltage(2, Vc_l);

    mcpwm_set_voltage(3, Va_r);
    mcpwm_set_voltage(4, Vb_r);
    mcpwm_set_voltage(5, Vc_r);

    // 결과 확인용 출력
    //printf("L_Va: %.2f | R_Va: %.2f\n", Va_l, Va_r);
}