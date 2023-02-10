
/************************************************************************************
FILE			:roboteq_HAL
DESCRIPTION		:roboteq control functions
*************************************************************************************/
/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "uartRingBufDMA.h"
#include "roboteq_HAL.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "math.h"

/* Private typedef -----------------------------------------------------------*/
GPIO_TypeDef* GPIO_PORT[] = {
		LEFTSEL_GPIO_Port,
		RIGHTSEL_GPIO_Port,
		DIRSEL_GPIO_Port,
		LED_GRN_GPIO_Port,
		AREA1_GPIO_Port,
		AREA2_GPIO_Port,
		AREA3_GPIO_Port,
		AREA4_GPIO_Port,
		LED_RED_GPIO_Port,
		RLY1_GPIO_Port,
		RLY2_GPIO_Port,
		RLY3_GPIO_Port,
		RLY4_GPIO_Port,
		RLY5_GPIO_Port,
		RLY6_GPIO_Port,
		ONLED_GPIO_Port,
		ALARM_GPIO_Port,
		SDLED_GPIO_Port,
		RSTLED_GPIO_Port,
		F1LED_GPIO_Port,
		F2LED_GPIO_Port,
		F3LED_GPIO_Port,
		F4LED_GPIO_Port,
		LED_ORG_GPIO_Port,
		WREN_GPIO_Port};

const uint16_t GPIO_PIN[] = {
		LEFTSEL_Pin,
		RIGHTSEL_Pin,
		DIRSEL_Pin,
		LED_GRN_Pin,
		AREA1_Pin,
		AREA2_Pin,
		AREA3_Pin,
		AREA4_Pin,
		LED_RED_Pin,
		RLY1_Pin,
		RLY2_Pin,
		RLY3_Pin,
		RLY4_Pin,
		RLY5_Pin,
		RLY6_Pin,
		ONLED_Pin,
		ALARM_Pin,
		SDLED_Pin,
		RSTLED_Pin,
		F1LED_Pin,
		F2LED_Pin,
		F3LED_Pin,
		F4LED_Pin,
		LED_ORG_Pin,
		WREN_Pin};

typedef enum
{
	LEFTSEL = 0,
	RIGHTSEL = 1,
	DIRSEL = 2,
	LED_GRN = 3,
	AREA1 = 4,
	AREA2 = 5,
	AREA3 = 6,
	AREA4 = 7,
	LED_RED = 8,
	RLY1 = 9,
	RLY2 = 10,
	RLY3 = 11,
	RLY4 = 12,
	RLY5 = 13,
	RLY6 = 14,
	ONLED = 15,
	ALARM = 16,
	SDLED = 17,
	RSTLED = 18,
	F1LED = 19,
	F2LED = 20,
	F3LED = 21,
	F4LED = 22,
	LED_ORG = 23,
	WREN = 24
}Pt_TypeDef;

const float r_coefficient_1st_meter_theta3[5][12] = {
    {3.5, 3.95, 2.55, 2.12, 1.92, 3.3, 1.7, 1.65, 1.62, 1.59, 1.57, 1.44}, // y =0
    {3.5, 4.00, 2.55, 2.12, 1.92, 3.03, 1.7, 1.65, 1.62, 1.59, 1.57, 1.43},// y =0.005
    {2.4, 2.07, 1.87, 1.75, 1.67, 2.8, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.01
    {2.4, 2.07, 1.87, 1.75, 1.67, 2.62, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.015
    {1.45, 1.43, 1.42, 1.425, 1.455, 2.45, 1.56, 1.61, 1.68, 1.75, 1.79, 1.74}	// y = 0.02
};
const float r_coefficient_1st_meter_theta2[5][12] = {
    {3.5, 3.95, 2.55, 2.12, 1.92, 4.85, 1.7, 1.65, 1.62, 1.59, 1.57, 1.44}, // y =0
    {3.5, 4.00, 2.55, 2.12, 1.92, 4.25, 1.7, 1.65, 1.62, 1.59, 1.57, 1.43},// y =0.005
    {2.4, 2.07, 1.87, 1.75, 1.67, 3.8, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.01
    {2.4, 2.07, 1.87, 1.75, 1.67, 3.45, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.015
    {1.45, 1.43, 1.42, 1.425, 1.455, 3.15, 1.56, 1.61, 1.68, 1.75, 1.79, 1.74}	// y = 0.02
};
const float r_coefficient_1st_meter_theta1[5][12] = {
    {3.5, 3.95, 2.55, 2.12, 1.92, 9.5, 1.7, 1.65, 1.62, 1.59, 1.57, 1.44}, // y =0
    {3.5, 4.00, 2.55, 2.12, 1.92, 7.35, 1.7, 1.65, 1.62, 1.59, 1.57, 1.43},// y =0.005
    {2.4, 2.07, 1.87, 1.75, 1.67, 6.06, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.01
    {2.4, 2.07, 1.87, 1.75, 1.67, 5.16, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.015
    {1.45, 1.43, 1.42, 1.425, 1.455, 4.5, 1.56, 1.61, 1.68, 1.75, 1.79, 1.74}	// y = 0.02
};
const float r_coefficient_1st_meter_theta0[5][12] = {
    {3.5, 3.95, 2.55, 2.12, 1.92, 0, 1.7, 1.65, 1.62, 1.59, 1.57, 1.44}, // y =0
    {3.5, 4.00, 2.55, 2.12, 1.92, 16, 1.7, 1.65, 1.62, 1.59, 1.57, 1.43},// y =0.005
    {2.4, 2.07, 1.87, 1.75, 1.67, 16, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.01
    {2.4, 2.07, 1.87, 1.75, 1.67, 10.9, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.015
    {1.45, 1.43, 1.42, 1.425, 1.455, 8.25, 1.56, 1.61, 1.68, 1.75, 1.79, 1.74}	// y = 0.02
};

const float r_coefficient_1st_meter_theta3_N[5][12] = {
    {3.5, 3.95, 2.55, 2.12, 1.92, 3.3, 1.7, 1.65, 1.62, 1.59, 1.57, 1.44}, // y =0
    {3.5, 4.00, 2.55, 2.12, 1.92, 3.63, 1.7, 1.65, 1.62, 1.59, 1.57, 1.43},// y =0.005
    {2.4, 2.07, 1.87, 1.75, 1.67, 4.05, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.01
    {2.4, 2.07, 1.87, 1.75, 1.67, 4.55, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.015
    {1.45, 1.43, 1.42, 1.425, 1.455, 5.22, 1.56, 1.61, 1.68, 1.75, 1.79, 1.74}	// y = 0.02
};
const float r_coefficient_1st_meter_theta2_N[5][12] = {
    {3.5, 3.95, 2.55, 2.12, 1.92, 4.85, 1.7, 1.65, 1.62, 1.59, 1.57, 1.44}, // y =0
    {3.5, 4.00, 2.55, 2.12, 1.92, 5.6, 1.7, 1.65, 1.62, 1.59, 1.57, 1.43},// y =0.005
    {2.4, 2.07, 1.87, 1.75, 1.67, 6.65, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.01
    {2.4, 2.07, 1.87, 1.75, 1.67, 8.35, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.015
    {1.45, 1.43, 1.42, 1.425, 1.455, 11.1, 1.56, 1.61, 1.68, 1.75, 1.79, 1.74}	// y = 0.02
};
const float r_coefficient_1st_meter_theta1_N[5][12] = {
    {3.5, 3.95, 2.55, 2.12, 1.92, 9.5, 1.7, 1.65, 1.62, 1.59, 1.57, 1.44}, // y =0
    {3.5, 4.00, 2.55, 2.12, 1.92, 13.2, 1.7, 1.65, 1.62, 1.59, 1.57, 1.43},// y =0.005
    {2.4, 2.07, 1.87, 1.75, 1.67, 14, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.01
    {2.4, 2.07, 1.87, 1.75, 1.67, 16, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.015
    {1.45, 1.43, 1.42, 1.425, 1.455, 20, 1.56, 1.61, 1.68, 1.75, 1.79, 1.74}	// y = 0.02
};
const float r_coefficient_1st_meter_theta0_N[5][12] = {
    {3.5, 3.95, 2.55, 2.12, 1.92, 0, 1.7, 1.65, 1.62, 1.59, 1.57, 1.44}, // y =0
    {3.5, 4.00, 2.55, 2.12, 1.92, 16, 1.7, 1.65, 1.62, 1.59, 1.57, 1.43},// y =0.005
    {2.4, 2.07, 1.87, 1.75, 1.67, 16, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.01
    {2.4, 2.07, 1.87, 1.75, 1.67, 10.9, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y = 0.015
    {1.45, 1.43, 1.42, 1.425, 1.455, 8.25, 1.56, 1.61, 1.68, 1.75, 1.79, 1.74}	// y = 0.02
};
const float r_coefficient_3[6][12] = {
    {3.5, 3.95, 2.55, 2.12, 1.92, 1.8, 1.7, 1.65, 1.62, 1.59, 1.57, 1.44}, // y/theta +/+
    {3.5, 4.00, 2.55, 2.12, 1.92, 1.79, 1.7, 1.65, 1.62, 1.59, 1.57, 1.43},// y/theta -/-
    {2.4, 2.07, 1.87, 1.75, 1.67, 1.63, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y/theta 0/+
    {2.4, 2.07, 1.87, 1.75, 1.67, 1.63, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y/theta 0/-
    {1.45, 1.43, 1.42, 1.425, 1.455, 1.47, 1.56, 1.61, 1.68, 1.75, 1.79, 1.74},	// y/theta -/+
    {1.45, 1.43, 1.42, 1.425, 1.455, 1.5, 1.55, 1.62, 1.69, 1.76, 1.8, 1.75}	// y/theta +/-
};
const float r_coefficient_2[6][12] = {
    {3.5, 3.95, 2.55, 2.12, 1.92, 1.8, 1.67, 1.65, 1.62, 1.55, 1.57, 1.44}, // y/theta +/+
    {3.5, 4.00, 2.55, 2.12, 1.92, 1.79, 1.67, 1.65, 1.62, 1.57, 1.57, 1.43},// y/theta -/-
    {2.4, 2.07, 1.87, 1.75, 1.67, 1.63, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y/theta 0/+
    {2.4, 2.07, 1.87, 1.75, 1.67, 1.63, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y/theta 0/-
    {1.45, 1.43, 1.42, 1.425, 1.455, 1.47, 1.56, 1.61, 1.68, 1.95, 1.79, 1.74},	// y/theta -/+
    {1.45, 1.43, 1.42, 1.425, 1.455, 1.5, 1.55, 1.62, 1.69, 1.88, 1.8, 1.75}	// y/theta +/-
};
const float r_coefficient_1[6][12] = {
    {3.5, 3.95, 2.55, 2.12, 1.92, 1.8, 1.68, 1.65, 1.62, 1.47, 1.57, 1.44}, // y/theta +/+
    {3.5, 4.00, 2.55, 2.12, 1.92, 1.79, 1.68, 1.65, 1.62, 1.47, 1.57, 1.43},// y/theta -/-
    {2.4, 2.07, 1.87, 1.75, 1.67, 1.63, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y/theta 0/+
    {2.4, 2.07, 1.87, 1.75, 1.67, 1.63, 1.59, 1.605, 1.605, 1.615, 1.615, 1.615},// y/theta 0/-
    {1.45, 1.43, 1.42, 1.425, 1.455, 1.47, 1.56, 1.61, 1.68, 0.0, 1.79, 1.74},	// y/theta -/+
    {1.45, 1.43, 1.42, 1.425, 1.455, 1.5, 1.55, 1.62, 1.69, 0.0, 1.8, 1.75}	// y/theta +/-
};

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
uint8_t open_loop=FALSE	;		// false
int32_t MAX_POSSIBLE_SIZE = 400; 
int32_t SAMPLES_PER_SECOND = 50; //(1s/DELTA_TIME) 
float ACCERATE_CONSTRAINT_1_METER = 1.4;
float ACCERATE_CONSTRAINT_MIDDLE = 0.10; 
float ACCERATE_CONSTRAINT_DEC = 0.3; 
float kp = -1.0; 
float ki = 0.0;
float kd = 0.0;
float last_travelling_time;
float err_sum; 
float real_curve_dist; 
float ut[400];//ut list for the bezier curve
float sampling_time = 0.020; 
int need_to_stop = 0;

struct{
	float2int wheel_circumference;
	float2int track_width;
	float2int upper_linear_v;
	float2int slow_linear_v;
	uint8_t gear_ratio;
	int32int8 cpr;
	uint8_t refresh_rate;
	uint8_t deck_gear_ratio;
	int32int8 deck_cpr;
}motorpara;

struct velo {
	float2int linear_v;
	float2int angular_v;
};

struct pos {
	float2int x;
	float2int y;
	float2int theta;
	float2int c_dis;
	float2int f_dis;
	struct velo velocity;
//	int32int8 l_enc_cnt;
//	int32int8 r_enc_cnt;
};

struct correction_data {
	float x_delta; 
	float y_delta; 
};

struct design_params {
	double v_s;
	double v_f;
	double accleration_constraint; //Maximum accleration 
	double jerk_constraint; //Jerk 
	double Sn;// 1m distance of the 2 QR codes 
	double v_max; //max permitable speed 
	int section_type;
};

struct convolution_params {
	double t0;
	double t1;
	double t2; //Total time = t0+t1+t2 
	double v0;
};

struct points {
	double x;
	double y;
	double yaw;
};

struct pos pose;

struct velo velocity;
struct velo correction_velocity;

struct design_params agv_constraint; //Physical constraints

struct convolution_params convol_params; 

struct points starting_point; 
struct points current_pose;  
float MAX_LINEAR_SPEED = 1.0;
float MAX_ANGULAR_SPEED = 5.0; 
float init_heading;
float next_point[2];


struct{
	float2int angle;
	uint8_t deck_sync;
}rotation;

float2int deck_rotation_angle;

uint8_t lifter_updn;

float2int current_deck_pos;

uint8_t current_lifter_dir;

FLAG fUART_roboteq;
FLAG fUART_pc;
FLAG fUART_deck;
FLAG fUART_lifter;
FLAG fINPUT_status;
FLAG f_lifter_status;	// ?FM read status flags

int32_t hall_count_buffer;

int32_t lifter_set_speed;
int32_t lifter_hall_count;
uint8_t lifter_sensor;
uint8_t lifter_status;
uint8_t lifter_exec_state = LIFTERMOVE_IDLE;
uint8_t home_exec_state = HOME_IDLE;
uint8_t deck_exec_state = DECKROTATION_IDLE;
int32_t deck_set_speed;
int32_t deck_hall_count;	// ?C
int32_t peak_hall_count;
uint32_t deck_mid_hall;
int32_t distance_hall_count;
int8_t deck_p_m;

uint8_t body_exec_state = BODYROTATION_IDLE;
int32_t encoder_speed;	// ?S
int32_t body_encoder_speed;	// ?S
int32_t body_set_speed;
int32_t peak_enc_count;
int32_t body_enc_count;
int32_t pre_body_enc_count;
int32_t distance_enc_count;
uint32_t body_mid_enc;
int8_t body_p_m;

uint8_t mixed_mode;
uint8_t motion_exec_state = MOTION_IDLE;
uint8_t bk_motion_exec_state = MOTION_IDLE;
#if MOTOR_MIXED_MODE
float angular_velocity;
float linear_velocity;
float theta_radian;
int8_t theta_count;
int8_t top_theta_count;
float delta_x;
float delta_y;
#else
float delta_x[3];
float delta_y[3];
float x_spd;
float r_spd;
float r_offset;
float remaining_distance = 0.0;

uint8_t keep_straight = POSE_SPEED_CORRECTION;
float elapse_time = 0;
uint8_t time_count;
float time_interval = TIME_INTERVAL2;
float P0[2];
float P1[2];
float P2[2];
float P3[2];
int16_t left_rpm;
int16_t right_rpm;
int32_t delta_dis_enc_count;
int32_t motion_dec_enc;
int16_t motion_dec_speed_l;
int16_t motion_dec_speed_r;
int16_t motion_dec_speed;
int16_t motion_speedl_dec_step;
int16_t motion_speedr_dec_step;
float motion_dec_time;
#endif

char str_roboteq[MAX_MOTOR_CMD_SIZE]={0};
uint8_t Snd2Pc[CmdPacketlength+PcSyncCode_SIZE];

char * roboteq_array[MAX_ARRAY_SIZE];
char * deck_array[MAX_ARRAY_SIZE];

int16_t motor_amps_1;		// ?A
int16_t motor_amps_2;		// ?A
int32_t encoder_count_1;	// ?C
int32_t encoder_count_2;	// ?C
int32_t encoder_speed_1;	// ?S
int32_t encoder_speed_2;	// ?S
uint16_t fault_flag;		// ?FF
int16_t battery_amps_1;		// ?BA
int16_t battery_amps_2;		// ?BA
int16_t power_1;			// ?P
int16_t power_2;			// ?P
uint8_t current_cmdid;
uint8_t cmd_sts;
uint8_t action_sts;
#if ENABLE_PC_STATUS_REQ
uint8_t motor_deck_status_bytes;
#endif
uint8_t status_cmdid;

int32_t motor_read_buf;

int32_t hall_speed;
uint16_t deck_fault_flag;		// ?FF
uint16_t wheel_macc;
uint16_t wheel_mdec;
uint8_t status_onoff;
uint8_t para_query;
uint8_t wheel_read_error;

float ticks_meter;
float t_delta;
float then = 0;
int32_t enc_left;
int32_t enc_right;
float dx;
float dr;
float x_final;
float y_final;
float theta_final;
float delta_dis;
float static_delta_dis;
float qr_interval = QR_INTERVAL_1;
float travel_time; 

float previous_x;
float previous_y;
float linear_speed;


uint8_t theta_y_mode = 0;
uint8_t velcmd_timeout = 0;
uint8_t refresh_count = 0;
int32_t start_left;
int32_t end_left;
int32_t start_right;
int32_t end_right;
float ticks_offset = 0.96;
int32_t pose_start_enc_cnt;
int32_t pose_2final_enc_cnt;
int32_t pose_end_enc_cnt;
float delta_offset=1.4;
float step_offset=1.5;
float c_1st_dis;
uint8_t posecmd_snd[255] = {0};
uint8_t posecmd_seq;
uint8_t led_green;
// float time_interval_min;

uint32_t log_i;
uint32_t log_d_i;
#if LOG_WHEEL
int32_t matrix_rpm[300];
int32_t matrix_dis[300];
#endif
#if LOG_DECK
int32_t matrix_d_rpm[200];
//int32_t matrix_d_dis[300];
#endif
extern int32_t prev_lencoder;
extern int32_t prev_rencoder;
extern int32_t lmult;
extern int32_t rmult;
extern int32_t left;
extern int32_t right;
extern int16_t speed_l;
extern int16_t speed_r;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern char PcData[CmdPacketlength];
extern int32_t TIMEOUT;
extern char RxBuf[RxBuf_SIZE];
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


/* Private function prototypes -----------------------------------------------*/
void uartRbSend (UART_HandleTypeDef *huart, char *str);
uint8_t Pc_Init();
uint8_t isPcPresent(int32_t Timeout);
void uartPcSend (uint8_t *str, uint8_t size);
void debugLog (char *str);
void Split_Str(char *cur_array, const char * chr, char **new_array);
void Hex2SignInt(char *cmd_str);
void Prepare_Channel_Data(int16_t chdata);
void Velocity2Rpm(float linear_v, float angular_v);
float Rpm2Velocity(uint16_t rpm);
void IO_Control();
void NV_Pt_On(Pt_TypeDef Pt);
void NV_Pt_Off(Pt_TypeDef Pt);
void NV_Pt_Toggle(Pt_TypeDef Pt);
uint8_t uart_Ack2PC(uint8_t cmdtype, uint8_t ack);
uint8_t uart_Data2PC();
uint8_t uart_Para2PC();
uint8_t uart_InitReq2PC();
uint8_t uart_ucGetCheckSum(uint8_t *msg, uint8_t length);
uint8_t uart_Motor_Rpm();
uint8_t uart_Motor_Stop();
uint8_t uart_Motor_Slow();
uint8_t uart_Motor_Estop();
uint8_t uart_Motor_Release();
uint8_t uart_Deck_Estop();
uint8_t uart_Deck_Release();
uint8_t uart_Deck_Stop();
uint8_t uart_Lifter_Stop();
uint8_t Velocity_Process();
uint8_t Local_Path_Planning();
uint8_t Read_Para_Query();
uint8_t Motor_Rotation();
uint8_t Deck_Home();
uint8_t Deck_Rotation();
uint8_t Lifter_Updown();
void Pc_Cmd_Process();
uint8_t Roboteq_Init();
uint8_t Deck_Init();
void Send_Querydata2PC();
void Receive_PC_Command();
void Exec_Action();
void Home_Action();
void Deck_Action();
void Body_Action();
void Lifter_Action();
void Motion_Action();
int32_t Absolute(int32_t value);
float Absolute_f(float value);
uint8_t isCmdConfirmed (int32_t Timeout);
uint8_t isDevicePresent (int32_t Timeout);
uint8_t isQueryConfirmed (int32_t Timeout);
uint8_t isWheelReadConfirmed (int32_t Timeout);
uint8_t isDeckCmdConfirmed (int32_t Timeout);
uint8_t isDeckDevicePresent (int32_t Timeout);
uint8_t isDeckReadConfirmed (int32_t Timeout);
uint8_t isLifterCmdConfirmed (int32_t Timeout);
uint8_t isLifterDevicePresent (int32_t Timeout);
uint8_t isLifterReadConfirmed (int32_t Timeout);
void Angle2EncCount(float angle);
uint8_t Configure_Motor_Speedmode();
uint8_t Configure_Motor_SpeedPositionmode();
uint8_t Configure_Motor_Positionmode();
uint8_t Configure_Deck_Positionmode();
uint8_t Send_Deck_Speed();
uint8_t Read_Deck_Hall_Count();
uint8_t Read_Deck_Hall_Speed();
uint8_t Send_Wheel_Speed(int16_t left_rpm, int16_t right_rpm);
uint8_t Send_Wheel_Command();
#if !TIMER_ENCODER
uint8_t Read_Wheel_Enc_Count();
uint8_t Read_Wheel1_Enc_Count();
uint8_t Read_Wheel_Enc_Speed();
#endif
uint8_t Read_Wheel_Fault_Flag();
uint8_t Read_Lifter_Upper_Input();
uint8_t Read_Lifter_Lower_Input();
uint8_t Send_Stream_Query_Off();
uint8_t Configure_Motor_Separatemode();
#if MOTOR_MIXED_MODE
uint8_t Configure_Motor_Mixedmode();
void Velocity2Throttle_Steering(float linear_v,float angular_v);
#else
void Generate_Bezier_Points(float x, float y, float theta, float travel_distance);
float Calculate_Linear_Velocity(float diff10, float diff21, float diff32, float t);
void Calculate_Velocity_Rotation(float t);
void Pose_Init();
void Pose_2nd_Init();
void Pose_Update();
void Set_R_Coefficient();
void Set_R_Coefficient_1st_Meter();
#endif
void step_plus();
void step_minus();
void deck_step_plus();
void deck_step_minus();
float T_angle(float a);
void Current_Cmd_Update();
void Action_Cmd_Process();
void Status_Cmd_Process();
void Slowspeed_Update();
void Calculate_Deceleration_Info();
void Calculate_Delta_Distance();
void Calculate_Static_Delta_Distance();
uint8_t Pose_End_Process();
void Pose_Speed_Init();
void Reset_Encoder_Count();
extern void Ringbuf_Init (void);
void Global_Planning();
void Led_Green();
extern uint8_t waitFor (char *string, uint32_t Timeout);
int BEZIER_ORDER=3;
float controlPoints [4];
float ang_vel = 0.0, ang_vel2 = 0.0;
float r_linvel;
float point[2];
float last_point[2];
float dtheta,dtheta2, last_dtheta2;
#if DEBUG_MODE_ON 
float Angveldebug[300];
float Linveldebug[300];
float rhodebug[300];
float vdebug[300];
float wdebug[300];
float xpointdebug[300];
float ypointdebug[300];
float rpmrightdebug[300];
float rpmleftdebug[300];
float dthetadebug[300];
float dthetadebug2[300];
#endif
float Y0[400];
float Y1[400];
float linear_speed_profile[400];
float debugleft_rpm, debugright_rpm;
float hypot_distance; 
float Kp_rho = 9.0;
float Kp_alpha= 15.0;
float Kp_beta = 3.0; 


/************************************************************************************
FUNCTION		:Roboteq_Init
DESCRIPTION		:roboteq initializing sequence.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t Roboteq_Init()
{
	Ringbuf_Init();
	HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *)RxBuf, RxBuf_SIZE);
	__HAL_DMA_DISABLE_IT(&DMA_UART7, DMA_IT_HT);

	/********* \5 **********/
	uartRbSend(device_uart, "\5\r");
	if(isDevicePresent(MOTOR_TIMEOUT) != 1)
	{
		debugLog("failed at \5\r\n\n");
		return 0;
	}
	debugLog("Device Present\n\n");
//	HAL_Delay(1000);
	isSpeedmode = 1;
	/********* Stream query **********/
	uartRbSend(device_uart, "^echof 1_# c\r");
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	HAL_Delay (1000);	// Delay is needed because Stream Query will take long time.
	if (uart_Motor_Stop() == 0){
		debugLog("Motor Stop fails\n\n");
		return 0;
	}
	if (Configure_Motor_Separatemode() == 0){
		return 0;
	}
	wheel_macc = 2000;
	uartRbSend(device_uart, "~MAC 1\r");
	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	wheel_macc = (motor_read_buf)/10;
	wheel_mdec = 2000;
	uartRbSend(device_uart, "~MDEC 1\r");
	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	wheel_mdec = (motor_read_buf)/10;
#if ROS_DEBUG
	status_onoff = 1;	// Enable communication status: test for PC ROS
#endif
	return 1;
}
/************************************************************************************
FUNCTION		:Deck_Init
DESCRIPTION		:Deck initializing sequence.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/6/15
*************************************************************************************/
uint8_t Deck_Init()
{
	Ringbuf_Init();
	HAL_UARTEx_ReceiveToIdle_DMA(&huart8, (uint8_t *)RxBuf, RxBuf_SIZE);
	__HAL_DMA_DISABLE_IT(&DMA_UART8, DMA_IT_HT);

	/********* \5 **********/
	uartRbSend(deck_uart, "\5\r");
	if(isDeckDevicePresent(MOTOR_TIMEOUT) != 1)
	{
		debugLog("failed at \5\r\n\n");
		return 0;
	}
	debugLog("Deck Present\n\n");
	uartRbSend(deck_uart, "^echof 1\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	HAL_Delay(1000);
	if ( uart_Deck_Stop() == 0){
		debugLog("Deck Stop fails\n\n");
		return 0;
	}
	current_lifter_dir = LIFTER_DN;
#if ENABLE_LIFTER_INIT
	if(Read_Lifter_Upper_Input() != 1){
		return 0;
	}
	if(lifter_sensor == 1){
		current_lifter_dir = LIFTER_UP;
	}
	if(Read_Lifter_Lower_Input() != 1){
		return 0;
	}
	if(lifter_sensor == 1){
		current_lifter_dir = LIFTER_DN;
	}
#endif
	return 1;
}
/************************************************************************************
FUNCTION		:Receive_PC_Command
DESCRIPTION		:Receive commands from PC and execute proper action to Roboteq controller.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/8/3
*************************************************************************************/
void Receive_PC_Command()
{
	if ((fault_flag&0x007F) != 0){	// Roboteq error flag checking
		cmd_sts = PC_CMDSTS_ERROR;
		if (encoder_speed_1 != 0 || encoder_speed_2 != 0){	// Still moving?
			uart_Motor_Stop();
		}
		if (isPCDataOK == TRUE){
			isPCDataOK = FALSE;
		}
	}else{
		if (current_cmdid == PC_CMDID_VELOCITY){
			if (velcmd_timeout != 0){
				velcmd_timeout--;
				if (velcmd_timeout == 0){
					velocity.linear_v.f = 0;
					velocity.angular_v.f = 0;
					if (uart_Motor_Rpm() == 0){
						cmd_sts = PC_CMDSTS_ERROR;
					}else{
						cmd_sts = PC_CMDSTS_ABORTED;
					}
				}
			}
		}
		// Receive command from PC and judge header is OK, after return ACK to PC, start command execution to Roboteq
		if (isPCDataOK == TRUE){
			isPCDataOK = FALSE;
			Pc_Cmd_Process();	// Process all commands from PC: control motor and deck.
		}
		if (isPCActionOK == TRUE){
			isPCActionOK = FALSE;
			Action_Cmd_Process();
		}
		if (isPCStatusOK == TRUE){
			isPCStatusOK = FALSE;
			Status_Cmd_Process();
		}
	}
}
/************************************************************************************
FUNCTION		:Send_Querydata2PC
DESCRIPTION		:Send received query data to PC.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/8/3
*************************************************************************************/
void Send_Querydata2PC()
{
	// Receive query info from Roboteq controller, send the data/error to PC and wait for PC to return ACK.
//	if (isDeckOption == TRUE){
//		if (isQueryOK== TRUE || isDeckQueryOK== TRUE){
//			isQueryOK = FALSE;
//			isDeckQueryOK = FALSE;
//			uart_Data2PC();
//			debugLog("Sending both query data to PC succeeds\n");
//		}
//	}else{
//		if (isQueryOK== TRUE){
//			isQueryOK = FALSE;
//			uart_Data2PC();
//			debugLog("Sending motor query data to PC succeeds\n");
//		}
//	}
#if !TIMER_ENCODER
	Read_Wheel_Enc_Count();
	Read_Wheel_Enc_Speed();
#endif
	Read_Wheel_Fault_Flag();
#if ENABLE_PC_STATUS_REQ
	if(motor_deck_status_bytes == MOTOR_QUERYDATA_LENGTH || motor_deck_status_bytes == (MOTOR_QUERYDATA_LENGTH + MOTOR_QUERYPARA_LENGTH)){
#endif
		refresh_count++;
		if(refresh_count >= motorpara.refresh_rate){
			refresh_count = 0;
			uart_Data2PC();
		}
#if ENABLE_PC_STATUS_REQ
		motor_deck_status_bytes = 0;
	}
#endif
	return;
}
/************************************************************************************
FUNCTION		:Exec_Action
DESCRIPTION		:Execute Home, Deck rotation, Wheel rotation and Lifter up/down actions.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/7/25
*************************************************************************************/
void Exec_Action(){
	Home_Action();		// Deck Home operation
	Deck_Action();		// Deck rotation
	Body_Action();		// Wheel rotation
	Lifter_Action();	// Lifter up down operation
	Motion_Action();	// Motion control while traveling
	return;
}

/************************************************************************************
FUNCTION		:uartRbSend
DESCRIPTION		:Send UART command to roboteq controller with 1000ms timeout
INPUT			:command string to be sent to roboteq controller
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void uartRbSend (UART_HandleTypeDef *huart, char *str)
{
	HAL_UART_Transmit(huart, (uint8_t *) str, strlen (str), MOTOR_TIMEOUT);
}

/************************************************************************************
FUNCTION		:debugLog
DESCRIPTION		:Send UART command to PC with 300ms timeout for debugging purpose
INPUT			:string to be sent to PC
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void debugLog (char *str)
{
	printf((char *) str);
}

/************************************************************************************
FUNCTION		:isDevicePresent
DESCRIPTION		:To check whether roboteq controller returns \6 within Timeout.
INPUT			:Timeout: in milliseconds, to decrease in SysTick_Handler.
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t isDevicePresent(int32_t Timeout)
{
	TIMEOUT = Timeout;
	while ((!isPresent)&&(TIMEOUT));
	isPresent = FALSE;
	if (TIMEOUT <= 0) return 0;
	return 1;
}

/************************************************************************************
FUNCTION		:isCmdConfirmed
DESCRIPTION		:To check whether roboteq controller returns +\r within Timeout.
INPUT			:Timeout: to decrease in SysTick_Handler.
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t isCmdConfirmed (int32_t Timeout)
{
	TIMEOUT = Timeout;
	while ((!isSetCmdOK)&&(TIMEOUT));
	isSetCmdOK = FALSE;
	if (TIMEOUT <= 0) return 0;
	return 1;
}

/************************************************************************************
FUNCTION		:isQueryConfirmed
DESCRIPTION		:To check whether roboteq controller returns DH? within Timeout.
INPUT			:Timeout: to decrease in SysTick_Handler.
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t isQueryConfirmed (int32_t Timeout)
{
	TIMEOUT = Timeout;
	while ((!isQueryOK)&&(TIMEOUT));
	isQueryOK = FALSE;
	if (TIMEOUT <= 0) return 0;
	return 1;
}
/************************************************************************************
FUNCTION		:isWheelReadConfirmed
DESCRIPTION		:To check whether roboteq controller returns value within Timeout.
INPUT			:Timeout: to decrease in SysTick_Handler.
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/7/7
*************************************************************************************/
uint8_t isWheelReadConfirmed (int32_t Timeout)
{
	TIMEOUT = Timeout;
	while ((!isWheelReadOK)&&(TIMEOUT));
	isWheelReadOK = FALSE;
	if (TIMEOUT <= 0) return 0;
	return 1;
}
/************************************************************************************
FUNCTION		:isDeckDevicePresent
DESCRIPTION		:To check whether roboteq controller returns \6 within Timeout.
INPUT			:Timeout: in milliseconds, to decrease in SysTick_Handler.
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/6/15
*************************************************************************************/
uint8_t isDeckDevicePresent(int32_t Timeout)
{
	TIMEOUT = Timeout;
	while ((!isDeckPresent)&&(TIMEOUT));
	isDeckPresent = FALSE;
	if (TIMEOUT <= 0) return 0;
	return 1;
}

/************************************************************************************
FUNCTION		:isDeckCmdConfirmed
DESCRIPTION		:To check whether roboteq controller returns +\r within Timeout.
INPUT			:Timeout: to decrease in SysTick_Handler.
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/6/15
*************************************************************************************/
uint8_t isDeckCmdConfirmed (int32_t Timeout)
{
	TIMEOUT = Timeout;
	while ((!isDeckSetCmdOK)&&(TIMEOUT));
	isDeckSetCmdOK = FALSE;
	if (TIMEOUT <= 0) return 0;
	return 1;
}
/************************************************************************************
FUNCTION		:isDeckReadConfirmed
DESCRIPTION		:To check whether roboteq controller returns value within Timeout.
INPUT			:Timeout: to decrease in SysTick_Handler.
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/7/7
*************************************************************************************/
uint8_t isDeckReadConfirmed (int32_t Timeout)
{
	TIMEOUT = Timeout;
	while ((!isDeckReadOK)&&(TIMEOUT));
	isDeckReadOK = FALSE;
	if (TIMEOUT <= 0) return 0;
	return 1;
}
/************************************************************************************
FUNCTION		:isLifterReadConfirmed
DESCRIPTION		:To check whether roboteq controller returns value within Timeout.
INPUT			:Timeout: to decrease in SysTick_Handler.
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/8/30
*************************************************************************************/
uint8_t isLifterReadConfirmed (int32_t Timeout)
{
	TIMEOUT = Timeout;
	while ((!isLifterReadOK)&&(TIMEOUT));
	isLifterReadOK = FALSE;
	if (TIMEOUT <= 0) return 0;
	return 1;
}

/************************************************************************************
FUNCTION		:Hex2SignInt
DESCRIPTION		:Convert HEX code to signed integer.
INPUT			:cmd_str
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void Hex2SignInt(char *cmd_str){
	char snum[5]={0};
	int value;
	if (PcData[DataIdx] == 0){
		strcpy(str_roboteq, "!");
		strcat (str_roboteq, cmd_str);
		strcat (str_roboteq, " ");

		strcat (str_roboteq, "1 ");
		value =(int16_t) ((PcData[DataIdx+1]&0xFF)<<8) | (PcData[DataIdx+2]&0xFF);
		Prepare_Channel_Data(value);

		strcat (str_roboteq, "_!");
		strcat (str_roboteq, cmd_str);
		strcat (str_roboteq, " ");

		strcat (str_roboteq, "2 ");
		value =(int16_t) ((PcData[DataIdx+1]&0xFF)<<8) | (PcData[DataIdx+2]&0xFF);
		Prepare_Channel_Data(value);
		strcat (str_roboteq, "\r");
	}else{
		strcpy(str_roboteq, "!");
		strcat (str_roboteq, cmd_str);
		strcat (str_roboteq, " ");
		itoa(PcData[DataIdx], snum, 10);
		strcat (str_roboteq, snum);
		strcat (str_roboteq, " ");
		value =(int16_t) ((PcData[DataIdx+1]&0xFF)<<8) | (PcData[DataIdx+2]&0xFF);
		Prepare_Channel_Data(value);
		strcat (str_roboteq, "\r");
	}

}
/************************************************************************************
FUNCTION		:Prepare_Channel_Data
DESCRIPTION		:Convert HEX code to signed integer.
INPUT			:chdata
OUTPUT			:None
UPDATE			:2022/5/18
*************************************************************************************/
void Prepare_Channel_Data(int16_t chdata){
	char snum[10]={0};

	if(chdata >= 0){
		strcat (str_roboteq, "+");
	}else if (chdata < 0){
		strcat (str_roboteq, "-");
		chdata =0x00-chdata;
	}
	itoa(chdata, snum, 10);
	strcat (str_roboteq, snum);
}
/************************************************************************************
FUNCTION		:Velocity2Rpm
DESCRIPTION		:Convert Velocity to RPM
INPUT			:linear, angular from PC
OUTPUT			:None
UPDATE			:2022/5/18
*************************************************************************************/
void Velocity2Rpm(float linear_v,float angular_v){
	float right_speed = linear_v + motorpara.track_width.f * angular_v / 2.0;
	float left_speed = linear_v - motorpara.track_width.f * angular_v / 2.0;
	int16_t left_rpm = left_speed / motorpara.wheel_circumference.f * M2SECS * motorpara.gear_ratio;
	int16_t right_rpm = right_speed / motorpara.wheel_circumference.f * M2SECS * motorpara.gear_ratio;

	debugleft_rpm = left_rpm;
	debugright_rpm = right_rpm;

	strcpy(str_roboteq, "!S 1 ");
	Prepare_Channel_Data(left_rpm);
	strcat (str_roboteq, "_!S 2 ");
	Prepare_Channel_Data(right_rpm);
	strcat (str_roboteq, "\r");
}

/************************************************************************************
FUNCTION		:Rpm2Velocity
DESCRIPTION		:Convert RPM to Velocity.
INPUT			:RPM
OUTPUT			:speed
UPDATE			:2022/9/16
*************************************************************************************/
float Rpm2Velocity(uint16_t rpm){
	return (((float)rpm) * motorpara.wheel_circumference.f / (M2SECS * (float)motorpara.gear_ratio));
}
/************************************************************************************
FUNCTION		:Split_Str
DESCRIPTION		:Query data from roboteq controller are separated by delimiter "?:". Need to extract the data to individual RAMs.
INPUT			:cur_array: current arrays with delimiter; chr:delimiter string; new_array: extract data to new arrays.
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void Split_Str(char *cur_array, const char * chr, char **new_array){
	int i =0;
	char* p = strtok (cur_array, chr);
	while (p != NULL && i<MAX_ARRAY_SIZE)	// Check MAX_ARRAY_SIZE to prevent RAM corruption.
	{
		new_array[i++] = p;
		p = strtok (NULL, chr);
	}
}

/************************************************************************************
FUNCTION		:Pc_Init
DESCRIPTION		:Resets the Ring buffer.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t Pc_Init(){
	Ringbuf_Init();

	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, (uint8_t *)RxBuf, RxBuf_SIZE);
	__HAL_DMA_DISABLE_IT(&DMA_UART4, DMA_IT_HT);

	/********* Init parameters request **********/
#if MCU_INITREQ_ENABLE
	uart_InitReq2PC();
	if(isPcPresent(10000) != 1)		// Temporary 1s timeout for sending COM command
#endif
	{
		motorpara.wheel_circumference.f = WHEEL_CIRCUMFERENCE;
		motorpara.track_width.f = TRACK_WIDTH;
		motorpara.upper_linear_v.f = UPPER_LINEAR_V;
		motorpara.slow_linear_v.f = SLOW_LINEAR_V;
		motorpara.gear_ratio = WHEEL_GEAR_RATIO;
		motorpara.cpr.i32 = WHEEL_ENC_CPR;
		motorpara.refresh_rate = REFRESH_RATE;
		motorpara.deck_gear_ratio = DECK_GEAR_RATIO;
		motorpara.deck_cpr.i32 = DECK_ENC_CPR;
		ticks_meter = (1 / motorpara.wheel_circumference.f) * motorpara.cpr.i32 * motorpara.gear_ratio;
		current_cmdid = PC_CMDID_NONE;
		cmd_sts = PC_CMDSTS_IDLE;
		debugLog("PC sending init parameters fails\n\n");
#if MCU_INITREQ_ENABLE
		return 0;
#else
		return 1;
#endif
	}
#if MCU_INITREQ_ENABLE
	memcpy(&motorpara.wheel_circumference.bytes[0],&PcData[DataIdx],sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio));
	memcpy(&motorpara.cpr.bytes[0],&PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)],sizeof(motorpara.cpr.i32));
	memcpy(&motorpara.deck_gear_ratio,&PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)+sizeof(motorpara.cpr.i32)],sizeof(motorpara.deck_gear_ratio));
	memcpy(&motorpara.deck_cpr.bytes[0],&PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)+sizeof(motorpara.cpr.i32)+sizeof(motorpara.deck_gear_ratio)],sizeof(motorpara.deck_cpr.i32));
	debugLog("PC sending init parameters succeeds\n\n");
	return 1;
#endif
}

/************************************************************************************
FUNCTION		:uart_Ack2PC
DESCRIPTION		:MCU send ACK command to PC.
INPUT			:ack: 0:OK; 1:NG
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t uart_Ack2PC(uint8_t cmdtype, uint8_t ack){
	//After confirm checksum is OK, send ACK packet.
	uint8_t n=0;

	Snd2Pc[n++]='G';
	Snd2Pc[n++]='R';
	Snd2Pc[n++]=cmdtype;
	Snd2Pc[n++]=MOTOR_ACK_LENGTH;
	Snd2Pc[n++]=PcData[CmdIDIdx];
	Snd2Pc[n++]=ack;
	Snd2Pc[n++]=uart_ucGetCheckSum(&Snd2Pc[DataIdx],MOTOR_ACK_LENGTH+PcDataLength_SIZE+PcCmdID_SIZE);
	Snd2Pc[n++]='\r';
	uartPcSend(Snd2Pc,n);
	return 1;
}
/************************************************************************************
FUNCTION		:uart_Data2PC
DESCRIPTION		:MCU send query data to PC.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t uart_Data2PC( ){
	//After receive QUERY data from Roboteq, send data to PC per 50ms.
	uint8_t n=0;

	Snd2Pc[n++]='G';
	Snd2Pc[n++]='R';
	Snd2Pc[n++]=TYPE_STATUS;
	Snd2Pc[n++]=((isDeckOption == FALSE)?MOTOR_QUERYDATA_LENGTH:(MOTOR_QUERYDATA_LENGTH+DECK_QUERYDATA_LENGTH));
	Snd2Pc[n++]=MCU_CMDID_STATUS;
#if TIMER_ENCODER
		Snd2Pc[n++]=(uint8_t)((left>>24)&0xFF);
		Snd2Pc[n++]=(uint8_t)((left>>16)&0xFF);
		Snd2Pc[n++]=(uint8_t)((left>>8)&0xFF);
		Snd2Pc[n++]=(uint8_t)(left&0xFF);
		Snd2Pc[n++]=(uint8_t)((right>>24)&0xFF);
		Snd2Pc[n++]=(uint8_t)((right>>16)&0xFF);
		Snd2Pc[n++]=(uint8_t)((right>>8)&0xFF);
		Snd2Pc[n++]=(uint8_t)(right&0xFF);
		Snd2Pc[n++]=(uint8_t)((speed_l>>24)&0xFF);
		Snd2Pc[n++]=(uint8_t)((speed_l>>16)&0xFF);
		Snd2Pc[n++]=(uint8_t)((speed_l>>8)&0xFF);
		Snd2Pc[n++]=(uint8_t)(speed_l&0xFF);
		Snd2Pc[n++]=(uint8_t)((speed_r>>24)&0xFF);
		Snd2Pc[n++]=(uint8_t)((speed_r>>16)&0xFF);
		Snd2Pc[n++]=(uint8_t)((speed_r>>8)&0xFF);
		Snd2Pc[n++]=(uint8_t)(speed_r&0xFF);
#else
	Snd2Pc[n++]=(uint8_t)(encoder_count_1&0xFF);
	Snd2Pc[n++]=(uint8_t)((encoder_count_1>>8)&0xFF);
	Snd2Pc[n++]=(uint8_t)((encoder_count_1>>16)&0xFF);
	Snd2Pc[n++]=(uint8_t)((encoder_count_1>>24)&0xFF);
	Snd2Pc[n++]=(uint8_t)(encoder_count_2&0xFF);
	Snd2Pc[n++]=(uint8_t)((encoder_count_2>>8)&0xFF);
	Snd2Pc[n++]=(uint8_t)((encoder_count_2>>16)&0xFF);
	Snd2Pc[n++]=(uint8_t)((encoder_count_2>>24)&0xFF);
	Snd2Pc[n++]=(uint8_t)(encoder_speed_1&0xFF);
	Snd2Pc[n++]=(uint8_t)((encoder_speed_1>>8)&0xFF);
	Snd2Pc[n++]=(uint8_t)((encoder_speed_1>>16)&0xFF);
	Snd2Pc[n++]=(uint8_t)((encoder_speed_1>>24)&0xFF);
	Snd2Pc[n++]=(uint8_t)(encoder_speed_2&0xFF);
	Snd2Pc[n++]=(uint8_t)((encoder_speed_2>>8)&0xFF);
	Snd2Pc[n++]=(uint8_t)((encoder_speed_2>>16)&0xFF);
	Snd2Pc[n++]=(uint8_t)((encoder_speed_2>>24)&0xFF);
#endif
	Snd2Pc[n++]=(uint8_t)(fault_flag&0xFF);
	Snd2Pc[n++]=(uint8_t)(isStopOn | isEstopOn | isSlowOn);
	Snd2Pc[n++]=(uint8_t)(current_cmdid&0xFF);
	Snd2Pc[n++]=(uint8_t)(cmd_sts);
#if ENABLE_PC_STATUS_REQ
	if(motor_deck_status_bytes == (MOTOR_QUERYDATA_LENGTH + MOTOR_QUERYPARA_LENGTH)){
#else
	if (isDeckOption == TRUE){
#endif
		Snd2Pc[n++]=(uint8_t)(current_deck_pos.bytes[0]);
		Snd2Pc[n++]=(uint8_t)(current_deck_pos.bytes[1]);
		Snd2Pc[n++]=(uint8_t)(current_deck_pos.bytes[2]);
		Snd2Pc[n++]=(uint8_t)(current_deck_pos.bytes[3]);
		Snd2Pc[n++]=(uint8_t)(current_lifter_dir);
		Snd2Pc[n++]=uart_ucGetCheckSum(&Snd2Pc[DataIdx],MOTOR_QUERYDATA_LENGTH+DECK_QUERYDATA_LENGTH+PcDataLength_SIZE+PcCmdID_SIZE);
	}else{
		Snd2Pc[n++]=uart_ucGetCheckSum(&Snd2Pc[DataIdx],MOTOR_QUERYDATA_LENGTH+PcDataLength_SIZE+PcCmdID_SIZE);
	}
	Snd2Pc[n++]='\r';
	uartPcSend(Snd2Pc,n);
	return 1;
//	for (int i = 0; i<PC_RETRY_TIMES; i++){
//		if(isPcCmdConfirmed(PC_TIMEOUT) == 1){
//			return 1;
//		}
//	}
//	return 0;
}
#if MCU_INITREQ_ENABLE
/************************************************************************************
FUNCTION		:uart_InitReq2PC
DESCRIPTION		:MCU requests init data to PC.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/20
*************************************************************************************/
uint8_t uart_InitReq2PC( ){
	//After receive QUERY data from Roboteq, send data to PC per 50ms.
	uint8_t n=0;

	Snd2Pc[n++]='G';
	Snd2Pc[n++]='R';
	Snd2Pc[n++]=TYPE_STATUS;
	Snd2Pc[n++]=MOTOR_INITREQ_LENGTH;
	Snd2Pc[n++]=MCU_CMDID_INITREQ;
	Snd2Pc[n++]=sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)+sizeof(motorpara.cpr.i32)+sizeof(motorpara.deck_gear_ratio)+sizeof(motorpara.deck_cpr.i32);	//4+4+4+1+4+1+4 bytes init data requested
	Snd2Pc[n++]=uart_ucGetCheckSum(&Snd2Pc[DataIdx],MOTOR_INITREQ_LENGTH+PcDataLength_SIZE+PcCmdID_SIZE);
	Snd2Pc[n++]='\r';
	uartPcSend(Snd2Pc,n);
	return 1;
//	for (int i = 0; i<PC_RETRY_TIMES; i++){
//		if(isPcCmdConfirmed(PC_TIMEOUT) == 1){
//			return 1;
//		}
//	}
//	return 0;
}
#endif
/************************************************************************************
FUNCTION		:uartPcSend
DESCRIPTION		:Send UART command to PC with 300ms timeout. Size is needed because command is in HEX format.
INPUT			:command and size to be sent to PC
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void uartPcSend (uint8_t *str, uint8_t size)
{
	HAL_UART_Transmit(pc_uart, (uint8_t *) str, size, PC_TIMEOUT);
}

/************************************************************************************
FUNCTION		:isPcPresent
DESCRIPTION		:To check whether PC returns initial parameters within Timeout.
INPUT			:Timeout: in milliseconds, to decrease in SysTick_Handler.
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/20
*************************************************************************************/
uint8_t isPcPresent(int32_t Timeout)
{
	TIMEOUT = Timeout;
	while ((!isPCPresent)&&(TIMEOUT));
	isPCPresent = FALSE;
	if (TIMEOUT <= 0) return 0;
	return 1;
}
/************************************************************************************
FUNCTION		:isPcCmdConfirmed
DESCRIPTION		:To check whether PC returns ACK packet within Timeout.
INPUT			:Timeout: to decrease in SysTick_Handler.
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t isPcCmdConfirmed (int32_t Timeout)
{
	TIMEOUT = Timeout;
	while ((!isPCAckOK)&&(TIMEOUT));
	isPCAckOK = FALSE;
	if (TIMEOUT <= 0) return 0;
	return 1;
}

/************************************************************************************
FUNCTION		:uart_ucGetCheckSum
DESCRIPTION		:8bits 1's complement check sum.
INPUT			:msg: array to calculate check sum; length: data length.
OUTPUT			:1's complement value
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t uart_ucGetCheckSum(uint8_t *msg, uint8_t length){
	uint8_t	sum,i;
	sum = 0;
	for	(i=0 ; i<length; i++) {
		sum += *msg;
		msg++;
	}
	return (0x01 - sum);
}

/************************************************************************************
FUNCTION		:IO_Control
DESCRIPTION		:Input/ Output control for STOP, SLOW and ESTOP, etc
INPUT			:void
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void IO_Control(){
	// ESTOP input process
    if(HAL_GPIO_ReadPin(ESTOP_GPIO_Port,ESTOP_Pin) == GPIO_PIN_SET){
    	if (isEstopOn == FALSE){
    		uart_Motor_Estop();
    		uart_Deck_Estop();
    		home_exec_state = HOME_IDLE;
    		deck_exec_state = DECKROTATION_IDLE;
    		body_exec_state = BODYROTATION_IDLE;
			lifter_exec_state = LIFTERMOVE_IDLE;
			motion_exec_state = MOTION_IDLE;
        	isEstopOn  = TRUE;
        	NV_Pt_On(LED_RED);
       	}
    }else{
    	if (isEstopOn == TRUE){
    		velocity.linear_v.f = 0;
    		velocity.angular_v.f = 0;
    		pose.velocity.linear_v.f = 0;
    		pose.velocity.angular_v.f = 0;
			current_deck_pos.f = 0.0;
			current_lifter_dir = LIFTER_DN;
    		uart_Motor_Release();
    		uart_Deck_Release();
        	isEstopOn  = FALSE;
        	NV_Pt_Off(LED_RED);
    	}
    }
	// STOP input process
    if(body_exec_state != BODYROTATION_IDLE){
    	return;
    }
	if(isEstopOn == FALSE){
		if(HAL_GPIO_ReadPin(STOP_GPIO_Port,STOP_Pin) == GPIO_PIN_SET){
			if (isStopOn == FALSE){
				uart_Motor_Stop();
				if (current_cmdid == PC_CMDID_POSE && motion_exec_state !=0){
					bk_motion_exec_state = motion_exec_state;
					motion_exec_state = MOTION_PAUSE;
				}
				isStopOn  = TRUE;
	        	NV_Pt_On(LED_ORG);
			}
		}else{
			if (isStopOn == TRUE){
				if (current_cmdid == PC_CMDID_POSE && motion_exec_state == MOTION_PAUSE_END){
					motion_exec_state = MOTION_RESUME;
				}else if (current_cmdid == PC_CMDID_VELOCITY && velcmd_timeout !=0){
					uart_Motor_Rpm(); // Recover to normal velocity
				}
				isStopOn  = FALSE;
	        	NV_Pt_Off(LED_ORG);
			}
		}
		// SLOW input process
		if(isStopOn == FALSE){
			if(HAL_GPIO_ReadPin(SLOW_GPIO_Port,SLOW_Pin) == GPIO_PIN_SET && (velocity.linear_v.f > motorpara.slow_linear_v.f || pose.velocity.linear_v.f > motorpara.slow_linear_v.f)){
				if (isSlowOn == FALSE){
					uart_Motor_Slow();		// Fix to slow velocity
					isSlowOn  = TRUE;
//		        	NV_Pt_On(LED_GRN);
				}
			}else{
				if (isSlowOn == TRUE){
					uart_Motor_Rpm();		// Recover to normal velocity
					isSlowOn  = FALSE;
//		        	NV_Pt_Off(LED_GRN);
				}
			}
		}
	}
}
/************************************************************************************
FUNCTION		:Led_Green
DESCRIPTION		:To indicate MCU is not hanging
INPUT			:void
OUTPUT			:None
UPDATE			:2022/11/17
*************************************************************************************/
void Led_Green(){
	led_green ^= 1;
	if(led_green){
		NV_Pt_On(LED_GRN);
	}else{
		NV_Pt_Off(LED_GRN);
	}
}
/************************************************************************************
FUNCTION		:uart_Motor_Stop
DESCRIPTION		:Send STOP command to roboteq controller. Change speed to 0.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t uart_Motor_Stop(){
	/********* STOP motor 1/2 **********/
	uartRbSend(device_uart, "!S 1 0\r");
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		debugLog("Motor 1 stop fails\n\n");
		return 0;
	}
	uartRbSend(device_uart, "!S 2 0\r");
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		debugLog("Motor 2 stop fails\n\n");
		return 0;
	}
	return 1;
}

/************************************************************************************
FUNCTION		:uart_Deck_Stop
DESCRIPTION		:Send STOP command to Deck roboteq controller. Change speed to 0.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/6/15
*************************************************************************************/
uint8_t uart_Deck_Stop(){
	/********* STOP motor 1/2 **********/
	uartRbSend(deck_uart, "!S 2 0\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		debugLog("Deck Lifter/Rotator stop fails\n\n");
		return 0;
	}
	return 1;
}

/************************************************************************************
FUNCTION		:uart_Lifter_Stop
DESCRIPTION		:Send STOP command to Lifter roboteq controller. Change speed to 0.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/7/15
*************************************************************************************/
uint8_t uart_Lifter_Stop(){
	/********* STOP motor 1/2 **********/
	uartRbSend(deck_uart, "!G 1 0\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		debugLog("Deck Lifter fails\n\n");
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:uart_Motor_Slow
DESCRIPTION		:Process speed according to formula.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t uart_Motor_Slow(){
	/********* SLOW motor 1/2 **********/
	if ((current_cmdid == PC_CMDID_VELOCITY && velcmd_timeout !=0) || (current_cmdid == PC_CMDID_POSE && motion_exec_state !=0)){
		Velocity2Rpm(motorpara.slow_linear_v.f, 0);
		if (Send_Wheel_Command() != 1){
			cmd_sts = PC_CMDSTS_ERROR;
			printf("Wheel command sending error \n");
			return 0;
		}
		cmd_sts = PC_CMDSTS_COMPLETED;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:uart_Motor_Rpm
DESCRIPTION		:Send RPM to roboteq controller.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t uart_Motor_Rpm(){
	if (current_cmdid == PC_CMDID_VELOCITY){
		Velocity2Rpm(velocity.linear_v.f, velocity.angular_v.f);
	}else if (current_cmdid == PC_CMDID_POSE && motion_exec_state !=0){
		Velocity2Rpm(pose.velocity.linear_v.f, pose.velocity.angular_v.f);
	}else{
		return 1;
	}
	if (Send_Wheel_Command() != 1){
		cmd_sts = PC_CMDSTS_ERROR;
		printf("Wheel command sending error \n");
		return 0;
	}
	cmd_sts = PC_CMDSTS_COMPLETED;
	return 1;
}
/************************************************************************************
FUNCTION		:uart_Motor_Estop
DESCRIPTION		:Send ESTOP command to roboteq controller.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t uart_Motor_Estop(){
	/********* STOP motor 1/2 and enter ESTOP mode**********/
	if (uart_Motor_Stop() == 0){
		return 0;
	}
	uartRbSend(device_uart, "!EX\r");
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:uart_Deck_Estop
DESCRIPTION		:Send ESTOP command to roboteq controller.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/10/4
*************************************************************************************/
uint8_t uart_Deck_Estop(){
	/********* STOP motor 1/2 and enter ESTOP mode**********/
	if (uart_Deck_Stop() == 0){
		return 0;
	}
	if (uart_Lifter_Stop() == 0){
		return 0;
	}
	uartRbSend(deck_uart, "!EX\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:uart_Motor_Release
DESCRIPTION		:Send Release command to roboteq controller to resume controller.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t uart_Motor_Release(){
	/********* Release ESTOP mode **********/
	uartRbSend(device_uart, "!MG\r");
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:uart_Deck_Release
DESCRIPTION		:Send Release command to roboteq controller to resume controller.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/10/4
*************************************************************************************/
uint8_t uart_Deck_Release(){
	/********* Release ESTOP mode **********/
	uartRbSend(deck_uart, "!MG\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:NV_Pt_On
DESCRIPTION		:Turns selected Port/Led On
INPUT			:Pt:LED to be Port/Led on
				This parameter can be one of the following values:
					LED1
					LED2
					LED3
					OUT1
					OUT2
					OUT3
					OUT4
					OUT5
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void NV_Pt_On(Pt_TypeDef Pt)
{
  HAL_GPIO_WritePin(GPIO_PORT[Pt], GPIO_PIN[Pt], GPIO_PIN_SET);
}

/************************************************************************************
FUNCTION		:NV_Pt_Off
DESCRIPTION		:Turns selected Port/Led Off.
INPUT			:Pt: LED to be Port/Led off
				This parameter can be one of the following values:
					LED1
					LED2
					LED3
					OUT1
					OUT2
					OUT3
					OUT4
					OUT5
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void NV_Pt_Off(Pt_TypeDef Pt)
{
  HAL_GPIO_WritePin(GPIO_PORT[Pt], GPIO_PIN[Pt], GPIO_PIN_RESET);
}

/************************************************************************************
FUNCTION		:NV_Pt_Toggle
DESCRIPTION		:Toggles the selected Port/Led.
INPUT			:Pt: Port/Led to be toggled
				This parameter can be one of the following values:
					LED1
					LED2
					LED3
					OUT1
					OUT2
					OUT3
					OUT4
					OUT5
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void NV_Pt_Toggle(Pt_TypeDef Pt)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Pt], GPIO_PIN[Pt]);
}

/************************************************************************************
FUNCTION		:Pc_Cmd_Process
DESCRIPTION		:Process commands from PC..
INPUT			:None
OUTPUT			:None
UPDATE			:2022/6/16
*************************************************************************************/
void Pc_Cmd_Process(){
    switch(PcData[CmdIDIdx])
    {
        case PC_CMDID_VELOCITY:
        	velcmd_timeout = VELCMD_TIMEOUT;
			memcpy(&velocity.linear_v.bytes[0],&PcData[DataIdx],sizeof(velocity.linear_v.f)+sizeof(velocity.angular_v.f));
			Velocity_Process();
			break;

        case PC_CMDID_POSE: //handler for the motor_pose command 
        	if(motion_exec_state == MOTION_IDLE || motion_exec_state == MOTION_START){
				memcpy(&pose.x.bytes[0],&PcData[DataIdx],sizeof(pose.x.f)+sizeof(pose.y.f)+sizeof(pose.theta.f)+sizeof(pose.c_dis.f)+sizeof(pose.f_dis.f)+sizeof(pose.velocity));
				debugLog("Receiving pose data from PC succeeds - Initial Command\n");
				qr_interval = pose.c_dis.f;
				pose_2final_enc_cnt = 0;
				remaining_distance = 1.0;
				if(pose.f_dis.f < 0.01){
				Calculate_Static_Delta_Distance();}
				Local_Path_Planning(); //Create the control points of the AGV 
        	}else{
				memcpy(&pose.x.bytes[0],&PcData[DataIdx],sizeof(pose.x.f)+sizeof(pose.y.f)+sizeof(pose.theta.f)+sizeof(pose.c_dis.f)+sizeof(pose.f_dis.f)+sizeof(pose.velocity));
				debugLog("Receiving pose data from PC succeeds - Initial Command\n");
				qr_interval = pose.c_dis.f;
				pose_2final_enc_cnt = 0;
				//remaining_distance = pose.f_dis.f;
				remaining_distance = 1.0;
				if(pose.f_dis.f < 0.01){
				Calculate_Static_Delta_Distance();}
				Local_Path_Planning();
        	}
            break;

        case PC_CMDID_ROTATION:
			memcpy(&rotation.angle.bytes[0],&PcData[DataIdx],sizeof(rotation.angle.f)+sizeof(rotation.deck_sync));
			debugLog("Receiving rotation data from PC succeeds\n");
			Motor_Rotation();
            break;

        case PC_CMDID_DECK_HOME:
			debugLog("Receiving Home request from PC succeeds\n");
			Deck_Home();
            break;

        case PC_CMDID_DECK_ROTATION:
        	rotation.deck_sync = 0;	// For determining whether it is sync with wheel rotation
			memcpy(&deck_rotation_angle.bytes[0],&PcData[DataIdx],sizeof(deck_rotation_angle.f));
			debugLog("Receiving Deck Rotation from PC succeeds\n");
			Deck_Rotation();
            break;

        case PC_CMDID_DECK_UPDOWN:
			lifter_updn = PcData[DataIdx];
			debugLog("Receiving Deck Up/down info from PC succeeds\n");
			Lifter_Updown();
            break;

        default:
			debugLog("Wrong command\n");
    }
}
/************************************************************************************
FUNCTION		:Velocity_Process
DESCRIPTION		:Based on velocity info from PC, convert to RPM and send command to Motor Controller.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/6/16
*************************************************************************************/
uint8_t Velocity_Process(){
	if (isStopOn == TRUE || isEstopOn == TRUE){
		cmd_sts = PC_CMDSTS_ERROR;
		return 0;
	}else if (isSlowOn == TRUE && velocity.linear_v.f > motorpara.slow_linear_v.f) {
		cmd_sts = PC_CMDSTS_COMPLETED;
		return 1;
	}else{
		if(isSpeedmode == 0){
			if (Configure_Motor_Speedmode() == 0){
				cmd_sts = PC_CMDSTS_ERROR;
				return 0;
			}
			isPositionmode = 0;
			isSpeedmode = 1;
		}
		Velocity2Rpm(velocity.linear_v.f,velocity.angular_v.f);			// Get speed data (m/s) from PC, need to calculate Power data or RPM data
		if (Send_Wheel_Command() != 1){
			printf("Wheel command sending error \n");
			cmd_sts = PC_CMDSTS_ERROR;
			return 0;
		}
		cmd_sts = PC_CMDSTS_COMPLETED;
	}
	return 1;
}

/************************************************************************************
FUNCTION		:Read_Para_Query
DESCRIPTION		:Based on query request from PC, read parameter from Roboteq controller.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/9/7
*************************************************************************************/
uint8_t Read_Para_Query(){
	wheel_read_error = 0;
	switch (para_query)
	{
		case PARA_QUERY_AMPS:
        	uartRbSend(device_uart, "?A 1\r");
        	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
        	{
        		wheel_read_error = 1;
        	}
        	motor_amps_1 = motor_read_buf;
        	uartRbSend(device_uart, "?A 2\r");
        	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
        	{
        		wheel_read_error = 1;
        	}
        	motor_amps_2 = motor_read_buf;
			break;

		case PARA_QUERY_BATTERY_AMPS:
        	uartRbSend(device_uart, "?BA 1\r");
        	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
        	{
        		wheel_read_error = 1;
        	}
        	battery_amps_1 = motor_read_buf;
        	uartRbSend(device_uart, "?BA 2\r");
        	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
        	{
        		wheel_read_error = 1;
        	}
        	battery_amps_2 = motor_read_buf;
			break;

		case PARA_QUERY_POWER:
        	uartRbSend(device_uart, "?P 1\r");
        	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
        	{
        		wheel_read_error = 1;
        	}
        	power_1 = motor_read_buf;
        	uartRbSend(device_uart, "?P 2\r");
        	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
        	{
        		wheel_read_error = 1;
        	}
        	power_2 = motor_read_buf;
			break;
        default:
			debugLog("Wrong command\n");
	}
	uart_Para2PC();
	return 1;
}

/************************************************************************************
FUNCTION		:uart_Para2PC
DESCRIPTION		:MCU send query parameter to PC.
INPUT			:None
OUTPUT			:1:OK; 0:NG
UPDATE			:2022/9/6
*************************************************************************************/
uint8_t uart_Para2PC( ){
	//After receive data from Roboteq, send data to PC per 50ms.
	uint8_t n=0;

	Snd2Pc[n++]='G';
	Snd2Pc[n++]='R';
	Snd2Pc[n++]=TYPE_STATUS;
	Snd2Pc[n++]=MOTOR_QUERYPARA_LENGTH;
	Snd2Pc[n++]=MCU_CMDID_QUERY;
	Snd2Pc[n++]=para_query;
	if(wheel_read_error == 0){
		if(para_query == PARA_QUERY_AMPS){
			Snd2Pc[n++]=(uint8_t)((motor_amps_1>>8)&0xFF);
			Snd2Pc[n++]=(uint8_t)(motor_amps_1&0xFF);
			Snd2Pc[n++]=(uint8_t)((motor_amps_2>>8)&0xFF);
			Snd2Pc[n++]=(uint8_t)(motor_amps_2&0xFF);
		}else if(para_query == PARA_QUERY_BATTERY_AMPS){
			Snd2Pc[n++]=(uint8_t)((battery_amps_1>>8)&0xFF);
			Snd2Pc[n++]=(uint8_t)(battery_amps_1&0xFF);
			Snd2Pc[n++]=(uint8_t)((battery_amps_2>>8)&0xFF);
			Snd2Pc[n++]=(uint8_t)(battery_amps_2&0xFF);
		}else if(para_query == PARA_QUERY_POWER){
			Snd2Pc[n++]=(uint8_t)((power_1>>8)&0xFF);
			Snd2Pc[n++]=(uint8_t)(power_1&0xFF);
			Snd2Pc[n++]=(uint8_t)((power_2>>8)&0xFF);
			Snd2Pc[n++]=(uint8_t)(power_2&0xFF);
		}else{
			Snd2Pc[n++]=0xFF;
			Snd2Pc[n++]=0xFF;
			Snd2Pc[n++]=0xFF;
			Snd2Pc[n++]=0xFF;
		}
	}
	Snd2Pc[n++]=uart_ucGetCheckSum(&Snd2Pc[DataIdx],MOTOR_QUERYPARA_LENGTH+PcDataLength_SIZE+PcCmdID_SIZE);
	Snd2Pc[n++]='\r';
	uartPcSend(Snd2Pc,n);
	return 1;
}
/************************************************************************************
FUNCTION		:Local_Path_Planning
DESCRIPTION		:Based on the pose info from PC, plan local path.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/5/20
*************************************************************************************/
uint8_t Local_Path_Planning()
{
	if(Absolute_f(pose.theta.f) > 5.0){
		cmd_sts = PC_CMDSTS_ABORTED;
		return 0;
	}
	create_section_constraints(); //Create section constraints according to the motion_exec_state and the command
	Generate_Bezier_Points(pose.x.f, pose.y.f, pose.theta.f, pose.c_dis.f);
	cmd_sts = PC_CMDSTS_INPROGRESS; 
	return 1;
}
/************************************************************************************
FUNCTION		:Global_Planning
DESCRIPTION		:Based on the pose info and velocity from PC, plan QR interval for each section.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/5/20
*************************************************************************************/
void Global_Planning(){
	uint8_t count = pose.f_dis.f;
	memset(posecmd_snd, 0,sizeof(posecmd_snd));
	pose_2final_enc_cnt = 0;	// Initialize to indicate it is first pose command
	c_1st_dis = pose.c_dis.f;	// First current distance backup
	Calculate_Static_Delta_Distance();	// Use velocity and deceleration rate to estimate the starting point of STOP
	for(uint8_t i = 0; i < count; i++){
		posecmd_snd[i] = QR_INTERVAL_1;
	}
}
/************************************************************************************
FUNCTION		:Pose_Init
DESCRIPTION		:Initialize pose calculation related parameters
INPUT			:None
OUTPUT			:None
UPDATE			:2022/9/2
*************************************************************************************/
void Pose_Init(){
//	Set_R_Coefficient();
#if FINAL_DISTANCE_COMP
	float dis_offset, dis_sqrt;
#endif
	// Set_R_Coefficient_1st_Meter();
	// r_offset = r_offset_debug;
	t_delta = DELTA_TIME;
	then = 0.0;
	enc_left = 0;
	enc_right = 0;
	dx = 0;
	dr = 0;
	x_final = pose.x.f;
	y_final = pose.y.f;
	previous_x = pose.x.f;
	previous_y = pose.y.f;
	theta_final = pose.theta.f * ONEDEDREE_RADIAN;
	pose_start_enc_cnt = left;
	linear_speed = 0;
#if FINAL_DISTANCE_COMP
	dis_offset = (pose.c_dis.f - x_final)*Absolute_f(sin(pose.theta.f  * ONEDEDREE_RADIAN + atan(pose.y.f/(pose.c_dis.f - x_final))));
	dis_sqrt = sqrt((pose.c_dis.f - x_final) * (pose.c_dis.f - x_final) + dis_offset * dis_offset);
#endif
	if((static_delta_dis * delta_offset) > pose.f_dis.f && pose_2final_enc_cnt == 0){
#if FINAL_DISTANCE_COMP
		pose_2final_enc_cnt = (int32_t) ((dis_sqrt) * ticks_meter * ticks_offset);
#else
		pose_2final_enc_cnt = (int32_t) ((pose.c_dis.f - x_final) * ticks_meter * ticks_offset);
#endif
	}else{
		pose_2final_enc_cnt = 0;
	}
#if LOG_DECK
	log_d_i = 0;
	memset(matrix_d_rpm,0,sizeof(matrix_d_rpm));
#endif
	keep_straight = POSE_SPEED_CORRECTION;
	x_spd = 0;
	r_spd = 0;
	elapse_time = 0;
	time_count = 0;
	// is1stPosepass = 0;
	if (pose.velocity.linear_v.f != 0 && wheel_macc !=0){
		time_interval = (qr_interval / pose.velocity.linear_v.f) + 2.0*pose.velocity.linear_v.f / motorpara.wheel_circumference.f * M2SECS * motorpara.gear_ratio / wheel_macc; //Test
		// time_interval_min = qr_interval / pose.velocity.linear_v.f;
	}else{
		time_interval = TIME_INTERVAL2;	//Default to 1.5 seconds
		// time_interval_min = TIME_INTERVAL2;
	}
}
/************************************************************************************
FUNCTION		:Pose_2nd_Init
DESCRIPTION		:2nd initialize pose calculation related parameters
INPUT			:None
OUTPUT			:None
UPDATE			:2022/9/2
*************************************************************************************/
void Pose_2nd_Init(){
#if FINAL_DISTANCE_COMP
	float dis_offset, dis_sqrt;
#endif
	// Set_R_Coefficient();
	// r_offset = r_offset_debug;
	// x_final = x_final + pose.x.f;
	x_final = pose.x.f;
	y_final = pose.y.f;
	theta_final = pose.theta.f * ONEDEDREE_RADIAN;
	pose_start_enc_cnt = left;
#if FINAL_DISTANCE_COMP
	dis_offset = (pose.c_dis.f - x_final)*Absolute_f(sin(pose.theta.f  * ONEDEDREE_RADIAN + atan(pose.y.f/(pose.c_dis.f - x_final))));
	dis_sqrt = sqrt((pose.c_dis.f - x_final) * (pose.c_dis.f - x_final) + dis_offset * dis_offset);
#endif
	if((static_delta_dis * delta_offset) > pose.f_dis.f && pose_2final_enc_cnt == 0 ){
#if FINAL_DISTANCE_COMP
		pose_2final_enc_cnt = (int32_t) ((dis_sqrt) * ticks_meter * ticks_offset);
#else
		pose_2final_enc_cnt = (int32_t) ((pose.c_dis.f - x_final) * ticks_meter * ticks_offset);
#endif
	}else{
		pose_2final_enc_cnt = 0;
	}
	keep_straight = POSE_SPEED_CORRECTION;
	elapse_time = 0;
	time_count = 0;
	if (pose.velocity.linear_v.f != 0){
		time_interval = (qr_interval / pose.velocity.linear_v.f) * 1.5; //Test
		// time_interval_min = qr_interval / pose.velocity.linear_v.f;
	}else{
		time_interval = TIME_INTERVAL2;	//Default to 1.5 seconds
		// time_interval_min = TIME_INTERVAL2;
	}
}

/************************************************************************************
FUNCTION		:Set_R_Coefficient()
DESCRIPTION		:Read r coefficient values from lookup table
INPUT			:None
OUTPUT			:None
UPDATE			:2022/11/24
*************************************************************************************/
void Set_R_Coefficient_1st_Meter(){
	uint8_t speed = (((uint8_t)(pose.velocity.linear_v.f * 10))%13) - 1;
	uint8_t mode_theta_y = 0;
	int32_t delta_y = 0;
	int32_t delta_theta = 0;
	uint8_t pos_neg = 0;

	if ((pose.y.f > 0 && pose.theta.f > 0) || (pose.y.f < 0 && pose.theta.f < 0)){
		pos_neg = 1;
	}else{
		pos_neg = 0;
	}

	if(Absolute_f(pose.y.f) < Y_ZERO_THRESHOLD){
		delta_y = 0;
	}else{
		if(Absolute_f(pose.y.f) > 0.004 && Absolute_f(pose.y.f) < 0.008){
			delta_y = 1;
		}else if (Absolute_f(pose.y.f) > 0.007 && Absolute_f(pose.y.f) < 0.012){
			delta_y = 2;
		}else if (Absolute_f(pose.y.f) > 0.011 && Absolute_f(pose.y.f) < 0.016){
			delta_y = 3;
		}else{
			delta_y = 4;
		}
	}
	if(Absolute_f(pose.theta.f) < THETA_ZERO_THRESHOLD){
		delta_theta = 0;
	}else{
		if(Absolute_f(pose.theta.f) > 0.25 && Absolute_f(pose.theta.f) < 1.1){
			delta_theta = 1;
		}else if (Absolute_f(pose.theta.f) > 1.05 && Absolute_f(pose.theta.f) < 2.1){
			delta_theta = 2;
		}else if (Absolute_f(pose.theta.f) > 2.05 && Absolute_f(pose.theta.f) < 3.1){
			delta_theta = 3;
		}else{
			delta_theta = 0;
		}

	}

	if((delta_y * delta_theta) > 0 ){
		if(pose.theta.f > THETA_ZERO_THRESHOLD){
			mode_theta_y = 0;
		}else{
			mode_theta_y = 1;
		}
	}else if (delta_y == 0) {
		if(pose.theta.f > THETA_ZERO_THRESHOLD){
			mode_theta_y = 2;
		}else if(pose.theta.f < -THETA_ZERO_THRESHOLD){
			mode_theta_y = 3;
		}else{
			mode_theta_y = 0;
		}
	}else{
		if(pose.theta.f > THETA_ZERO_THRESHOLD){
			mode_theta_y = 4;
		}else{
			mode_theta_y = 5;
		}
	}
	if(speed > 11 || speed < 0){
		speed = 0;
	}
	if(pos_neg == 1){
		if(delta_theta == 0){
			r_offset = r_coefficient_1st_meter_theta0[delta_y][speed];
		}else if(delta_theta == 1){
			r_offset = r_coefficient_1st_meter_theta1[delta_y][speed];
		}else if (delta_theta == 2){
			r_offset = r_coefficient_1st_meter_theta2[delta_y][speed];
		}else{
			r_offset = r_coefficient_1st_meter_theta3[delta_y][speed];
		}
	}else{
		if(delta_theta == 0){
			r_offset = r_coefficient_1st_meter_theta0_N[delta_y][speed];
		}else if(delta_theta == 1){
			r_offset = r_coefficient_1st_meter_theta1_N[delta_y][speed];
		}else if (delta_theta == 2){
			r_offset = r_coefficient_1st_meter_theta2_N[delta_y][speed];
		}else{
			r_offset = r_coefficient_1st_meter_theta3_N[delta_y][speed];
		}
	}

//	r_offset = r_offset_debug;
}
/************************************************************************************
FUNCTION		:Set_R_Coefficient()
DESCRIPTION		:Read r coefficient values from lookup table
INPUT			:None
OUTPUT			:None
UPDATE			:2022/10/11
*************************************************************************************/
void Set_R_Coefficient(){
	uint8_t speed = (((uint8_t)(pose.velocity.linear_v.f * 10))%13) - 1;
	uint8_t mode_theta_y = 0;
	int32_t delta_y = 0;
	int32_t delta_theta = 0;

	if(Absolute_f(pose.y.f) < Y_ZERO_THRESHOLD){
		delta_y = 0;
	}else{
		if(pose.y.f > Y_ZERO_THRESHOLD){
			delta_y = 1;
		}else{
			delta_y = -1;
		}
	}
	if(Absolute_f(pose.theta.f) < THETA_ZERO_THRESHOLD){
		delta_theta = 0;
	}else{
		if(pose.theta.f > THETA_ZERO_THRESHOLD){
			delta_theta = 1;
		}else{
			delta_theta = -1;
		}
	}
	if((delta_y * delta_theta) > 0 ){
		if(pose.theta.f > THETA_ZERO_THRESHOLD){
			mode_theta_y = 0;
		}else{
			mode_theta_y = 1;
		}
	}else if (delta_y == 0) {
		if(pose.theta.f > THETA_ZERO_THRESHOLD){
			mode_theta_y = 2;
		}else if(pose.theta.f < -THETA_ZERO_THRESHOLD){
			mode_theta_y = 3;
		}else{
			mode_theta_y = 0;
		}
	}else{
		if(pose.theta.f > THETA_ZERO_THRESHOLD){
			mode_theta_y = 4;
		}else{
			mode_theta_y = 5;
		}
	}
	if(speed > 11 || speed < 0){
		speed = 0;
	}
	if(Absolute_f(pose.theta.f) < TABLE1_THETA_THRESHOLD){
		r_offset = r_coefficient_1[mode_theta_y][speed];
	}else if(Absolute_f(pose.theta.f) < TABLE2_THETA_THRESHOLD){
		r_offset = r_coefficient_2[mode_theta_y][speed];
	}else{
		r_offset = r_coefficient_3[mode_theta_y][speed];
	}
//	r_offset = r_offset_debug;
}
/************************************************************************************
FUNCTION		:Pose_Update
DESCRIPTION		:Calculate pose
INPUT			:None
OUTPUT			:None
UPDATE			:2022/9/2
*************************************************************************************/
void Pose_Update(){
	elapse_time += t_delta;
	time_count++;

	float now = elapse_time;
	float elapsed;
	float d_left, d_right, d, th, x, y;
	elapsed = now - then;
	if(enc_left == 0){
	  d_left = 0;
	  d_right = 0;
	}else{
	  d_left = (left - enc_left) / (ticks_meter*ticks_offset);
	  d_right = (right - enc_right) / (ticks_meter*ticks_offset);
	}
	enc_left = left;
	enc_right = right;
	d = (d_left + d_right) / 2.0;
	th = (d_right - d_left) / motorpara.track_width.f;
	dx = d / elapsed;
	dr = th / elapsed;
	if (d != 0)
	{
	  x = cos(th) * d;
	  y = -sin(th) * d;
	  // calculate the final position of the robot
	  x_final = x_final + (cos(theta_final) * x - sin(theta_final) * y);
	  y_final = y_final + (sin(theta_final) * x + cos(theta_final) * y);
	}
	if (th != 0){
	  theta_final = theta_final + th;
	}
	then = now;
#if ENABLE_POSE_AUTO_4M
	if(x_final > 2.1 && is1stPosepass == 0){
		is1stPosepass = 1;
		if(theta_y_mode == 0){
			pose.x.f = 0.1;
			pose.y.f = Y_BIAS;
			pose.theta.f = THETA_BIAS;
			pose.c_dis.f = 4.0;
			pose.f_dis.f = 4.0;
		}else if(theta_y_mode == 1){
			pose.x.f = 0.1;
			pose.y.f = -Y_BIAS;
			pose.theta.f = -THETA_BIAS;
			pose.c_dis.f = 4.0;
			pose.f_dis.f = 4.0;
		}else if(theta_y_mode == 2){
			pose.x.f = 0.1;
			pose.y.f = Y_BIAS;
			pose.theta.f = -THETA_BIAS;
			pose.c_dis.f = 4.0;
			pose.f_dis.f = 4.0;
		}else if(theta_y_mode == 3){
			pose.x.f = 0.1;
			pose.y.f = -Y_BIAS;
			pose.theta.f = THETA_BIAS;
			pose.c_dis.f = 4.0;
			pose.f_dis.f = 4.0;
		}else{
			pose.x.f = 0.1;
			pose.y.f = 0.015;
			pose.theta.f = 3.0;
			pose.c_dis.f = 4.0;
			pose.f_dis.f = 4.0;
		}

		Calculate_Delta_Distance();
		qr_interval = QR_INTERVAL_2;
		Local_Path_Planning();
	}
#endif
}
/************************************************************************************
FUNCTION		:Generate_Bezier_Points
DESCRIPTION		:Generate 4 Bezier points.
INPUT			:x: current pose.x.f, y:current pose.y.f, theta: current pose.theta.f 
OUTPUT			:None
UPDATE			:2022/8/31
*************************************************************************************/
void Generate_Bezier_Points(float x, float y, float theta, float travel_distance)
{
	P0[0] = x;
	P0[1] = y;
	P3[0] = QR_INTERVAL_1;// ending point is (1.0,0) 
	P3[1] = 0;
	float offset = 1.0;
	float starting_yaw = theta * ONE_DEGREE_RADIAN; 
	float ending_yaw = 0; 
	float dist = hypot((P0[0] - P3[0]), (P0[1] - P3[0])) / offset ; 
	P1[0] = P0[0] + dist * cos(starting_yaw);
	P1[1] = P0[1] + dist * sin(starting_yaw); 
	P2[0] = P3[0] - dist * cos(ending_yaw); 
	P2[1] = P3[1] - dist * sin(ending_yaw); 
	controlPoints[0] = P1[0];
	controlPoints[1] = P1[1];
	controlPoints[2] = P2[0];
	controlPoints[3] = P2[1];
}
/************************************************************************************
FUNCTION		:Calculate_Linear_Velocity
DESCRIPTION		:Calculate Linear Velocity for X and Y.
INPUT			:diff10, diff21, diff32, t
OUTPUT			:x speed or y speed
UPDATE			:2022/8/31
*************************************************************************************/
float Calculate_Linear_Velocity(float diff10, float diff21, float diff32, float t)
{
/*#if
	float B02 = 2 * (1 - t);
	float B12 = 2 * t;

	return (B02 * diff10 + B12 * diff21); //directional speed
	float B02 = 3 * (1 - t)* (1 - t);
	float B12 = 6 * (1 - t) * t;
	float B22 = 3 * t * t;

	return (B02 * diff10 + B12 * diff21 + B22 * diff32); //directional speed
#endif */
}
/************************************************************************************
FUNCTION		:Calculate_Velocity_Rotation
DESCRIPTION		:Calculate angular Velocity for z.
INPUT			:t
OUTPUT			:r speed
UPDATE			:2022/8/31
*************************************************************************************/
void Calculate_Velocity_Rotation(float t)//Calculation of the rotational speed
{
#if BEZIER_3_POINTS
	float Px1 = Calculate_Linear_Velocity(delta_x[0], delta_x[1], 0, t);
	float Py1 = Calculate_Linear_Velocity(delta_y[0], delta_y[1], 0, t);

	float B01 = -2;
	float B11 = 2;

	float Px2 = B01 * delta_x[0] + B11 * delta_x[1];
	float Py2 = B01 * delta_y[0] + B11 * delta_y[1];
	x_spd = Px1;
	r_spd = (Px1 * Py2 - Px2 * Py1) / (Px1 * Px1 + Py1 * Py1);
	float Px1 = Calculate_Linear_Velocity(delta_x[0], delta_x[1], delta_x[2], t);
	float Py1 = Calculate_Linear_Velocity(delta_y[0], delta_y[1], delta_y[2], t);

	float B01 = 6 * (t - 1);
	float B11 = 6 * (1 - 2 * t);
	float B21 = 6 * t;

	float Px2 = B01 * delta_x[0] + B11 * delta_x[1] + B21 * delta_x[2];
	float Py2 = B01 * delta_y[0] + B11 * delta_y[1] + B21 * delta_y[2];
	
	if((linear_speed < pose.velocity.linear_v.f) && (elapse_time <= ACCEL_TICKS * t_delta)){
		linear_speed = pose.velocity.linear_v.f;  
	}else{
		linear_speed = sqrt(((x_final - previous_x)*(x_final - previous_x)) + ((y_final - previous_y)*(y_final - previous_y))) / t_delta;
	}
	// x_spd = Px1;
	previous_x = x_final;
	previous_y = y_final;
	r_spd = (Px1 * Py2 - Px2 * Py1) / (Px1 * Px1 + Py1 * Py1);
#endif
}

// Calculate the binomial coefficient
int binomialCoeff(int n, int k)
{
    int res = 1;
    if (k > n - k)
        k = n - k;
    for (int i = 0; i < k; ++i)
    {
        res *= (n - i);
        res /= (i + 1);
    }
    return res;
}

//Calculate the new polynomial 
float ploynomial(float t, int n, int stage) {
	float value = pow((1 - t), n) * pow(t, (stage - n));
	return value;
}


// Calculate a point on the Bezier curve
void bezierCurve(float t, float* point, float* controlPoints)
{
	/*
    float s = 1.0 - t;
    point[0] = 0; 
    point[1] = 0;
	

    for (int i = 0; i <= BEZIER_ORDER; ++i)
    {
        int bin = binomialCoeff(BEZIER_ORDER, i);
        float b = bin * pow(t, i) * pow(s, BEZIER_ORDER - i);
        point[0] += b * controlPoints[i * 2];
        point[1] += b * controlPoints[i * 2 + 1];
    }
*/  
	point[0] = ploynomial(t, 3, 3) * P0[0] + ploynomial(t, 2, 3) * P1[0] + ploynomial(t, 1, 3) * P2[0] + ploynomial(t, 0, 3) * P3[0]; 
	point[1] = ploynomial(t, 3, 3) * P0[1] + ploynomial(t, 2, 3) * P1[1] + ploynomial(t, 1, 3) * P2[1] + ploynomial(t, 0, 3) * P3[1]; 
}


/************************************************************************************
FUNCTION		:	calc_control_command 
DESCRIPTION		:	using the PID method to correct the AGV position 
INPUT			:	x_diff: difference of the real x position to the calculated position 
                    y_diff: difference of the real y position to the calculated position 
					theta: heading of the AGV
					theta_goal: goal heading of the AGV 

OUTPUT			:	None
UPDATE			:	2023/1/30 
*************************************************************************************/
void calc_control_command(double x_diff, double y_diff, double theta, double theta_goal, double *rho, double *v, double *w) {
	*rho =  hypot(x_diff, y_diff);
	double alpha = (atan2(y_diff, x_diff) - theta + PI) - (int)((atan2(y_diff, x_diff) - theta + PI) / (2*PI)) * (2*PI);
	double beta = (theta_goal - theta - alpha + PI) - (int)((theta_goal - theta - alpha + PI) / (2*PI)) * (2*PI);
	*v = Kp_rho * (*rho);
	*w = Kp_alpha * alpha - Kp_beta * beta;
	if (alpha > PI / 2 || alpha < -PI / 2) {
	*v = -(*v);
	}
} 
/************************************************************************************
FUNCTION		:	get_current_pose
DESCRIPTION		:	read current pose using the incremental method
INPUT			:	starting_pose: the initial pose for the calculation 
                    left_enc: left wheel encoder count 
					right_enc: right wheel encoder count 
OUTPUT			:	current_pose: current pose using the incremental method calculation 
UPDATE			:	2022/12/27
*************************************************************************************/
struct points get_current_pose(struct points starting_pose) {
	struct points _temp_pose = starting_pose; 
	float d_left, d_right, d, th, x, y;
	if (enc_left == 0) {
		d_left = 0;
		d_right = 0;
	}
	else {
		d_left = (left - enc_left) / (ticks_meter * ticks_offset);
		d_right = (right - enc_right) / (ticks_meter * ticks_offset);
	}
	enc_left = left;
	enc_right = right;
	d = (d_left + d_right) / 2.0;
	th = (d_right - d_left) / motorpara.track_width.f;
	if (d != 0)
	{
		x = cos(th) * d;
		y = -sin(th) * d;
		// calculate the final position of the robot
		_temp_pose.x = _temp_pose.x + (cos(theta_final) * x - sin(theta_final) * y);
		_temp_pose.y = _temp_pose.y + (sin(theta_final) * x + cos(theta_final) * y);
	}
	if (th != 0) {
		theta_final = theta_final + th;
	}

	current_pose.x = _temp_pose.x; 
	current_pose.y = _temp_pose.y; 
}



/************************************************************************************
FUNCTION		:	setVelocityProfile
DESCRIPTION		:	velocity profile function
INPUT			:	target_velocity: the desired final velocity of the robot in m/s
					current_velocity: the current velocity of the robot in m/s
					acceleration: the acceleration of the robot in m/s^2
					left_velocity: a pointer to a float where the calculated left wheel velocity will be stored
					right_velocity: a pointer to a float where the calculated right wheel velocity will be stored
OUTPUT			:	float pointer of left and right velocity
UPDATE			:	2022/12/27
*************************************************************************************/
void setVelocityProfile(float target_velocity, float current_velocity, float acceleration, float* left_velocity, float* right_velocity) {
  // Calculate the time it will take to reach the target velocity
  float time = (target_velocity - current_velocity) / acceleration;

  // Calculate the left and right wheel velocities
  *left_velocity = (2 * target_velocity - time * acceleration) / 2;
  *right_velocity = (2 * target_velocity + time * acceleration) / 2;
}
/************************************************************************************
FUNCTION		:sgn
DESCRIPTION		:Return the sign of the function 
INPUT			:a double number 
OUTPUT			:the sign of the number 
UPDATE			:2023/1/30 
*************************************************************************************/
int sgn(double x) { //Returns the sign of the x 
	if (x > 0) {
		return 1;
	}
	else if (x == 0)
	{
		return 0;
	}
	else if (x < 0) {
		return -1;
	}
	else
	{
		return 0;
	}
}

/************************************************************************************
FUNCTION		:min
DESCRIPTION		:return the min value of the two given number 
INPUT			:two double number 
OUTPUT			:the smaller value of the two number 
UPDATE			:2023/1/30
*************************************************************************************/
float min(float a, float b) {//Return the min value between a and b 
	if (a < b) {
		return a;
	}
	else {
		return b;
	}
} 

/************************************************************************************
FUNCTION		:max
DESCRIPTION		:return the max value of the two given number
INPUT			:two double number
OUTPUT			:the bigger value of the two number
UPDATE			:2023/1/30
*************************************************************************************/
float max(float a, float b) {
	if (a > b) {
		return a;
	}
	else {
		return b;
	}
}

/************************************************************************************
FUNCTION		:generation_convolute_params
DESCRIPTION		:return the convolution parameters for the convolution pratice 
INPUT			:physical constraint of the AGV 
OUTPUT			:convolution parameters of the AGV 
UPDATE			:2023/1/30
*************************************************************************************/
struct convolution_params generation_convolute_params(struct design_params param)
{

	double accleration_constant = param.accleration_constraint;
	double jerk_constant = param.jerk_constraint;
	double v_n = param.v_max;
	double v_i = param.v_s;
	double v_f = param.v_f;
	double Sn = param.Sn;

	double t2 = accleration_constant / jerk_constant;
	double t1_star = (v_n - sgn(v_i * v_f) * min(v_i, v_f)) / accleration_constant;
	double Sn_star = (v_i + v_f) * (t1_star + t2) / 2.0;

	double t1, t0, v0;
	//Calculate t1 
	if (Sn == Sn_star) {
		t1 = 2 * v_n / accleration_constant;
	}
	else if (Sn > Sn_star) {
		double a = (v_n - v_f) / accleration_constant;
		double b = (v_n - v_i) / accleration_constant;
		t1 = max(a, b);
	}
	else if (Sn < Sn_star) {
		double a = (v_n + v_f) / accleration_constant;
		double b = (v_n + v_i) / accleration_constant;
		t1 = max(a, b);
	}


	//Caculate v0 
	if (Sn > Sn_star) {
		v0 = v_n - v_i;
	}
	else {
		v0 = -v_n - v_i;
	}

	//Calculate t0 
	t0 = ((sgn(Sn - Sn_star)) * (Sn - 0.5 * (v_i + v_f) * (t1 + t2))) / v_n;

	struct convolution_params _convol_params; 
	_convol_params.t0 = t0; 
	_convol_params.t1 = t1; 
	_convol_params.t2 = t2; 
	_convol_params.v0 = v0; 

	return _convol_params;
}

/************************************************************************************
FUNCTION		:travel_time_est 
DESCRIPTION		:estimate the maximum travelling time of the AGV 
INPUT			:convolution parameters 
OUTPUT			:the sum of t0, t1 and t2 
UPDATE			:2023/1/30
*************************************************************************************/
double travel_time_est(struct convolution_params params) { //Calculate the estimated travel time  
	return params.t0 + params.t1 + params.t2; 
} 
/************************************************************************************
FUNCTION		:get_distance
DESCRIPTION		:calculate the distance between two given points
INPUT			:points coordinates
OUTPUT			:distance
UPDATE			:2023/1/30
*************************************************************************************/
float get_distance(double x1,double y1,double x2,double y2) {
	return sqrt(pow((x1 - x2),2) + pow((y1 - y2),2));
}

/************************************************************************************
FUNCTION		:pose_based_speed_correction 
DESCRIPTION		:correction of the pose based on the currect pose and the following point 
INPUT			:pose data from the onboard controller and calculated pose 
OUTPUT			:delta x and delta y value 
UPDATE			:2023/1/30
*************************************************************************************/
struct correction_data pose_based_speed_correction(struct pos current_pose, struct pos calculated_pose)
{
	struct correction_data correction;
	correction.x_delta = current_pose.x.f - calculated_pose.x.f; 
	correction.y_delta = current_pose.y.f - calculated_pose.y.f; 
	return correction; 	
}
/************************************************************************************
FUNCTION		:theta_correction 
DESCRIPTION		:correction of the heading from the onboard 
INPUT			:pose data from the onboard controller and calculated pose
OUTPUT			:delta x and delta y value
UPDATE			:2023/1/30
*************************************************************************************/
float theta_correction() {
	Pose_Update(); 
	return atan2(y_final, x_final); 
}



/************************************************************************************
FUNCTION		:x_correction_pid
DESCRIPTION		:using the pid control to generate the Sn value of each travelling 
INPUT			:pose data from the PC
OUTPUT			:corrected Sn value for the 
UPDATE			:2023/1/30
*************************************************************************************/
float x_correction_pid(float  x_ideal,float x_err, float kp, float ki, float kd) {
	float sampling_time = x_ideal / pose.velocity.linear_v.f; 
	err_sum += x_err; 
	return x_ideal + kp * x_err + (err_sum*ki)/sampling_time; //The PID result of the corrected x
} 

float heading_pid(float heading_err, float kp, float ki, float kd) {
	heading_err = heading_err * ONE_DEGREE_RADIAN; 
	return heading_err * kp; 
}



/************************************************************************************
FUNCTION		:create_section_constraints 
DESCRIPTION		:create constraints for each sections 
INPUT			:pose command from computer 
OUTPUT			:constraints for this section 
UPDATE			:2023/1/30
*************************************************************************************/

void create_section_constraints(void) {
#if ORIGINAL_STATE_MACHINE
	if (pose.c_dis.f == 1 && pose.f_dis.f == 0){//Only travel 1 meter
		agv_constraint.section_type = ONLY_ONE_METER; 
		agv_constraint.Sn = x_correction_pid(0.835,pose.x.f,-1.5,0,0); //Sn distance (1.055-kp*pose.x.f)
		agv_constraint.v_f = 0;
		agv_constraint.v_max = pose.velocity.linear_v.f; //Current max speed allowed 
		agv_constraint.jerk_constraint = 3;
		agv_constraint.accleration_constraint = ACCERATE_CONSTRAINT_1_METER;
		agv_constraint.v_s = 0;
	}
	else if (pose.f_dis.f != 0 && pose.c_dis.f == 1){//Travel multiple meters in start up section 
		agv_constraint.section_type = START_UP;
		agv_constraint.Sn = x_correction_pid(0.835,pose.x.f,-1.5,-0.001,0); //QR distance
		agv_constraint.v_f = pose.velocity.linear_v.f;
		agv_constraint.v_max = 0.85; //Current max speed allowed 
		agv_constraint.jerk_constraint = 3;
		agv_constraint.accleration_constraint = ACCERATE_CONSTRAINT_1_METER;
		agv_constraint.v_s = 0;
	}
	else if (pose.f_dis.f != 0 && pose.c_dis.f != 1) {//Travel multiple meters in middle section 
		agv_constraint.section_type = MIDDLE;
		agv_constraint.Sn = x_correction_pid(0.81,pose.x.f,-1.9,-0.002,0); //QR distance
		agv_constraint.v_f = pose.velocity.linear_v.f;
		agv_constraint.v_max = pose.velocity.linear_v.f; //Current max speed allowed 
		agv_constraint.jerk_constraint = 3;
		agv_constraint.accleration_constraint = ACCERATE_CONSTRAINT_MIDDLE;
		agv_constraint.v_s = agv_constraint.v_f;
	}
	else if (pose.f_dis.f == 0 && pose.c_dis.f != 0) {
		agv_constraint.section_type = DEC; 
		agv_constraint.Sn = x_correction_pid(1.0,pose.x.f,-1.9,-0.002,0); //QR distance
		agv_constraint.v_f = 0;
		agv_constraint.v_max = pose.velocity.linear_v.f; //Current max speed allowed 
		agv_constraint.jerk_constraint = 3;
		agv_constraint.accleration_constraint = ACCERATE_CONSTRAINT_DEC;
		agv_constraint.v_s = pose.velocity.linear_v.f;
	}
#endif 
	if (pose.f_dis.f == 0) {
		if (motion_exec_state == MOTION_IDLE) {
			//section_type: ONLY_1_METER 
			agv_constraint.section_type = ONLY_ONE_METER;
			agv_constraint.Sn = x_correction_pid(0.835, pose.x.f, -1.5, 0, 0); //Sn distance (1.055-kp*pose.x.f)
			agv_constraint.v_f = 0;
			agv_constraint.v_max = pose.velocity.linear_v.f; //Current max speed allowed 
			agv_constraint.jerk_constraint = 3;
			agv_constraint.accleration_constraint = ACCERATE_CONSTRAINT_1_METER;
			agv_constraint.v_s = 0;
			motion_exec_state = MOTION_INIT;//Starting from standstill 
		}
		else if (motion_exec_state == MOTION_START) {
			agv_constraint.section_type = DEC;
			agv_constraint.Sn = x_correction_pid(1.0, pose.x.f, -1.9, -0.002, 0); //QR distance
			agv_constraint.v_f = 0;
			agv_constraint.v_max = pose.velocity.linear_v.f; //Current max speed allowed 
			agv_constraint.jerk_constraint = 3;
			agv_constraint.accleration_constraint = ACCERATE_CONSTRAINT_DEC;
			agv_constraint.v_s = pose.velocity.linear_v.f;
		}
	}
	else {
		if (motion_exec_state == MOTION_IDLE) {
		  //section_type: START_UP
			agv_constraint.section_type = START_UP;
			agv_constraint.Sn = x_correction_pid(0.835, pose.x.f, -1.5, -0.001, 0); //QR distance
			agv_constraint.v_f = pose.velocity.linear_v.f;
			agv_constraint.v_max = 0.85; //Current max speed allowed 
			agv_constraint.jerk_constraint = 3;
			agv_constraint.accleration_constraint = ACCERATE_CONSTRAINT_1_METER;
			agv_constraint.v_s = 0;
			motion_exec_state = MOTION_INIT;
		} 
		else if (motion_exec_state == MOTION_START) {
			agv_constraint.section_type = MIDDLE;
			agv_constraint.Sn = x_correction_pid(0.81, pose.x.f, -1.9, -0.002, 0); //QR distance
			agv_constraint.v_f = pose.velocity.linear_v.f;
			agv_constraint.v_max = pose.velocity.linear_v.f; //Current max speed allowed 
			agv_constraint.jerk_constraint = 3;
			agv_constraint.accleration_constraint = ACCERATE_CONSTRAINT_MIDDLE;
			agv_constraint.v_s = agv_constraint.v_f;
		}
	}


}


/************************************************************************************
FUNCTION		:Motion_Action
DESCRIPTION		:State machine for actions monitor and execution.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/08/25
*************************************************************************************/
void Motion_Action()
{
	if (motion_exec_state == MOTION_IDLE){
	  return;
	}

	switch (motion_exec_state)
	{
		case MOTION_IDLE:
			break;

		case MOTION_INIT:
			Pose_Init();
			motion_exec_state = MOTION_START;
		case MOTION_START:
/*
#if BEZIER_CURVE_3POINTS
			if(elapse_time < time_interval){
				if((time_count%2) == 0){
					if(keep_straight == POSE_SPEED_CORRECTION){
						Calculate_Velocity_Rotation(elapse_time);
					}else{
						r_spd = 0;
					}
					Slowspeed_Update();
					if (Send_Wheel_Command() != 1){
						cmd_sts = PC_CMDSTS_ERROR;
						printf("Wheel command sending error \n");
						return;
					}
					cmd_sts = PC_CMDSTS_INPROGRESS;
				}
				Pose_Update();
				if((static_delta_dis * delta_offset) > pose.f_dis.f){
					Calculate_Delta_Distance();
#if 
					if((left - pose_start_enc_cnt + static_delta_dis * delta_offset * ticks_meter * ticks_offset) > pose_2final_enc_cnt){
						Calculate_Deceleration_Info();
						motion_exec_state = MOTION_DEC;
					}
#else

					if((x_final + delta_dis*delta_offset) > 0){
						Calculate_Deceleration_Info();
						motion_exec_state = MOTION_DEC;

}
#endif
				}
			}
#else */
		    //if((static_delta_dis * delta_offset) > pose.f_dis.f){
			//PathFinderController controller = {9, 15, 3};
		    	//double x = pose.x.f;
		    	//double y = pose.y.f;
		    	//double theta = pose.theta.f;
		    	//double rho = 0.0,v=0.0,w=0.0,x_diff=0.0;
				starting_point.x = pose.x.f; 
				starting_point.y = pose.y.f; 
				starting_point.yaw = pose.theta.f * ONE_DEGREE_RADIAN; 

		    	last_point[0] = pose.x.f; //Also the start points 
		    	last_point[1] = pose.y.f;
				point[0] = pose.x.f; point[1] = pose.y.f; //Starting position 
		    	double last_angle = pose.theta.f;  
				convol_params = generation_convolute_params(agv_constraint); 
				travel_time = travel_time_est(convol_params); 
				int M1 = (int)(convol_params.t1 * SAMPLES_PER_SECOND);
				int M2 = (int)(convol_params.t2 * SAMPLES_PER_SECOND);

				for (int i = 0; i <= MAX_POSSIBLE_SIZE; i++) {
					if (i <= (int)(convol_params.t0 * SAMPLES_PER_SECOND)) {
						if (agv_constraint.v_s != 0) {
							Y0[i] = convol_params.v0 - agv_constraint.v_s;
						}
						else {
							Y0[i] = convol_params.v0;
						}
					}
					else if (i <= (int)(travel_time * SAMPLES_PER_SECOND)) {
						if (agv_constraint.v_s != 0) {
							Y0[i] = agv_constraint.v_f - agv_constraint.v_s;
						}
						else {
							Y0[i] = agv_constraint.v_f;
						}
					}
				}

				for (int i = 0; i < MAX_POSSIBLE_SIZE; i++) {
					int start_index = i;
					int end_index = i - M1 + 1;
					float sum = 0;
					for (int j = end_index; j <= start_index; j++) {
						if (j >= 0 && j < MAX_POSSIBLE_SIZE) {
							sum += Y0[j];
						}
					}
					Y1[i] = sum / M1;
				} 

				int total_points_count = 0;
				real_curve_dist = 0; //Initize the curve distance count before a curve begins 
				//Convolution to create y2 as the linear speed function 
				for (int i = 0; i < MAX_POSSIBLE_SIZE; i++) {
					int start_index = i;
					int end_index = i - M2 + 1;
					float sum = 0;
					for (int j = end_index; j <= start_index; j++) {
						if (j >= 0 && j < MAX_POSSIBLE_SIZE) {
							sum += Y1[j];
						}
					}
					linear_speed_profile[i] = sum / M2 + agv_constraint.v_s; 
					real_curve_dist = real_curve_dist + linear_speed_profile[i] * 0.020; 
					ut[i] = real_curve_dist / (agv_constraint.Sn); 
					total_points_count += 1;
				}

				float real_time = 0; 
				float T_max = travel_time; 
			    for (int i = 0; i < total_points_count-1; i++)
		        {
				//float t = real_time / T_max; //normalized time 
				float u = ut[i]; 
				float u_next = ut[i + 1]; 
		        bezierCurve(u, point, controlPoints); 
				bezierCurve(u_next, next_point, controlPoints); 
		        if (i==0){
					float dx = next_point[0] - pose.x.f; 
					float dy = next_point[1] - pose.y.f; 
					//float first_heading = atan2((next_point[1] - pose.y.f), (next_point[0] - pose.x.f)); 
					float first_heading = atan(dy / dx); //Calculate the theta value
					dtheta2 = first_heading - starting_point.yaw; 
					last_angle = first_heading; 
				}
				else
				{   
					//float target_heading = atan2((next_point[1] - point[1]), (next_point[0] - point[0]));
					float dx = point[0] - next_point[0]; 
					float dy = point[1] - next_point[1];
				    float target_heading = atan(dy / dx);
					dtheta2 = target_heading - last_angle; //Delta in the angle change
					//dtheta2 = atan2((point[1] - last_point[1]), (point[0] - last_point[0]));
					last_angle = target_heading; 
				}
		        last_dtheta2 = dtheta2;
		        //central angular velocity omega
				ang_vel = dtheta2/sampling_time; 
		        //last_angle = dtheta2; //Saving the last theta value
		        //Robot velocity
				r_linvel = linear_speed_profile[i]; 
				linear_speed_profile[i] = 0;
		        //r_linvel2 = hypot(last_point[0]-point[0], last_point[0]-point[1]);
		        //Update the points information
		        last_point[0] = point[0];
		        last_point[1] = point[1];
				ut[i] = 0; 

		        // x_diff = pose.f_dis.f - point[0];
		        //double y_diff = 0.0 - point[1];
		        //calc_control_command(x_diff, y_diff, theta, 0.0, &rho,&v,&w);
		        //rhodebug[i] = rho;
#if DEBUG_MODE_ON
		        vdebug[i] = r_linvel;
		        //wdebug[i] = w;
		        xpointdebug[i] = point[0];
		        ypointdebug[i] = point[1];
		        rpmrightdebug[i]  = debugright_rpm;
		        rpmleftdebug[i]  = debugleft_rpm;
		        dthetadebug[i] = dtheta;
		        dthetadebug2[i] = dtheta2;
				Angveldebug[i] = ang_vel;
				Linveldebug[i] = r_linvel;
#endif 

		        if (fabs(r_linvel) > MAX_LINEAR_SPEED) {
		        	r_linvel = copysign(MAX_LINEAR_SPEED, r_linvel);
		        }
		        if (fabs(ang_vel) > MAX_ANGULAR_SPEED) {
		        	//ang_vel = copysign(MAX_ANGULAR_SPEED, ang_vel);
		        }
				Pose_Update();
				Velocity2Rpm(r_linvel,ang_vel);
				//Velocity2Rpm(0, ang_vel); 

				if (Send_Wheel_Command() != 1){
					cmd_sts = PC_CMDSTS_ERROR;
					printf("Wheel command sending error \n");
					return;
					}

				cmd_sts = PC_CMDSTS_INPROGRESS;
			    //theta = theta + w * t;
				real_time = real_time + DELTA_TIME; 
				//int time_interval = (int)(DELTA_TIME*1000);
				HAL_Delay(20);//Delay by the delta_time millisecs. (one delta_time)
		    }
            
			Pose_Update();
	        //Not to break purposely 
	case MOTION_DEC: //Post section cleanup of the states
			/*
#if ORIGINAL_DEC_PROFILE
#if LOG_DECK
	matrix_d_rpm[log_d_i] = motion_dec_speed_l;
	log_d_i++;
	matrix_d_rpm[log_d_i] = speed_l;
	log_d_i++;
	matrix_d_rpm[log_d_i] = left;
	log_d_i++;
	if(log_d_i >= 200){
		log_d_i = 0;
	}
#endif
#if 1
				if(x_final >= (pose.c_dis.f * MOTION_LOWER_THRESHOLD) && x_final < (pose.c_dis.f * MOTION_UPPER_THRESHOLD)){
#else
				if(Absolute(left - pose_start_enc_cnt) >= (Absolute(pose_2final_enc_cnt) * MOTION_LOWER_THRESHOLD) && Absolute(left - pose_start_enc_cnt) < (Absolute(pose_2final_enc_cnt) * _UPPER_THRESHOLD)){
#endif
					if(speed_l <= motion_speedl_dec_step *1.5){
						Pose_Speed_Init();
					    motion_exec_state = MOTION_IDLE;
						cmd_sts = PC_CMDSTS_COMPLETED;
						end_left = left;
						end_right = right;
					}else{
						motion_dec_speed_l = 0;
						motion_dec_speed_r = 0;
					}
#if 1
				}else if(x_final < (pose.c_dis.f * MOTION_LOWER_THRESHOLD)){
#else
				}else if (Absolute(left - pose_start_enc_cnt) < (Absolute(pose_2final_enc_cnt) * MOTION_LOWER_THRESHOLD)){
#endif
					if (motion_dec_speed_l > 0){
						if(Absolute(motion_dec_speed_l)  > motion_speedl_dec_step){
							motion_dec_speed_l -= motion_speedl_dec_step;
							if(Absolute(motion_dec_speed_l) < motion_speedl_dec_step){
								motion_dec_speed_l = motion_speedl_dec_step;
							}
						}
						else{
							motion_dec_speed_l = motion_speedl_dec_step;
						}
					}else{
						motion_dec_speed_l = MOTION_STEP_SPEED;
					}
					if (motion_dec_speed_r > 0){
						if(Absolute(motion_dec_speed_r)  > motion_speedr_dec_step)
						{
							motion_dec_speed_r -= motion_speedr_dec_step;
							if(Absolute(motion_dec_speed_r) < motion_speedr_dec_step)
							{
								motion_dec_speed_r = motion_speedr_dec_step;
							}
						}
						else{
							motion_dec_speed_r = motion_speedr_dec_step;
						}
					}else{
						motion_dec_speed_r = MOTION_STEP_SPEED;
					}
					cmd_sts = PC_CMDSTS_INPROGRESS;
				}else{
						Pose_Speed_Init();
						motion_exec_state = MOTION_IDLE;
						cmd_sts = PC_CMDSTS_COMPLETED;
						end_left = left;
						end_right = right;
				}

				if (Send_Wheel_Speed(motion_dec_speed_l, motion_dec_speed_r) != 1){
					printf("Wheel speed sending error \n");
					cmd_sts = PC_CMDSTS_ERROR;
					return;
				}
*/          
			if (agv_constraint.section_type == ONLY_ONE_METER || agv_constraint.section_type == DEC) //This section ends, AGV shall stop 
                {
	            Pose_Speed_Init();
				Velocity2Rpm(pose.velocity.linear_v.f, pose.velocity.angular_v.f); //Halt the AGV if the section is ONLY_ONE_METER/DEC 
				need_to_stop = 1; 
				cmd_sts = PC_CMDSTS_COMPLETED;
				motion_exec_state = MOTION_IDLE; //AGV stops and change state to MOTION_IDLE
                }
                else
                {
				cmd_sts = PC_CMDSTS_COMPLETED; 
				motion_exec_state = MOTION_START; 
                } 
			if (Send_Wheel_Speed(pose.velocity.linear_v.f, pose.velocity.angular_v.f) != 1) {
				printf("Wheel speed sending error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			} 
			end_left = left;
			end_right = right;
			Pose_Update();
			break;

	case MOTION_PAUSE:
			Calculate_Deceleration_Info();
			cmd_sts = PC_CMDSTS_INPROGRESS;
			motion_exec_state = MOTION_PAUSE_DEC;
			//break; //Note: No break purposely.
		case MOTION_PAUSE_DEC:
			if((time_count%2) == 0){
				if(Absolute(motion_dec_speed_l)  > motion_speedl_dec_step){
					motion_dec_speed_l -= motion_speedl_dec_step;
					if(Absolute(motion_dec_speed_l) < motion_speedl_dec_step){
						motion_dec_speed_l = 0;
						motion_dec_speed_r = 0;
					}
				}else{
					motion_dec_speed_l = 0;
					motion_dec_speed_r = 0;

				}
				cmd_sts = PC_CMDSTS_INPROGRESS;
				if (Send_Wheel_Speed(motion_dec_speed_l, motion_dec_speed_l) != 1){
					printf("Wheel speed sending error \n");
					cmd_sts = PC_CMDSTS_ERROR;
					return;
				}
			}
			Pose_Update();
			if(speed_l == 0){
				motion_exec_state = MOTION_PAUSE_END;
			}
			break;

		case MOTION_PAUSE_END:
			break;

		case MOTION_RESUME:
			if(bk_motion_exec_state == MOTION_DEC){
				motion_exec_state = MOTION_DEC;
			}else{
#if ENABLE_SIMPLE_RESUME
				if((pose.c_dis.f - x_final) > qr_interval/2){
					keep_straight = POSE_SPEED_NORMAL;	// Original speed, no correction
				}else{
					keep_straight = POSE_SPEED_SLOW; // SLOW mode
				}
				motion_exec_state = MOTION_START;
				elapse_time = 0;
				time_count = 0;
#else
				if((pose.c_dis.f - x_final) > qr_interval/2){
					pose.x.f = 0;
					pose.y.f = y_final;
					pose.theta.f = theta_final / ONEDEDREE_RADIAN;
					qr_interval = pose.c_dis.f - x_final;
					Local_Path_Planning();
				}else{
					pose.x.f = x_final;
					pose.y.f = y_final;
					pose.theta.f = theta_final / ONEDEDREE_RADIAN;
					motion_exec_state = MOTION_START;
					keep_straight = POSE_SPEED_SLOW;
				}
#endif
			}
			break;

		case MOTION_ABORTED:
			Pose_End_Process();
			motion_exec_state = MOTION_IDLE;
			break;

		default:
			debugLog("Wrong command\n");
	}
}
/************************************************************************************
FUNCTION		:Slowspeed_Update
DESCRIPTION		:Update slow speed in POSE mode.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/10/4
*************************************************************************************/
void Slowspeed_Update(){
	if(isSlowOn || (keep_straight == POSE_SPEED_SLOW)){
		Velocity2Rpm(motorpara.slow_linear_v.f, r_spd);
	}else{
#if BEZIER_CURVE_3POINTS
		Velocity2Rpm(pose.velocity.linear_v.f, r_spd); //Sending the RPM command to the
#else
		Velocity2Rpm(pose.velocity.linear_v.f, ang_vel);
#endif
		// Velocity2Rpm(dx, (pose.velocity.angular_v.f-r_offset*r_spd));
	}
}
/************************************************************************************
FUNCTION		:Calculate_Deceleration_Info
DESCRIPTION		:Calculate the start point of Deceleration to Final Distance.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/10/6
*************************************************************************************/
void Calculate_Deceleration_Info(){
#if POSE_STOP_OPTION
#if 1
	delta_dis_enc_count = pose_2final_enc_cnt - (left - pose_start_enc_cnt);
#else
	delta_dis_enc_count = delta_dis * (ticks_meter*ticks_offset);
#endif
	motion_dec_enc = left;
	motion_dec_speed_l = speed_l;
	motion_dec_speed_r = speed_r;
	motion_dec_speed = (motion_dec_speed_l < motion_dec_speed_r)?motion_dec_speed_l:motion_dec_speed_r;
	motion_dec_time = ((float)((motion_dec_speed_l > motion_dec_speed_r)?motion_dec_speed_l:motion_dec_speed_r)) / wheel_mdec / (DELTA_DEC_TIME);
	if(motion_dec_time > 0 ){
		motion_speedl_dec_step = (int16_t)( (float)motion_dec_speed_l / motion_dec_time * step_offset);
		motion_speedr_dec_step = (int16_t)( (float)motion_dec_speed_r / motion_dec_time * step_offset);
	}else{
		motion_speedl_dec_step = MOTION_STEP_SPEED;
		motion_speedr_dec_step = MOTION_STEP_SPEED;
	}
	#else
	x_spd = 0;
	r_spd = 0;
	velocity.linear_v.f = 0;
	velocity.angular_v.f = 0;
	Velocity2Rpm(velocity.linear_v.f,velocity.angular_v.f);			// Get speed data (m/s) from PC, need to calculate Power data or RPM data
	if (Send_Wheel_Command() != 1){
		printf("Wheel command sending error \n");
		return;
	}
	motion_exec_state = MOTION_IDLE;
#endif
}
/************************************************************************************
FUNCTION		:Calculate_Delta_Distance
DESCRIPTION		:Calculate the remaining distance to Final Destination.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/10/6
*************************************************************************************/
void Calculate_Delta_Distance(){
	float dec_speed_l = Rpm2Velocity(speed_l);
	float dec_speed_r = Rpm2Velocity(speed_r);
	float dec_speed = (dec_speed_l + dec_speed_r)/2;
	int16_t max_speed = ((speed_l > speed_r)?speed_l:speed_r);
	float decc_speed = dec_speed / ((float)(max_speed) / wheel_mdec);
	delta_dis = dec_speed * dec_speed /(2.0 * decc_speed);
}
/************************************************************************************
FUNCTION		:Calculate_Static_Delta_Distance
DESCRIPTION		:Calculate the remaining distance to Final Destination by velocity values in pose command
INPUT			:None
OUTPUT			:None
UPDATE			:2022/11/16
*************************************************************************************/
void Calculate_Static_Delta_Distance(){
	float dec_speed = pose.velocity.linear_v.f;
	float left_rpm = dec_speed / motorpara.wheel_circumference.f * M2SECS * motorpara.gear_ratio;
	float decc_speed = dec_speed / (left_rpm / wheel_mdec);
	static_delta_dis = dec_speed * dec_speed /(2.0 * decc_speed);
}
/************************************************************************************
FUNCTION		:Pose_End_Process
DESCRIPTION		:Abort current Pose command process.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/10/6
*************************************************************************************/
uint8_t Pose_End_Process(){
	Pose_Speed_Init();
	cmd_sts = PC_CMDSTS_ABORTED;
	if (uart_Motor_Stop() != 1){
		cmd_sts = PC_CMDSTS_ERROR;
		printf("Wheel command sending error \n");
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Pose_Speed_Init
DESCRIPTION		:pose velocity and deceleration speed set to 0.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/10/6
*************************************************************************************/
void Pose_Speed_Init(){
	motion_dec_speed_l = 0;
	motion_dec_speed_r = 0;
	pose.velocity.linear_v.f = 0;
	pose.velocity.angular_v.f = 0;
}
/************************************************************************************
FUNCTION		:Configure_Motor_SpeedPositionmode
DESCRIPTION		:Configure Speed Position Mode.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/8/25
*************************************************************************************/
uint8_t Configure_Motor_SpeedPositionmode(){
	if (encoder_speed_1 != 0 || encoder_speed_2 !=0){
		if (uart_Motor_Stop() != 1){
			return 0;
		}
	}
	uartRbSend(device_uart, "^MMOD 1 6_^MMOD 2 6\r");
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Configure_Motor_Separatemode
DESCRIPTION		:Configure Mixed Mode to Separate mode.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/8/25
*************************************************************************************/
uint8_t Configure_Motor_Separatemode(){
	uartRbSend(device_uart, "^MXMD 0\r");
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	mixed_mode = MIXEDMODE_MODE_SEPARATE;
	return 1;
}
#if MOTOR_MIXED_MODE
/************************************************************************************
FUNCTION		:Velocity2Throttle_Steering
DESCRIPTION		:Convert Velocity to Throttle RPM and Steering RPM
INPUT			:linear, angular
OUTPUT			:None
UPDATE			:2022/5/18
*************************************************************************************/
void Velocity2Throttle_Steering(float linear_v,float angular_v){
	float throttle_speed = linear_v;
	float steering_speed = motorpara.track_width.f * angular_v / 2.0;

	int16_t throttle_rpm = throttle_speed / motorpara.wheel_circumference.f * M2SECS * motorpara.gear_ratio;
	int16_t steering_rpm = steering_speed / motorpara.wheel_circumference.f * M2SECS * motorpara.gear_ratio;

	strcpy(str_roboteq, "!S 1 ");
	Prepare_Channel_Data(throttle_rpm);
	strcat (str_roboteq, "_!S 2 ");
	Prepare_Channel_Data(steering_rpm);
	strcat (str_roboteq, "\r");
}
/************************************************************************************
FUNCTION		:Configure_Motor_Mixedmode
DESCRIPTION		:Configure Mixed Mode.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/8/25
*************************************************************************************/
uint8_t Configure_Motor_Mixedmode(){
	if (encoder_speed_1 != 0 || encoder_speed_2 !=0){
		if (uart_Motor_Stop() != 1){
			return 0;
		}
	}
	uartRbSend(device_uart, "^MXMD 1\r");
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	mixed_mode = MIXEDMODE_MODE_1;
	return 1;
}

#endif
/************************************************************************************
FUNCTION		:Send_Wheel_Command
DESCRIPTION		:Send wheel Command
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/8/25
*************************************************************************************/
uint8_t Send_Wheel_Command(){
	uartRbSend(device_uart, &str_roboteq[0]);
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Motor_Rotation
DESCRIPTION		:Based on the rotation angle and sync info from PC, send commands to Motor and Deck controller.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/6/16
*************************************************************************************/
uint8_t Motor_Rotation()
{
#if COUNT_POSITION_MODE
	if(isPositionmode == 0){
		if (Configure_Motor_Positionmode() == 0){
			return 0;
		}
		isSpeedmode = 0;
		isPositionmode = 1;
	}
	Angle2EncCount(rotation.angle.f);
	uartRbSend(device_uart, &str_roboteq[0]);
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		debugLog("Motor 1 speed fails\n\n");
		return 0;
	}
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		debugLog("Motor 2 speed fails\n\n");
		return 0;
	}

//	uartRbSend(device_uart, "^RWD 00\r");
//	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
	return 1;
#else
	if(lifter_exec_state != LIFTERMOVE_IDLE || home_exec_state != HOME_IDLE){ // Lifter UP/DONW and Home are exclusive to Wheel rotation
		return 0;
	}
	if (isDeckOption == 1 && rotation.deck_sync == 1){
		deck_rotation_angle.f = rotation.angle.f;
		if (Deck_Rotation() != 1){
			body_exec_state = BODYROTATION_IDLE;
			cmd_sts = PC_CMDSTS_ERROR;
			return 0;
		}
	}
#if TIMER_ENCODER
	Reset_Encoder_Count();
	body_exec_state = BODYROTATION_INIT;
	cmd_sts = PC_CMDSTS_INPROGRESS;
	printf("Wheel rotation starts \n");
	return 1;
#else
	if (Configure_Motor_Speedmode() != 1){
		deck_exec_state = DECKROTATION_IDLE;
		return 0;
	}
	body_exec_state = BODYROTATION_INIT;
	printf("Wheel rotation starts \n");
	return 1;
#endif

#endif
}

/************************************************************************************
FUNCTION		:Body_Action
DESCRIPTION		:State machine for actions monitor and execution.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/07/08
*************************************************************************************/
void Body_Action()
{
	if (body_exec_state == BODYROTATION_IDLE){
	  return;
	}
#if TIMER_ENCODER
	body_enc_count = left;
	if(body_exec_state > BODYROTATION_INIT && Absolute(body_enc_count) > Absolute(distance_enc_count)*BODYROTATION_FORCE_STOP){
		body_set_speed = 0;
		body_exec_state = BODYROTATION_IDLE;
		cmd_sts = PC_CMDSTS_ABORTED;
		if (Send_Wheel_Speed(body_set_speed, -body_set_speed) != 1){
			printf("Wheel speed sending error \n");
			cmd_sts = PC_CMDSTS_ERROR;
		}
	}
#else
	if(Read_Wheel1_Enc_Count() != 1){
	  return;
	}
	body_enc_count = motor_read_buf;
#endif
	switch (body_exec_state)
	{
		case BODYROTATION_IDLE:
			break;

		case BODYROTATION_INIT:
			if (body_enc_count < BODYROTATION_STOP_JUDGE){
				body_exec_state = BODYROTATION_INC;
				float angle = T_angle(rotation.angle.f);
				distance_enc_count = (PI * motorpara.track_width.f * ticks_meter * angle / FULL_ANGLE);
				if(distance_enc_count > 0 ){
					body_p_m = 1;
				}else{
					body_p_m = -1;
				}
				distance_enc_count -=(BODY_DISTANCE_OFFSET*distance_enc_count);
				body_mid_enc = (float)Absolute(distance_enc_count)/(1.0 + BODYROTATION_DIS_BIAS);
				body_set_speed = 0;
				peak_enc_count = 0;
#if LOG_WHEEL
				log_i = 0;
				memset(matrix_rpm,0,sizeof(matrix_rpm));
				memset(matrix_dis,0,sizeof(matrix_dis));
#endif
			}else{
				body_exec_state = BODYROTATION_IDLE;
				cmd_sts = PC_CMDSTS_ERROR;
			}
			break;
#if ENABLE_WHEEL_R_RESUME
		case BODYROTATION_PAUSE:
			body_set_speed = 0;
			cmd_sts = PC_CMDSTS_INPROGRESS;
			if (Send_Wheel_Speed(body_set_speed, -body_set_speed) != 1){
				printf("Wheel speed sending error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			break;

		case BODYROTATION_RESUME:
			rotation.angle.f = (float)(distance_enc_count - body_enc_count) * FULL_ANGLE / (PI * motorpara.track_width.f * ticks_meter);
			Motor_Rotation();
			break;

		case BODYROTATION_ABORTED:
			body_set_speed = 0;
			if (Send_Wheel_Speed(body_set_speed, -body_set_speed) != 1){
				printf("Wheel speed sending error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			body_exec_state = BODYROTATION_IDLE;
			cmd_sts = PC_CMDSTS_ABORTED;
			break;
#endif
		case BODYROTATION_INC:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			if(Absolute(body_enc_count) < body_mid_enc){
				body_set_speed += (body_p_m * BODY_INC_STEP);
				if(Absolute(body_set_speed) >= BODYROTATION_PEAK_RPM){
					peak_enc_count = body_enc_count; // encoder count when reach peak
					body_set_speed = (body_p_m * BODYROTATION_PEAK_RPM);
					body_exec_state = BODYROTATION_KEEP;
				}
			}else{
				peak_enc_count = body_enc_count; // encoder count when reach peak
				body_set_speed -= (body_p_m * BODY_DEC_STEP);
				body_exec_state = BODYROTATION_DEC;
			}
			if (Send_Wheel_Speed(body_set_speed, -body_set_speed) != 1){
				printf("Wheel speed sending error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
#if LOG_WHEEL
			matrix_rpm[log_i] = body_set_speed;
			matrix_dis[log_i] = body_enc_count;
			log_i++;
#endif
			break;

		case BODYROTATION_KEEP:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			if(Absolute(body_enc_count) < (Absolute(distance_enc_count) - Absolute(peak_enc_count)*BODYROTATION_DIS_BIAS)){
				// maintain current speed: +/-BODYROTATION_PEAK_RPM
			}else{
				body_set_speed -= (body_p_m * BODY_DEC_STEP);
				body_exec_state = BODYROTATION_DEC;
			}
			if (Send_Wheel_Speed(body_set_speed, -body_set_speed) != 1){
				printf("Wheel speed sending error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
#if LOG_WHEEL
			matrix_rpm[log_i] = body_set_speed;
			matrix_dis[log_i] = body_enc_count;
			log_i++;
#endif
			break;

		case BODYROTATION_DEC:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			if(Absolute(body_enc_count) > (Absolute(distance_enc_count) * BODYROTATION_LOWER_THRESHOLD) && Absolute(body_enc_count) < (Absolute(distance_enc_count) * BODYROTATION_UPPER_THRESHOLD)){
				if(Absolute(body_set_speed) <= (BODY_DEC_STEP * BODY_SPEED_RANGE)){
					body_set_speed = 0;
					body_exec_state = BODYROTATION_IDLE;
					cmd_sts = PC_CMDSTS_COMPLETED;
				}else{
#if 0
					body_set_speed = body_set_speed/2;
#else
					if ((body_set_speed * body_p_m) > 0){
						if(Absolute(body_set_speed)  > (BODY_DEC_STEP)){
							body_set_speed -= (body_p_m * BODY_DEC_STEP);
							if(Absolute(body_set_speed) < BODY_DEC_STEP){
								body_set_speed = (body_p_m * BODY_DEC_STEP);
							}
						}else{
							body_set_speed = (body_p_m * BODY_DEC_STEP);
						}
					}else{
						body_set_speed += (body_p_m * BODY_INC_STEP);
					}
#endif
					body_exec_state = BODYROTATION_FINETUNE;
				}

			}else if (Absolute(body_enc_count) < (Absolute(distance_enc_count) * BODYROTATION_LOWER_THRESHOLD)){
				if ((body_set_speed * body_p_m) > 0){
					if(Absolute(body_set_speed)  > (BODY_DEC_STEP)){
						body_set_speed -= (body_p_m * BODY_DEC_STEP);
						if(Absolute(body_set_speed) < BODY_DEC_STEP){
							body_set_speed = (body_p_m * BODY_DEC_STEP);
						}
					}else{
						body_set_speed = (body_p_m * BODY_DEC_STEP);
					}
				}else{
					body_set_speed += (body_p_m * BODY_INC_STEP);
				}
			}else{
				if(Absolute(body_set_speed) < (BODY_DEC_STEP * BODY_SPEED_RANGE)){
					body_set_speed = 0;
				}else{
#if 0
					body_set_speed = body_set_speed/2;
#else
					if ((body_set_speed * body_p_m) > 0){
						if(Absolute(body_set_speed)  > (BODY_DEC_STEP)){
							body_set_speed -= (body_p_m * BODY_DEC_STEP);
							if(Absolute(body_set_speed) < BODY_DEC_STEP){
								body_set_speed = (body_p_m * BODY_DEC_STEP);
							}
						}else{
							body_set_speed = (body_p_m * BODY_DEC_STEP);
						}
					}else{
						body_set_speed += (body_p_m * BODY_INC_STEP);
					}
#endif
				}
				body_exec_state = BODYROTATION_FINETUNE;
			}
			if (Send_Wheel_Speed(body_set_speed, -body_set_speed) != 1){
				printf("Wheel speed sending error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
#if LOG_WHEEL
			matrix_rpm[log_i] = body_set_speed;
			matrix_dis[log_i] = body_enc_count;
			log_i++;
#endif
			break;

      case BODYROTATION_FINETUNE:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			if((Absolute(body_enc_count) > (Absolute(distance_enc_count) * BODYROTATION_LOWER_THRESHOLD) && Absolute(body_enc_count) < (Absolute(distance_enc_count) * BODYROTATION_UPPER_THRESHOLD))){
				if(Absolute(body_set_speed) < (BODY_DEC_STEP * BODY_SPEED_RANGE)){
					body_set_speed = 0;
					body_exec_state = BODYROTATION_IDLE;
					cmd_sts = PC_CMDSTS_COMPLETED;
				}else{
					if ((body_set_speed * body_p_m) > 0){
						body_set_speed -=(body_p_m * BODY_FINE_STEP);
					}else{
						body_set_speed =-(body_p_m * BODY_FINE_STEP);
					}
				}
			}else if (Absolute(body_enc_count) < (Absolute(distance_enc_count) * BODYROTATION_LOWER_THRESHOLD)){
				body_set_speed = (body_p_m * BODY_FINE_STEP);
			}else{
				body_set_speed =-(body_p_m * BODY_FINE_STEP);
			}
			if (Send_Wheel_Speed(body_set_speed, -body_set_speed) != 1){
				printf("Wheel speed sending error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
#if LOG_WHEEL
			matrix_rpm[log_i] = body_set_speed;
			matrix_dis[log_i] = body_enc_count;
			log_i++;
#endif
			break;

        default:
			debugLog("Wrong command\n");
	}
}
/************************************************************************************
FUNCTION		:Absolute
DESCRIPTION		:Get absolute value of integer.
INPUT			:integer
OUTPUT			:None
UPDATE			:2022/7/7
*************************************************************************************/
int32_t Absolute(int32_t value) {
  if (value < 0)
    return (-value);
  return value;
}
/************************************************************************************
FUNCTION		:Absolute_f
DESCRIPTION		:Get absolute value of float type.
INPUT			:float
OUTPUT			:absolute integer value
UPDATE			:2022/7/7
*************************************************************************************/
float Absolute_f(float value) {
  if (value < 0)
    return (-value);
  return value;
}
/************************************************************************************
FUNCTION		:Send_Stream_Query_Off
DESCRIPTION		:Send stream query off to Roboteq controller
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/8/2
*************************************************************************************/
uint8_t Send_Stream_Query_Off(){
	uartRbSend(device_uart, "# c\r");
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}

/************************************************************************************
FUNCTION		:Send_Wheel_RPM
DESCRIPTION		:Send wheel speed in RPM
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/7/8
*************************************************************************************/
uint8_t Send_Wheel_Speed(int16_t left_rpm, int16_t right_rpm){
	strcpy(str_roboteq, "!S 1 ");
	Prepare_Channel_Data(left_rpm);
	strcat (str_roboteq, "_!S 2 ");
	Prepare_Channel_Data(right_rpm);
	strcat (str_roboteq, "\r");
	if (Send_Wheel_Command() != 1){
		printf("Wheel command sending error \n");
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Deck_Home
DESCRIPTION		:Return Deck to Home position
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/6/16
*************************************************************************************/
uint8_t Deck_Home(){
	uartRbSend(deck_uart, "^MMOD 2 0\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		cmd_sts = PC_CMDSTS_ERROR;
		return 0;
	}
	uartRbSend(deck_uart, "!CB 2 0\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		cmd_sts = PC_CMDSTS_ERROR;
		return 0;
	}

	uartRbSend(deck_uart, "^MMOD 2 1\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		cmd_sts = PC_CMDSTS_ERROR;
		return 0;
	}
//	uartRbSend(deck_uart, "^KP 2 2\r");
//	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		cmd_sts = PC_CMDSTS_ERROR;
//		return 0;
//	}
//	uartRbSend(deck_uart, "^KI 2 1\r");
//	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		cmd_sts = PC_CMDSTS_ERROR;
//		return 0;
//	}
//	uartRbSend(deck_uart, "^KD 2 0\r");
//	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		cmd_sts = PC_CMDSTS_ERROR;
//		return 0;
//	}
	home_exec_state = HOME_INC;
	cmd_sts = PC_CMDSTS_INPROGRESS;
	return 1;
}
/************************************************************************************
FUNCTION		:Home_Action
DESCRIPTION		:State machine for actions monitor and execution.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/07/01
*************************************************************************************/
void Home_Action()
{
	switch (home_exec_state)
	{
		case HOME_IDLE:
			break;
#if ENABLE_HOME_RESUME
		case HOME_PAUSE:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			deck_set_speed = 0;
			if (Send_Deck_Speed() != 1){
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			break;

		case HOME_RESUME:
			Deck_Home();
			break;

		case HOME_ABORTED:
			deck_set_speed = 0;
			if (Send_Deck_Speed() != 1){
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			cmd_sts = PC_CMDSTS_ABORTED;
			home_exec_state = HOME_IDLE;
			break;
#endif
		case HOME_INC:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			if(Read_Deck_Hall_Count() != 1){
				printf("Deck hall count reading error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			if (hall_count_buffer > HOME_ONEROUND){
				cmd_sts = PC_CMDSTS_ABORTED;
				home_exec_state = HOME_IDLE;
				deck_set_speed = 0;
				current_deck_pos.f = 0.0;
				if (Send_Deck_Speed() != 1){
					cmd_sts = PC_CMDSTS_ERROR;
					return;
				}
				return;
			}
			if(HAL_GPIO_ReadPin(HOME_GPIO_Port,HOME_Pin) == GPIO_PIN_SET){
				deck_set_speed = HOME_INIT_RPM;
				if (Send_Deck_Speed() != 1){
					cmd_sts = PC_CMDSTS_ERROR;
					return;
				}
			}else{
				home_exec_state = HOME_DEC;
				deck_set_speed = HOME_END_RPM;
				if (Send_Deck_Speed() != 1){
					cmd_sts = PC_CMDSTS_ERROR;
					return;
				}
			}
			break;

		case HOME_DEC:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			if(HAL_GPIO_ReadPin(HOME_GPIO_Port,HOME_Pin) == GPIO_PIN_RESET){
				deck_set_speed = HOME_END_RPM;
				if (Send_Deck_Speed() != 1){
					cmd_sts = PC_CMDSTS_ERROR;
					return;
				}
			}else{
				cmd_sts = PC_CMDSTS_COMPLETED;
				home_exec_state = HOME_IDLE;
				deck_set_speed = 0;
				current_deck_pos.f = 0.0;
				if (Send_Deck_Speed() != 1){
					cmd_sts = PC_CMDSTS_ERROR;
					return;
				}
			}
			break;

        default:
			debugLog("Wrong command\n");
	}
}
/************************************************************************************
FUNCTION		:Deck_Rotation
DESCRIPTION		:Based on the rotation angle from PC, send commands to Deck controller.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/6/16
*************************************************************************************/
uint8_t Deck_Rotation(){
#if COUNT_POSITION_MODE
	float angle;
	uint16_t CBcount_Per_Rev;
	if (Configure_Deck_Positionmode() == 1){
		angle = T_angle(deck_rotation_angle.f);
		motorpara.deck_gear_ratio=50;
		motorpara.deck_cpr.i32=24;
		CBcount_Per_Rev =(73.0/22.0)*(float)motorpara.deck_gear_ratio*(float)motorpara.deck_cpr.i32 + 62; // 62 is compensation value
		int16_t hall_count =((float)CBcount_Per_Rev / FULL_ANGLE) * angle;

		strcpy(str_roboteq, "!PR 2 ");
		Prepare_Channel_Data(hall_count);
		strcat (str_roboteq, "\r");
		uartRbSend(deck_uart, &str_roboteq[0]);
		if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		return 1;
	}
	return 0;
#else
//	uartRbSend(deck_uart, "!AC 1 50000\r");	//Acceleration value is in 0.1*RPM per second.
//	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
//	uartRbSend(deck_uart, "!DC 1 50000\r");	//Deceleration value is in 0.1*RPM per second
//	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
//	uartRbSend(deck_uart, "^MXRPM 1 30000\r");	//Deceleration value is in 0.1*RPM per second
//	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
//	uartRbSend(deck_uart, "^MVEL 1 9000\r");	//Deceleration value is in 0.1*RPM per second
//	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
	if(lifter_exec_state != LIFTERMOVE_IDLE){  // Deck Rotation is exclusive to Lifter Up/down
		cmd_sts = PC_CMDSTS_ERROR;
		return 0;
	}
	uartRbSend(deck_uart, "^MMOD 2 0\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		cmd_sts = PC_CMDSTS_ERROR;
		return 0;
	}
	uartRbSend(deck_uart, "!CB 2 0\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		cmd_sts = PC_CMDSTS_ERROR;
		return 0;
	}

	uartRbSend(deck_uart, "^MMOD 2 1\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		cmd_sts = PC_CMDSTS_ERROR;
		return 0;
	}
	printf("Deck rotation starts \n");
	deck_exec_state = DECKROTATION_INIT;
	cmd_sts = PC_CMDSTS_INPROGRESS;
	return 1;
#endif
}
/************************************************************************************
FUNCTION		:Deck_Action
DESCRIPTION		:State machine for actions monitor and execution.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/07/01
*************************************************************************************/
void Deck_Action()
{
	static uint16_t CBcount_per_rev;
	if(deck_exec_state == DECKROTATION_IDLE){
		return;
	}
	if(Read_Deck_Hall_Count() != 1){
		printf("Deck hall count reading error \n");
		cmd_sts = PC_CMDSTS_ERROR;
		return;
	}
	deck_hall_count = hall_count_buffer;
	if(deck_exec_state > DECKROTATION_INIT && Absolute(deck_hall_count) > Absolute(distance_hall_count)*DECKROTATION_FORCE_STOP){
		deck_set_speed = DECK_FINE_STEP;
		if (Send_Deck_Speed() != 1){
			printf("Deck speed sending error \n");
			cmd_sts = PC_CMDSTS_ERROR;
		}
		deck_exec_state = DECKROTATION_IDLE;
		cmd_sts = PC_CMDSTS_ABORTED;
		deck_set_speed = 0;
		if (Send_Deck_Speed() != 1){
			printf("Deck speed sending error \n");
			cmd_sts = PC_CMDSTS_ERROR;
		}
	}
	switch (deck_exec_state)
	{
		case DECKROTATION_IDLE:
			break;

		case DECKROTATION_INIT:
			if (deck_hall_count < DECKROTATION_STOP_JUDGE){
				deck_exec_state = DECKROTATION_INC;
				float angle = T_angle(deck_rotation_angle.f);
				CBcount_per_rev =(73.0/22.0) * (float)motorpara.deck_gear_ratio * (float)motorpara.deck_cpr.i32 + 60; // 59 is compensation value
				distance_hall_count =((float)CBcount_per_rev / FULL_ANGLE) * angle;
				if(distance_hall_count > 0){
					deck_p_m = 1;
				}else{
					deck_p_m = -1;
				}
				deck_mid_hall = (float)Absolute(distance_hall_count)/(1.0 + DECKROTATION_DIS_BIAS);
				deck_set_speed = 0;
				peak_hall_count = 0;
#if LOG_DECK
				log_d_i = 0;
				memset(matrix_d_rpm,0,sizeof(matrix_d_rpm));
//				memset(matrix_d_dis,0,sizeof(matrix_d_dis));
#endif
			}else{
				deck_exec_state = DECKROTATION_IDLE;
				cmd_sts = PC_CMDSTS_ERROR;
			}
			break;
#if ENABLE_DECK_RESUME
		case DECKROTATION_PAUSE:
			deck_set_speed = 0;
			if (Send_Deck_Speed() != 1){
				printf("Deck speed sending error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			cmd_sts = PC_CMDSTS_INPROGRESS;
			break;

		case DECKROTATION_RESUME:
			deck_rotation_angle.f = (float)(distance_hall_count - deck_hall_count) * FULL_ANGLE / (float)CBcount_per_rev;
			if (isDeckOption == 1 && rotation.deck_sync == 0 && body_exec_state == BODYROTATION_IDLE){
				Deck_Rotation();
			}
			break;

		case DECKROTATION_ABORTED:
			deck_set_speed = 0;
			if (Send_Deck_Speed() != 1){
				printf("Deck speed sending error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			cmd_sts = PC_CMDSTS_ABORTED;
			deck_exec_state = DECKROTATION_IDLE;
			break;
#endif
		case DECKROTATION_INC:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			if (Absolute(deck_hall_count) < deck_mid_hall){
				deck_set_speed += (deck_p_m * DECK_INC_STEP);
				if(Absolute(deck_set_speed) >= DECKROTATION_PEAK_RPM){
					peak_hall_count = deck_hall_count;	// hall counter when reach peak
					deck_set_speed = (deck_p_m * DECKROTATION_PEAK_RPM);
					deck_exec_state = DECKROTATION_KEEP;
				}
			}else{
				peak_hall_count = deck_hall_count;	// hall counter when reach peak
				deck_set_speed -= (deck_p_m * DECK_DEC_STEP);
				deck_exec_state = DECKROTATION_DEC;
			}
			if (Send_Deck_Speed() != 1){
				printf("Deck speed sending error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
#if LOG_DECK
			matrix_d_rpm[log_d_i] = deck_set_speed;
//			matrix_d_dis[log_d_i] = deck_hall_count;
			log_d_i++;
#endif
			break;

		case DECKROTATION_KEEP:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			if(Absolute(deck_hall_count) < (Absolute(distance_hall_count) - Absolute(peak_hall_count)*DECKROTATION_DIS_BIAS)){
				// maintain current speed: +/-DECKROTATION_PEAK_RPM
			}else{
				deck_set_speed -= (deck_p_m * DECK_DEC_STEP);
				deck_exec_state = DECKROTATION_DEC;
			}
			if (Send_Deck_Speed() != 1){
				printf("Deck speed sending error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
#if LOG_DECK
			matrix_d_rpm[log_d_i] = deck_set_speed;
//			matrix_d_dis[log_d_i] = deck_hall_count;
			log_d_i++;
#endif
			break;

		case DECKROTATION_DEC:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			if(Absolute(deck_hall_count) > (Absolute(distance_hall_count) * DECKROTATION_LOWER_THRESHOLD) && Absolute(deck_hall_count) < (Absolute(distance_hall_count) * DECKROTATION_UPPER_THRESHOLD)){
				if (Absolute(deck_set_speed) <= (DECK_DEC_STEP * DECK_SPEED_RANGE)){
					deck_set_speed = 0;
					deck_exec_state = DECKROTATION_POSITION;
				}else{
#if 0
					deck_set_speed = deck_set_speed/2;
#else
					if ((deck_set_speed * deck_p_m) > 0){
						if (Absolute(deck_set_speed)  > (DECK_DEC_STEP)){
							deck_set_speed -= (deck_p_m * DECK_DEC_STEP);
							if (Absolute(deck_set_speed) < DECK_DEC_STEP){
								deck_set_speed = (deck_p_m * DECK_DEC_STEP);
							}
						}else{
							deck_set_speed = (deck_p_m * DECK_DEC_STEP);
						}
					}else{
						deck_set_speed += (deck_p_m * DECK_DEC_STEP);
					}
#endif

					deck_exec_state = DECKROTATION_FINETUNE;
				}
			}else if (Absolute(deck_hall_count) < (Absolute(distance_hall_count) * DECKROTATION_LOWER_THRESHOLD)){
				if ((deck_set_speed * deck_p_m) > 0){
					if (Absolute(deck_set_speed)  > (DECK_DEC_STEP)){
						deck_set_speed -= (deck_p_m * DECK_DEC_STEP);
						if (Absolute(deck_set_speed) < DECK_DEC_STEP){
							deck_set_speed = (deck_p_m * DECK_DEC_STEP);
						}
					}else{
						deck_set_speed = (deck_p_m * DECK_DEC_STEP);
					}
				}else{
					deck_set_speed += (deck_p_m * DECK_DEC_STEP);
				}
			}else{
				if (Absolute(deck_set_speed) < (DECK_DEC_STEP * DECK_SPEED_RANGE)){
					deck_set_speed = 0;
				}else{
#if 0
					deck_set_speed = deck_set_speed/2;
#else
					if ((deck_set_speed * deck_p_m) > 0){
						if (Absolute(deck_set_speed)  > (DECK_DEC_STEP)){
							deck_set_speed -= (deck_p_m * DECK_DEC_STEP);
							if (Absolute(deck_set_speed) < DECK_DEC_STEP){
								deck_set_speed = (deck_p_m * DECK_DEC_STEP);
							}
						}else{
							deck_set_speed = (deck_p_m * DECK_DEC_STEP);
						}
					}else{
						deck_set_speed += (deck_p_m * DECK_DEC_STEP);
					}
#endif
				}
				deck_exec_state = DECKROTATION_FINETUNE;
			}
			if (Send_Deck_Speed() != 1){
				printf("Deck speed sending error \n");
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			if(deck_set_speed == 0){
				uartRbSend(deck_uart, "!MS 2\r");
				if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
				{
					cmd_sts = PC_CMDSTS_ERROR;
					return;
				}
			}
#if LOG_DECK
			matrix_d_rpm[log_d_i] = deck_set_speed;
	//		matrix_d_dis[log_d_i] = deck_hall_count;
			log_d_i++;
#endif
			break;

      case DECKROTATION_FINETUNE:
  		cmd_sts = PC_CMDSTS_INPROGRESS;
        if((Absolute(deck_hall_count) > (Absolute(distance_hall_count) * DECKROTATION_LOWER_THRESHOLD) && Absolute(deck_hall_count) < (Absolute(distance_hall_count) * DECKROTATION_UPPER_THRESHOLD))){
			if (Absolute(deck_set_speed) <= (DECK_DEC_STEP * DECK_SPEED_RANGE)){
	        	deck_set_speed = 0;
	        	deck_exec_state = DECKROTATION_POSITION;
			}else{
				if ((deck_set_speed * deck_p_m) > 0){
					deck_set_speed = (deck_p_m * DECK_FINE_STEP);
				}else{
					deck_set_speed = -(deck_p_m * DECK_FINE_STEP);
				}
			}
        }else if (Absolute(deck_hall_count) < (Absolute(distance_hall_count) * DECKROTATION_LOWER_THRESHOLD)){
        	deck_set_speed = (deck_p_m * DECK_FINE_STEP);
        }else{
			deck_set_speed = -(deck_p_m * DECK_FINE_STEP);
        }
        if (Send_Deck_Speed() != 1){
			printf("Deck speed sending error \n");
			cmd_sts = PC_CMDSTS_ERROR;
        	return;
        }
		if(deck_set_speed == 0 && deck_exec_state == DECKROTATION_POSITION){
			uartRbSend(deck_uart, "!EX\r");
			if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
			{
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
		}
#if LOG_DECK
		matrix_d_rpm[log_d_i] = deck_set_speed;
//		matrix_d_dis[log_d_i] = deck_hall_count;
		log_d_i++;
#endif
        break;

      case DECKROTATION_POSITION:
  		cmd_sts = PC_CMDSTS_INPROGRESS;
		uartRbSend(deck_uart, "!MG\r");
		if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			cmd_sts = PC_CMDSTS_ERROR;
			return;
		}
		if(Read_Deck_Hall_Speed() != 1){
			printf("Deck hall speed reading error \n");
			cmd_sts = PC_CMDSTS_ERROR;
			return;
		}
		if(hall_speed != 0){
			return;
		}
		current_deck_pos.f += (float)(deck_hall_count * FULL_ANGLE) / CBcount_per_rev;
		current_deck_pos.f = T_angle(current_deck_pos.f);
		deck_exec_state = DECKROTATION_IDLE;
		cmd_sts = PC_CMDSTS_COMPLETED;
    	break;

        default:
			debugLog("Wrong command\n");
	}
}
/************************************************************************************
FUNCTION		:Send_Deck_Speed
DESCRIPTION		:Send deck speed in RPM
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/7/7
*************************************************************************************/
uint8_t Send_Deck_Speed(){
	strcpy(str_roboteq, "!S 2 ");
	Prepare_Channel_Data(deck_set_speed);
	strcat (str_roboteq, "\r");
	uartRbSend(deck_uart, &str_roboteq[0]);
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Send_Lifter_Speed
DESCRIPTION		:Send Lifter speed in RPM
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/7/14
*************************************************************************************/
uint8_t Send_Lifter_Speed(){
	strcpy(str_roboteq, "!G 1 ");
	Prepare_Channel_Data(lifter_set_speed);
	strcat (str_roboteq, "\r");
	uartRbSend(deck_uart, &str_roboteq[0]);
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Read_Deck_Hall_Count
DESCRIPTION		:Read Deck hall count
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/7/7
*************************************************************************************/
uint8_t Read_Deck_Hall_Count(){
	uartRbSend(deck_uart, "?CB 2\r");
	if(isDeckReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Read_Deck_Hall_Speed
DESCRIPTION		:Read Deck hall speed
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/8/8
*************************************************************************************/
uint8_t Read_Deck_Hall_Speed(){
	uartRbSend(deck_uart, "?BS 2\r");
	if(isDeckReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Read_Lifter_Upper_Input
DESCRIPTION		:Read Lifter upper input level
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/7/7
*************************************************************************************/
uint8_t Read_Lifter_Upper_Input(){
	uartRbSend(deck_uart, "?DI 2\r");
	if(isDeckReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Read_Lifter_Lower_Input
DESCRIPTION		:Read Lifter lower input level
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/7/7
*************************************************************************************/
uint8_t Read_Lifter_Lower_Input(){
	uartRbSend(deck_uart, "?DI 1\r");
	if(isDeckReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Read_Lifter_Hall_Count
DESCRIPTION		:Read Lifter hall count
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/7/7
*************************************************************************************/
uint8_t Read_Lifter_Hall_Count(){
	uartRbSend(deck_uart, "?CB 1\r");
	if(isDeckReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Read_Lifter_Status_Flag
DESCRIPTION		:Read Lifter FM (status flag)
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/7/7
*************************************************************************************/
uint8_t Read_Lifter_Status_Flag(){
	uartRbSend(deck_uart, "?FM 1\r");
	if(isDeckReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
#if !TIMER_ENCODER
/************************************************************************************
FUNCTION		:Read_Wheel_Enc_Count
DESCRIPTION		:Read Wheel encoder count
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/7/7
*************************************************************************************/
uint8_t Read_Wheel_Enc_Count(){
	uartRbSend(device_uart, "?C 1\r");
	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	encoder_count_1 = motor_read_buf;
	uartRbSend(device_uart, "?C 2\r");
	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	encoder_count_2 = motor_read_buf;
	return 1;
}
/************************************************************************************
FUNCTION		:Read_Wheel1_Enc_Count
DESCRIPTION		:Read Wheel encoder count
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/7/7
*************************************************************************************/
uint8_t Read_Wheel1_Enc_Count(){
	uartRbSend(device_uart, "?C 2\r");
	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	encoder_count_2 = motor_read_buf;
	return 1;
}
/************************************************************************************
FUNCTION		:Read_Wheel_Enc_Speed
DESCRIPTION		:Read Wheel encoder speed
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/7/20
*************************************************************************************/
uint8_t Read_Wheel_Enc_Speed(){
	uartRbSend(device_uart, "?S 1\r");
	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	encoder_speed_1 = motor_read_buf;
	uartRbSend(device_uart, "?S 2\r");
	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	encoder_speed_2 = motor_read_buf;
	return 1;
}
#endif
/************************************************************************************
FUNCTION		:Read_Wheel_Fault_Flag
DESCRIPTION		:Read Wheel Fault Flag
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/7/20
*************************************************************************************/
uint8_t Read_Wheel_Fault_Flag(){
	uartRbSend(device_uart, "?FF\r");
	if(isWheelReadConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Lifter_Updown
DESCRIPTION		:Move Lifter to upper limit or lower limit.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/6/16
*************************************************************************************/
uint8_t Lifter_Updown(){
#if ROS_DEBUG
	status_onoff = lifter_updn;
#else
	if((deck_exec_state != DECKROTATION_IDLE) || (body_exec_state != BODYROTATION_IDLE)){ // Deck Rotation & Wheel rotation are exclusive to Lifter Up/down
		cmd_sts = PC_CMDSTS_ERROR;
		return 0;
	}
	if(lifter_updn == LIFTER_UP){
		if(Read_Lifter_Upper_Input() != 1){
			cmd_sts = PC_CMDSTS_ERROR;
			return 0;
		}
		if(lifter_sensor != 0){
			cmd_sts = PC_CMDSTS_ERROR;
			return 0;
		}
	}else{
		if(Read_Lifter_Lower_Input() != 1){
			cmd_sts = PC_CMDSTS_ERROR;
			return 0;
		}
		if(lifter_sensor != 0){
			cmd_sts = PC_CMDSTS_ERROR;
			return 0;
		}
	}
	uartRbSend(deck_uart, "^MMOD 1 0\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		cmd_sts = PC_CMDSTS_ERROR;
		return 0;
	}
	uartRbSend(deck_uart, "!CB 1 0\r");
	if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		cmd_sts = PC_CMDSTS_ERROR;
		return 0;
	}
	cmd_sts = PC_CMDSTS_INPROGRESS;
	lifter_exec_state = LIFTERMOVE_INIT;
#endif
	return 1;
}

/************************************************************************************
FUNCTION		:Lifter_Action
DESCRIPTION		:State machine for actions monitor and execution.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/07/14
*************************************************************************************/
void Lifter_Action()
{
	if(lifter_exec_state == LIFTERMOVE_IDLE){
		return;
	}
	if(Read_Lifter_Hall_Count() != 1){
		cmd_sts = PC_CMDSTS_ERROR;
		return;
	}
	lifter_hall_count = hall_count_buffer;

	switch (lifter_exec_state)
	{
		case LIFTERMOVE_IDLE:
			break;

		case LIFTERMOVE_INIT:
			if(lifter_hall_count < LIFTER_STOP_JUDGE){
				if(lifter_updn == LIFTER_UP){
					lifter_exec_state = LIFTERMOVE_UP;
				}else{
					lifter_exec_state = LIFTERMOVE_DOWN;
				}
			}else{
				lifter_exec_state = LIFTERMOVE_IDLE;
				cmd_sts = PC_CMDSTS_ERROR;
			}
			break;
#if ENABLE_LIFTER_RESUME
		case LIFTERMOVE_PAUSE:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			lifter_set_speed = 0;
			if (Send_Lifter_Speed() != 1){
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			break;

		case LIFTERMOVE_RESUME:
			Lifter_Updown();
			break;

		case LIFTERMOVE_ABORTED:
			lifter_set_speed = 0;
			if (Send_Lifter_Speed() != 1){
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			lifter_exec_state = LIFTERMOVE_IDLE;
			cmd_sts = PC_CMDSTS_ABORTED;
			break;
#endif
		case LIFTERMOVE_UP:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			lifter_set_speed = LIFTER_UP_SPEED;
			if (Send_Lifter_Speed() != 1){
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			lifter_exec_state = LIFTERMOVE_FM_STATUS;
			break;

		case LIFTERMOVE_DOWN:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			lifter_set_speed = LIFTER_DN_SPEED;
			if (Send_Lifter_Speed() != 1){
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			lifter_exec_state = LIFTERMOVE_FM_STATUS;
			break;

		case LIFTERMOVE_FM_STATUS:
			cmd_sts = PC_CMDSTS_INPROGRESS;
			if(Read_Lifter_Status_Flag() != 1){
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			if(lifter_updn == LIFTER_UP){
				if((lifter_status&0x08) == 0x08 || (lifter_status&0x10) == 0x10){
					lifter_exec_state = LIFTERMOVE_STOP;
					current_lifter_dir = LIFTER_UP;
				}
			}else{
				if((lifter_status&0x08) == 0x08 || (lifter_status&0x20) == 0x20){
					lifter_exec_state = LIFTERMOVE_STOP;
					current_lifter_dir = LIFTER_DN;
				}
			}
			break;

		case LIFTERMOVE_STOP:
			lifter_set_speed = 0;
			if (Send_Lifter_Speed() != 1){
				cmd_sts = PC_CMDSTS_ERROR;
				return;
			}
			lifter_exec_state = LIFTERMOVE_IDLE;
			cmd_sts = PC_CMDSTS_COMPLETED;
			break;

        default:
			debugLog("Wrong command\n");
	}
}
/************************************************************************************
FUNCTION		:Configure_Motor_Speedmode
DESCRIPTION		:Configure Speed Mode.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/6/20
*************************************************************************************/
uint8_t Configure_Motor_Speedmode(){
//	uartRbSend(device_uart, "^RWD 1000\r");
//	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
	if (encoder_speed_1 != 0 || encoder_speed_2 !=0){
		if (uart_Motor_Stop() != 1){
			return 0;
		}
	}
	uartRbSend(device_uart, "^MMOD 1 0_^MMOD 2 0\r");
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}

	uartRbSend(device_uart, "!C 1 0_!C 2 0\r");
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}

	uartRbSend(device_uart, "^MMOD 1 1_^MMOD 2 1\r");
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
	{
		return 0;
	}
//	uartRbSend(device_uart, "!AC 1 2000_!AC 2 2000\r");	//Acceleration value is in 0.1*RPM per second.
//	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
//	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
//	uartRbSend(device_uart, "!DC 1 2000_!DC 2 2000\r");	//Deceleration value is in 0.1*RPM per second
//	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
//	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
//	uartRbSend(device_uart, "^KPG 1 600000_^KPG 2 600000\r");
//	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
//	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
//	uartRbSend(device_uart, "^KIG 1 2000000_^KIG 2 2000000\r");
//	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
//	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
//	uartRbSend(device_uart, "^KDG 1 0_^KDG 2 0\r");
//	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
//	if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//	{
//		return 0;
//	}
	return 1;
}

/************************************************************************************
FUNCTION		:Configure_Motor_Positionmode
DESCRIPTION		:Configure Position Mode.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/6/20
*************************************************************************************/
uint8_t Configure_Motor_Positionmode(){
	if (uart_Motor_Stop() == 1){
//		uartRbSend(device_uart, "^RWD 1000\r");
//		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//		{
//			return 0;
//		}

		uartRbSend(device_uart, "^MMOD 1 0\r");
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(device_uart, "^MMOD 2 0\r");
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(device_uart, "!C 1 0\r");
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(device_uart, "!C 2 0\r");
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
//		uartRbSend(device_uart, "^KPG 3 2000000\r");
//		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//		{
//			return 0;
//		}
//		uartRbSend(device_uart, "^KIG 3 1000000\r");
//		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//		{
//			return 0;
//		}
//		uartRbSend(device_uart, "^KDG 3 0\r");
//		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//		{
//			return 0;
//		}
//		uartRbSend(device_uart, "^KPG 4 2000000\r");
//		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//		{
//			return 0;
//		}
//		uartRbSend(device_uart, "^KIG 4 1000000\r");
//		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//		{
//			return 0;
//		}
//		uartRbSend(device_uart, "^KDG 4 0\r");
//		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
//		{
//			return 0;
//		}
		uartRbSend(device_uart, "^MMOD 1 3\r");
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(device_uart, "^MMOD 2 3\r");
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(device_uart, "!AC 1 20000\r");	//Acceleration value is in 0.1*RPM per second.
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(device_uart, "!AC 2 20000\r");
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(device_uart, "!DC 1 20000\r");	//Deceleration value is in 0.1*RPM per second
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(device_uart, "!DC 2 20000\r");
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(device_uart, "^MVEL 1 1000\r");
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(device_uart, "^MVEL 2 1000\r");
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(device_uart, "!S 1 1000\r");
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(device_uart, "!S 2 1000\r");
		if(isCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Configure_Deck_Positionmode
DESCRIPTION		:Configure Position Mode.
INPUT			:None
OUTPUT			:Success: 1; Fail: 0
UPDATE			:2022/07/01
*************************************************************************************/
uint8_t Configure_Deck_Positionmode(){
	if (uart_Deck_Stop() == 1){
		uartRbSend(deck_uart, "^MMOD 2 0\r");
		if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(deck_uart, "!CB 2 0\r");
		if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(deck_uart, "^MMOD 2 3\r");
		if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(deck_uart, "^KP 2 20\r");
		if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(deck_uart, "^KI 2 10\r");
		if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(deck_uart, "^KD 2 0\r");
		if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}

		uartRbSend(deck_uart, "!AC 2 50000\r");	//Acceleration value is in 0.1*RPM per second.
		if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(deck_uart, "!DC 2 50000\r");	//Deceleration value is in 0.1*RPM per second
		if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
		uartRbSend(deck_uart, "!S 2 3000\r");
		if(isDeckCmdConfirmed(MOTOR_TIMEOUT) != 1)
		{
			return 0;
		}
	}
	return 1;
}
/************************************************************************************
FUNCTION		:Angle2EncCount
DESCRIPTION		:Convert Angle to Encoder Count
INPUT			:Rotation Angle from PC
OUTPUT			:None
UPDATE			:2022/6/20
*************************************************************************************/
void Angle2EncCount(float angle){
	angle = T_angle(angle);
	float body_circumference = PI * motorpara.track_width.f;
//	float distance_per_enccount = motorpara.wheel_circumference.f / (float)motorpara.cpr.i32;
	int16_t enc_count = body_circumference * (float)motorpara.cpr.i32 * (float)motorpara.gear_ratio * angle / FULL_ANGLE / motorpara.wheel_circumference.f;

	if(angle > 0){
		strcpy(str_roboteq, "!PR 1 ");
		Prepare_Channel_Data(enc_count);
		strcat (str_roboteq, "_!PR 2 ");
		Prepare_Channel_Data(-enc_count);
		strcat (str_roboteq, "\r");
	}else{
		strcpy(str_roboteq, "!PR 1 ");
		Prepare_Channel_Data(-enc_count);
		strcat (str_roboteq, "_!PR 2 ");
		Prepare_Channel_Data(enc_count);
		strcat (str_roboteq, "\r");
	}
}
/************************************************************************************
FUNCTION		:T_angle
DESCRIPTION		:Transform Angle to 0~180, 0~-180
INPUT			:Rotation Angle from PC
OUTPUT			:angle
UPDATE			:2022/6/20
*************************************************************************************/
float T_angle(float a){
    a = fmod(a, FULL_ANGLE);
    if(a > STRAIGHT_ANGLE)a = a - FULL_ANGLE;
    else if(a < - STRAIGHT_ANGLE)a = a + FULL_ANGLE;
    return a;
}
/************************************************************************************
FUNCTION		:Current_Cmd_Update
DESCRIPTION		:Process commands from PC..
INPUT			:None
OUTPUT			:None
UPDATE			:2022/9/22
*************************************************************************************/
void Current_Cmd_Update(){
    switch(current_cmdid)
    {
    	case PC_CMDID_NONE:
    		current_cmdid = PcData[CmdIDIdx];
			break;

        case PC_CMDID_VELOCITY:
    		current_cmdid = PcData[CmdIDIdx];
			break;

        case PC_CMDID_POSE:
        	if(motion_exec_state == 0){
    			current_cmdid = PcData[CmdIDIdx];
        	}else{
        		if(PcData[CmdIDIdx] == PC_CMDID_POSE){
        			current_cmdid = PcData[CmdIDIdx];
        		}else{
        			isPCDataOK = FALSE;
        		}
        	}
            break;

        case PC_CMDID_ROTATION:
        	if(body_exec_state == 0){
    			current_cmdid = PcData[CmdIDIdx];
        	}else{
        		isPCDataOK = FALSE;
        	}
            break;

        case PC_CMDID_DECK_HOME:
        	if(home_exec_state == 0){
    			current_cmdid = PcData[CmdIDIdx];
        	}else{
        		isPCDataOK = FALSE;
        	}
            break;

        case PC_CMDID_DECK_ROTATION:
        	if(deck_exec_state == 0){
    			current_cmdid = PcData[CmdIDIdx];
        	}else{
        		isPCDataOK = FALSE;
        	}
            break;

        case PC_CMDID_DECK_UPDOWN:
        	if(lifter_exec_state == 0){
    			current_cmdid = PcData[CmdIDIdx];
        	}else{
        		isPCDataOK = FALSE;
        	}
            break;

        default:
			current_cmdid = PcData[CmdIDIdx];
			debugLog("Wrong command\n");
    }
}

/************************************************************************************
FUNCTION		:Action_Cmd_Process
DESCRIPTION		:Process action commands from PC.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/9/22
*************************************************************************************/
void Action_Cmd_Process(){
    switch(current_cmdid)
    {
        case PC_CMDID_VELOCITY:
			break;

        case PC_CMDID_POSE:
        	if(motion_exec_state != 0){
        		if(action_sts == PC_ACTIONSTS_PAUSE){
					bk_motion_exec_state = motion_exec_state;
        			motion_exec_state = MOTION_PAUSE;
        		}else if(action_sts == PC_ACTIONSTS_RESUME){
        			motion_exec_state = MOTION_RESUME;
        		}else if(action_sts == PC_ACTIONSTS_ABORTED){
        			motion_exec_state = MOTION_ABORTED;
        		}
        	}
            break;

        case PC_CMDID_ROTATION:
#if ENABLE_WHEEL_R_RESUME
        	if(body_exec_state != 0){
        		if(action_sts == PC_ACTIONSTS_PAUSE){
        			body_exec_state = BODYROTATION_PAUSE;
        			if (isDeckOption == 1 && rotation.deck_sync == 1){
            			deck_exec_state = DECKROTATION_PAUSE;
        			}
        		}else if(action_sts == PC_ACTIONSTS_RESUME){
        			body_exec_state = BODYROTATION_RESUME;
        			if (isDeckOption == 1 && rotation.deck_sync == 1){
            			deck_exec_state = DECKROTATION_RESUME;
        			}
        		}else if(action_sts == PC_ACTIONSTS_ABORTED){
        			body_exec_state = BODYROTATION_ABORTED;
        			if (isDeckOption == 1 && rotation.deck_sync == 1){
            			deck_exec_state = DECKROTATION_ABORTED;
        			}
        		}
        	}
#endif
            break;

        case PC_CMDID_DECK_HOME:
#if ENABLE_HOME_RESUME
        	if(home_exec_state != 0){
        		if(action_sts == PC_ACTIONSTS_PAUSE){
        			home_exec_state = HOME_PAUSE;
        		}else if(action_sts == PC_ACTIONSTS_RESUME){
        			home_exec_state = HOME_RESUME;
        		}else if(action_sts == PC_ACTIONSTS_ABORTED){
        			home_exec_state = HOME_ABORTED;
        		}
        	}
#endif
            break;

        case PC_CMDID_DECK_ROTATION:
#if ENABLE_DECK_RESUME
        	if(deck_exec_state != 0){
        		if(action_sts == PC_ACTIONSTS_PAUSE){
        			deck_exec_state = DECKROTATION_PAUSE;
        		}else if(action_sts == PC_ACTIONSTS_RESUME){
        			deck_exec_state = DECKROTATION_RESUME;
        		}else if(action_sts == PC_ACTIONSTS_ABORTED){
        			deck_exec_state = DECKROTATION_ABORTED;
        		}
        	}
#endif
            break;

        case PC_CMDID_DECK_UPDOWN:
#if ENABLE_LIFTER_RESUME
        	if(lifter_exec_state != 0){
        		if(action_sts == PC_ACTIONSTS_PAUSE){
        			lifter_exec_state = LIFTERMOVE_PAUSE;
        		}else if(action_sts == PC_ACTIONSTS_RESUME){
        			lifter_exec_state = LIFTERMOVE_RESUME;
        		}else if(action_sts == PC_ACTIONSTS_ABORTED){
        			lifter_exec_state = LIFTERMOVE_ABORTED;
        		}
        	}
#endif
            break;

        default:
			debugLog("Wrong command\n");
    }
}

/************************************************************************************
FUNCTION		:Status_Cmd_Process
DESCRIPTION		:Process status commands from PC.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/9/27
*************************************************************************************/
void Status_Cmd_Process(){
    switch(status_cmdid)
    {
    case PC_STATUS_CMDID_QUERY:
		debugLog("Receiving query from PC succeeds\n");
		Read_Para_Query();
        break;

#if ENABLE_PC_STATUS_REQ
    case PC_STATUS_CMDID_STATUS:
		debugLog("Receiving staus request from PC succeeds\n");
        break;
#endif

#if !MCU_INITREQ_ENABLE
    case PC_STATUS_CMDID_INITPARA:
    	memcpy(&motorpara.wheel_circumference.bytes[0],&PcData[DataIdx],sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio));
    	motorpara.cpr.bytes[3] = PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)];
    	motorpara.cpr.bytes[2] = PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)+1];
    	motorpara.cpr.bytes[1] = PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)+2];
    	motorpara.cpr.bytes[0] = PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)+3];
//        	memcpy(&motorpara.cpr.bytes[0],&PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)],sizeof(motorpara.cpr.i32));
    	memcpy(&motorpara.refresh_rate,&PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)+sizeof(motorpara.cpr.i32)],sizeof(motorpara.refresh_rate)+sizeof(motorpara.deck_gear_ratio));
    	motorpara.deck_cpr.bytes[3] = PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)+sizeof(motorpara.cpr.i32)+sizeof(motorpara.refresh_rate)+sizeof(motorpara.deck_gear_ratio)];
    	motorpara.deck_cpr.bytes[2] = PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)+sizeof(motorpara.cpr.i32)+sizeof(motorpara.refresh_rate)+sizeof(motorpara.deck_gear_ratio)+1];
    	motorpara.deck_cpr.bytes[1] = PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)+sizeof(motorpara.cpr.i32)+sizeof(motorpara.refresh_rate)+sizeof(motorpara.deck_gear_ratio)+2];
    	motorpara.deck_cpr.bytes[0] = PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)+sizeof(motorpara.cpr.i32)+sizeof(motorpara.refresh_rate)+sizeof(motorpara.deck_gear_ratio)+3];
//        	memcpy(&motorpara.deck_cpr.bytes[0],&PcData[DataIdx+sizeof(motorpara.wheel_circumference.f)+sizeof(motorpara.track_width.f)+sizeof(motorpara.upper_linear_v.f)+sizeof(motorpara.slow_linear_v.f)+sizeof(motorpara.gear_ratio)+sizeof(motorpara.cpr.i32)+sizeof(motorpara.deck_gear_ratio)],sizeof(motorpara.deck_cpr.i32));
    	ticks_meter = (1 / motorpara.wheel_circumference.f) * motorpara.cpr.i32 * motorpara.gear_ratio;
    	Reset_Encoder_Count();
    	debugLog("PC sending init parameters succeeds\n\n");
        break;
#endif

        default:
			debugLog("Wrong command\n");
    }
}
/************************************************************************************
FUNCTION		:Reset_Encoder_Count
DESCRIPTION		:Reset Encoder Count when necessary.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/11/04
*************************************************************************************/
void Reset_Encoder_Count(){
	if (speed_l != 0 || speed_r !=0){
		uart_Motor_Stop();
	}
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	prev_lencoder = 0;
	prev_rencoder = 0;
	lmult = 0;
	rmult = 0;
	left = 1;
	right = 1;
}
