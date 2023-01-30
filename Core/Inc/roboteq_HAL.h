/************************************************************************************
FILE			:roboteq_HAL.h
DESCRIPTION		:
*************************************************************************************/

#ifndef roboteq_HAL_H_
#define roboteq_HAL_H_

typedef uint8_t	BOOL;		/* Boolean type 	*/

#ifndef	FALSE
#define	FALSE			(0)
#endif	/* #ifndef	FALSE	*/

#ifndef	TRUE
#define	TRUE			(1)
#endif	/* #ifndef	TRUE	*/

/************************************
	Flag type definition
************************************/
typedef struct {
	uint8_t		bit0:1;
	uint8_t		bit1:1;
	uint8_t		bit2:1;
	uint8_t		bit3:1;
	uint8_t		bit4:1;
	uint8_t		bit5:1;
	uint8_t		bit6:1;
	uint8_t		bit7:1;
} FLAG;
typedef union
{
	float f;
	uint8_t bytes[4];
}float2int;
typedef union
{
	uint32_t i32;
	uint8_t bytes[4];
}int32int8;

#define FIRM_VERSION_CORE_MAIN (1u)
#define FIRM_VERSION_CORE_MINOR (0u)
#define FIRM_VERSION_CORE_PATCH (0u)

#define LOG_DECK	1
#define LOG_WHEEL	0
#define COUNT_POSITION_MODE 0
#define TIMER_ENCODER 1
#define MOTOR_MIXED_MODE 0
#define SPEED_POSITION_MODE 1
#define ROS_DEBUG 0
#define ENABLE_LIFTER_INIT 1
#define POSE_STOP_OPTION 1	//1: use encoder count; 0: use remaining time
#define MCU_INITREQ_ENABLE 0	//0: Disable MCU INITREQ to PC; 1: enable MCU INITREQ to PC
#define PC_CHECKSUM_ENABLE 0	//0: Disable PC checksum checking for testing purpose; 1: enable PC checksum
#define ENABLE_PC_STATUS_REQ 0	//1: Enable PC STATUS Request; 0: Disable PC STATUS Request
#define ENABLE_SIMPLE_RESUME 1	//1: Simple resume of POSE command 0: Complicated Resume processing
#define ENABLE_HOME_RESUME	0	//1: Enable Pause / resume function for Home; 0: Disable Pause / resume function for Home
#define ENABLE_DECK_RESUME	0	//1: Enable Pause / resume function for Deck Rotation; 0: Disable Pause / resume function for Deck Rotation
#define ENABLE_WHEEL_R_RESUME	0	//1: Enable Pause / resume function for Wheel Rotation; 0: Disable Pause / resume function for Wheel Rotation
#define ENABLE_LIFTER_RESUME	0	//1: Enable Pause / resume function for Lifter; 0: Disable Pause / resume function for Lifter
#define ENABLE_POSE_AUTO_4M	0	//1: Enable Pause / resume function for Lifter; 0: Disable Pause / resume function for Lifter
#define BEZIER_CURVE_3POINTS	1
#define FINAL_DISTANCE_COMP	0

#define PcSyncCode_SIZE 2
#define PcCmdType_SIZE 1
#define PcDataLength_SIZE 1
#define PcCmdID_SIZE 1
#define PcData_SIZE 128
#define PcCheckSum_SIZE 1
#define PcTrail_SIZE 1
#define CmdTypeIdx 0
#define DataLengthIdx 1
#define CmdIDIdx 2
#define DataIdx 3
#define CmdPacketlength (PcCmdType_SIZE+PcDataLength_SIZE+PcCmdID_SIZE+PcData_SIZE+PcCheckSum_SIZE+PcTrail_SIZE)
#define MOTOR_QUERYDATA_LENGTH 20
#define MOTOR_QUERYPARA_LENGTH 5
#define DECK_QUERYDATA_LENGTH 5
#define MOTOR_ACK_LENGTH 1
#define MOTOR_INITREQ_LENGTH 1
#define PC_RETRY_TIMES 3
#define PC_TIMEOUT 300
#define ROBOTEQ_RETRY_TIMES	3
#define MOTOR_TIMEOUT 300
#define MAX_ARRAY_SIZE 24
#define MAX_MOTOR_CMD_SIZE 50

#define	TYPE_DATA		0xC3
#define	TYPE_ACTION		0xA3
#define	TYPE_STATUS		0x83
#define	TYPE_ACK		0x3C
#define	TYPE_ACTION_ACK	0x3A
#define	TYPE_STATUS_ACK	0x38

#define	PC_CMDID_NONE			0x00
#define	PC_CMDID_VELOCITY		0x01
#define	PC_CMDID_POSE			0x02
#define	PC_CMDID_ROTATION		0x03

#define	PC_CMDID_DECK_HOME		0x10
#define	PC_CMDID_DECK_ROTATION	0x11
#define	PC_CMDID_DECK_UPDOWN	0x12

#define	PC_STATUS_CMDID_INITPARA		0x01
#define	PC_STATUS_CMDID_QUERY			0x02
#if ENABLE_PC_STATUS_REQ
#define	PC_STATUS_CMDID_STATUS			0x03
#endif
#if MCU_INITREQ_ENABLE
#define	MCU_CMDID_INITREQ		0x81
#endif
#define	MCU_CMDID_QUERY			0x82
#define	MCU_CMDID_STATUS		0x83

#define INITPARA_W_C			4

#define PC_CMDSTS_IDLE			0x00
#define PC_CMDSTS_INPROGRESS	0x01
#define PC_CMDSTS_COMPLETED		0x02
#define PC_CMDSTS_ERROR			0x03
#define PC_CMDSTS_ABORTED		0x04

#define PC_ACTIONSTS_NORMAL		0x00
#define PC_ACTIONSTS_PAUSE		0x01
#define PC_ACTIONSTS_RESUME		0x02
#define PC_ACTIONSTS_ABORTED	0x03

#define	PARA_QUERY_AMPS			0x00
#define	PARA_QUERY_BATTERY_AMPS	0x01
#define	PARA_QUERY_POWER		0x02

#define WHEEL_CIRCUMFERENCE 0.49
#define TRACK_WIDTH 0.5759
#define UPPER_LINEAR_V	1.0
#define SLOW_LINEAR_V	0.25
#define WHEEL_GEAR_RATIO 21
#define WHEEL_ENC_PPR 200
#define WHEEL_ENC_QUADRATURE 4
#define WHEEL_ENC_CPR (WHEEL_ENC_PPR * WHEEL_ENC_QUADRATURE)
#define DECK_GEAR_RATIO 24
#define DECK_ENC_CPR 50
#define REFRESH_RATE 4

#define	isDataAvailable	fUART_roboteq.bit0
#define	isSetCmdOK	fUART_roboteq.bit1
#define	isQueryOK 	fUART_roboteq.bit2
#define	isPresent	fUART_roboteq.bit3
#define	isFailInit	fUART_roboteq.bit4
#define	isSpeedmode	fUART_roboteq.bit5
#define	isPositionmode	fUART_roboteq.bit6
#define	isWheelReadOK	fUART_roboteq.bit7

#define	isPCDataOK	fUART_pc.bit0
#define	isPCActionOK	fUART_pc.bit1
#define	isPCStatusOK	fUART_pc.bit2
#define	isPCAckOK	fUART_pc.bit3
#define	isPCPresent	fUART_pc.bit4
#define	isPCFailInit	fUART_pc.bit5
#define	is1stPosepass	fUART_pc.bit6

#define	isDeckSetCmdOK	fUART_deck.bit0
#define	isDeckQueryOK 	fUART_deck.bit1
#define	isDeckPresent	fUART_deck.bit2
#define	isDeckFailInit	fUART_deck.bit3
#define	isDeckOption	fUART_deck.bit4
#define	isDeckReadOK	fUART_deck.bit5

#define	isLifterReadOK		fUART_lifter.bit0

#define	isStopOn	fINPUT_status.bit0
#define	isSlowOn	fINPUT_status.bit1
#define	isEstopOn	fINPUT_status.bit2

#define	f_Overheat		fault_flag.bit0
#define	f_Overvoltage	fault_flag.bit1
#define	f_Undervoltage	fault_flag.bit2
#define	f_Shortciruit	fault_flag.bit3
#define	f_Emergencystop	fault_flag.bit4
#define	f_Setupfault	fault_flag.bit5
#define	f_MOSFETfailure	fault_flag.bit6
#define	f_Defaultconfig	fault_flag.bit7

#define PI 3.1415926535

#define HOME_INIT_RPM	842
#define HOME_END_RPM	(-280)
#define HOME_IDLE	0
#define HOME_INC	1
#define HOME_DEC	2
#define HOME_PAUSE	3
#define HOME_RESUME	4
#define HOME_ABORTED 5

#define HOME_ONEROUND 4240


#define DECKROTATION_PEAK_RPM	1100
#define DECK_INC_STEP (55)
#define DECK_DEC_STEP (55)
#define DECK_FINE_STEP (110)

#define DECKROTATION_IDLE 0
#define DECKROTATION_INIT 1
#define DECKROTATION_INC 2
#define DECKROTATION_KEEP 3
#define DECKROTATION_DEC 4
#define DECKROTATION_FINETUNE 5
#define DECKROTATION_POSITION 6
#define DECKROTATION_PAUSE 7
#define DECKROTATION_RESUME 8
#define DECKROTATION_ABORTED 9


#define DECKROTATION_DIS_BIAS 1.15
#define DECKROTATION_LOWER_THRESHOLD 0.995
#define DECKROTATION_UPPER_THRESHOLD 1.005
#define DECKROTATION_DOWN_COUNT 5
#define DECKROTATION_STOP_JUDGE 10
#define DECK_SPEED_RANGE 2
#define DECKROTATION_FORCE_STOP (1.2)

#define BODYROTATION_PEAK_RPM	485
#define BODY_INC_STEP 25
#define BODY_DEC_STEP 25
#define BODY_FINE_STEP 42

#define BODYROTATION_IDLE 0
#define BODYROTATION_INIT 1
#define BODYROTATION_INC 2
#define BODYROTATION_KEEP 3
#define BODYROTATION_DEC 4
#define BODYROTATION_FINETUNE 5
#define BODYROTATION_PAUSE 6
#define BODYROTATION_RESUME 7
#define BODYROTATION_ABORTED 8

#define BODYROTATION_DIS_BIAS 1.15
#define BODYROTATION_LOWER_THRESHOLD 0.995
#define BODYROTATION_UPPER_THRESHOLD 1.005
#define BODYROTATION_DOWN_COUNT 5
#define BODYROTATION_STOP_JUDGE 20
#define BODY_SPEED_RANGE 3
#define BODY_DISTANCE_OFFSET 0.0455
#define BODYROTATION_FORCE_STOP (1.2)



#define LIFTERMOVE_IDLE 0
#define LIFTERMOVE_INIT 1
#define LIFTERMOVE_UP 2
#define LIFTERMOVE_DOWN 3
#define LIFTERMOVE_FM_STATUS 4
#define LIFTERMOVE_STOP 5
#define LIFTERMOVE_PAUSE 6
#define LIFTERMOVE_RESUME 7
#define LIFTERMOVE_ABORTED 8


#define LIFTER_UP_SPEED 500
#define LIFTER_DN_SPEED (-200)
#define LIFTER_STOP_JUDGE 5
#define LIFTER_UP 1
#define LIFTER_DN 0
#define f_amps_limit	f_lifter_status.bit0
#define f_motor_installed	f_lifter_status.bit1
#define f_loop_error	f_lifter_status.bit2
#define f_safety_stop	f_lifter_status.bit3
#define f_fwd_limit		f_lifter_status.bit4
#define f_rwd_limit		f_lifter_status.bit5
#define f_amps_trig		f_lifter_status.bit6
#define MOTION_IDLE 0
#define MOTION_INIT 1
#define MOTION_START 2
#define MOTION_DEC 3
#define MOTION_PAUSE 4
#define MOTION_PAUSE_DEC 5
#define MOTION_PAUSE_END 6
#define MOTION_RESUME 7
#define MOTION_ABORTED 8

#define MOTION_FINETUNE 7
#define MOTION_LOWER_THRESHOLD 0.998
#define MOTION_UPPER_THRESHOLD 1.01

#define MOTION_STEP_SPEED 50

#define MIXEDMODE_MODE_SEPARATE 0
#define MIXEDMODE_MODE_1 1
#define MIXEDMODE_MODE_2 1

#define QR_INTERVAL_2 (2)	// 2 meters
#define QR_INTERVAL_1 (1)	// 1 meters
#define QR_SKIP (0)			// QR skip
#define TIME_INTERVAL2 (2.1) // 2m/(1m/s)=2 seconds + 4*DELTA_TIME
#define TIME_INTERVAL1 (1.1) // 2m/(1m/s)=2 seconds + 4*DELTA_TIME
#define DELTA_TIME (0.025)
#define DELTA_DEC_TIME (0.05)
#define ACCEL_TICKS (5)
#define ONEDEDREE_RADIAN (0.01745329252)
#define M2SECS (60.0)
#define FULL_ANGLE (360.0)
#define STRAIGHT_ANGLE (180.0)
#define RIGHT_ANGLE (90.0)
#define CURVE_SECTION_NUM (12.0)
#define CURVE_P1_X (1.5)
#define CURVE_P1_Y (1.5)
#define CURVE_P2_X (2.3)
#define CURVE_P2_Y (2.0)
#define Y_ZERO_THRESHOLD (0.004)	//5mm
#define THETA_ZERO_THRESHOLD (0.3)	//0.3 degree
#define TABLE1_THETA_THRESHOLD (1.3)
#define TABLE2_THETA_THRESHOLD (2.3)
#define POSE_SPEED_CORRECTION 0
#define POSE_SPEED_SLOW 1
#define POSE_SPEED_NORMAL 2

#define Y_BIAS (0.01)
#define THETA_BIAS (2.0)

#define VELCMD_TIMEOUT (30)

#endif /* roboteq_HAL_H_ */
