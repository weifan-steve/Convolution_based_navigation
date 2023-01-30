/************************************************************************************
FILE			:uartRingBufDMA.c
DESCRIPTION		:Process DMA Idle event
*************************************************************************************/
/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "uartRingBufDMA.h"
#include "string.h"
#include "stdio.h"
#include "roboteq_HAL.h"

/* Private variables ---------------------------------------------------------*/
char RxBuf[RxBuf_SIZE];
char MainBuf[MainBuf_SIZE];
uint16_t oldPos = 0;
uint16_t newPos = 0;
uint16_t Head, Tail;
char PcData[CmdPacketlength];
/* Timeout is in milliseconds */
int32_t TIMEOUT = 0;

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern char * roboteq_array[MAX_ARRAY_SIZE];
extern int32_t encoder_count_1;		// ?C
extern int32_t encoder_count_2;		// ?C
extern int32_t encoder_speed_1;		// ?S
extern int32_t encoder_speed_2;		// ?S
extern uint16_t fault_flag;			// ?FF
extern char * deck_array[MAX_ARRAY_SIZE];
extern FLAG fUART_roboteq;
extern FLAG fUART_deck;
extern FLAG fUART_lifter;
extern FLAG fUART_pc;
extern uint8_t lifter_status;
extern uint8_t lifter_sensor;
extern int32_t hall_count_buffer;
extern int32_t motor_read_buf;
extern int32_t hall_speed;
extern uint8_t current_cmdid;
extern uint8_t cmd_sts;
extern uint8_t action_sts;
#if ENABLE_PC_STATUS_REQ
extern uint8_t motor_deck_status_bytes;
#endif
extern uint8_t para_query;
extern uint8_t status_cmdid;

/* Private function prototypes -----------------------------------------------*/
void Ringbuf_Init (void);
void Ringbuf_Reset (void);
uint8_t checkString (char *str, char *buffertolookinto);
uint8_t waitFor (char *string, uint32_t Timeout);
uint8_t copyUpto (char *string, char *buffertocopyinto, uint32_t Timeout);
uint8_t getAfter (char *string, uint8_t numberofchars, char *buffertocopyinto, uint32_t Timeout);

extern uint8_t uart_Ack2PC(uint8_t cmdtype, uint8_t ack);
extern void Split_Str(char *cur_array, const char * chr, char **new_array);
extern uint8_t uart_ucGetCheckSum(uint8_t *msg, uint8_t length);
extern void Current_Cmd_Update();

/************************************************************************************
FUNCTION		:Ringbuf_Init
DESCRIPTION		:Initialize the Ring Buffer.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void Ringbuf_Init (void)
{
	memset(RxBuf, '\0', RxBuf_SIZE);
	memset(MainBuf, '\0', MainBuf_SIZE);

	Head = Tail = 0;
	oldPos = 0;
	newPos = 0;
}

/************************************************************************************
FUNCTION		:Ringbuf_Reset
DESCRIPTION		:Resets the Ring buffer.
INPUT			:None
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void Ringbuf_Reset (void)
{
	memset(MainBuf,'\0', MainBuf_SIZE);
	memset(RxBuf, '\0', RxBuf_SIZE);
	Tail = 0;
	Head = 0;
	oldPos = 0;
	newPos = 0;
	isSetCmdOK = FALSE;
	isQueryOK = FALSE;
	isPresent = FALSE;
	isPCDataOK = FALSE;
	isPCAckOK = FALSE;
	isPCPresent = FALSE;
}

/************************************************************************************
FUNCTION		:checkString
DESCRIPTION		:checks whether the entered string is present in the given buffer
INPUT			:str: entered string; buffertolookinto: given buffer
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t checkString (char *str, char *buffertolookinto)
{
	int stringlength = strlen (str);
	int bufferlength = strlen (buffertolookinto);
	int so_far = 0;
	int indx = 0;
repeat:
	while (str[so_far] != buffertolookinto[indx])
	{
		indx++;
		if (indx>bufferlength) return 0;
	}

	if (str[so_far] == buffertolookinto[indx])
	{
		while (str[so_far] == buffertolookinto[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far != stringlength)
	{
		so_far =0;
		if (indx >= bufferlength) return 0;
		goto repeat;
	}

	if (so_far == stringlength) return 1;
	else return 0;
}


/* Waits for a particular string to arrive in the incoming buffer... It also increments the tail
 * returns 1, if the string is detected
 * return 0, in case of the timeout
 */
/************************************************************************************
FUNCTION		:waitFor
DESCRIPTION		:Waits for a particular string to arrive in the incoming buffer... It also increments the tail
INPUT			:string: search string; Timeout:
OUTPUT			:1: if the string is detected; 0: in case of the timeout or search counter ends
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t waitFor (char *string, uint32_t Timeout)
{
	int so_far =0;
	int again_count =0;
	int len = strlen (string);

	TIMEOUT = Timeout;

	while ((Tail==Head)&&TIMEOUT);  // let's wait for the data to show up
	isDataAvailable = FALSE;

again:
	again_count++;
	/* If the data doesn't show up, then return 0 */
	if (TIMEOUT <= 0) return 0;

	if (again_count > 6) return 0;

	/* if the incoming data does not match with the string, we will simply increment the index
	 * And wait for the string to arrive in the incoming data
	 * */
	while (MainBuf[Tail] != string[so_far])  // peek in the rx_buffer to see if we get the string
	{
		if (TIMEOUT <= 0) return 0;


		if (Tail == Head) goto again;
		Tail++;

		if (Tail==MainBuf_SIZE) Tail = 0;
	}

	/* If the incoming data does match with the string, we will return 1 to indicate this */
	while (MainBuf[Tail] == string[so_far]) // if we got the first letter of the string
	{
		if (TIMEOUT <= 0) return 0;
		so_far++;

		if (Tail == Head) goto again;
		Tail++;
		if (Tail==MainBuf_SIZE) Tail = 0;
		if (so_far == len) return 1;
	}

//	if (so_far != len)
//	{
//		so_far = 0;
//		goto again;
//	}

//	HAL_Delay (100);

	if ((so_far!=len)&&isDataAvailable)
	{
		isDataAvailable = FALSE;
//		so_far = 0;
		goto again;
	}
	else
	{
		so_far = 0;
		goto again;
	}


	return 0;
}


/************************************************************************************
FUNCTION		:copyUpto
DESCRIPTION		:copies the data from the incoming buffer into our buffer
INPUT			:string: search string; buffertocopyinto: target buffer; Timeout:
OUTPUT			:1: if the end string gets copied; 0: in case of the timeout
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t copyUpto (char *string, char *buffertocopyinto, uint32_t Timeout)
{
	int so_far =0;
	int len = strlen (string);
	int indx = 0;

	TIMEOUT = Timeout;
	while ((Tail==Head)&&TIMEOUT);
	isDataAvailable = FALSE;
again:

	if (TIMEOUT<=0) return 0;

	/* Keep copying data until the string is found in the incoming data */
	while (MainBuf[Tail] != string [so_far])
	{
		buffertocopyinto[indx] = MainBuf[Tail];

		if (Tail == Head) goto again;
		Tail++;
		indx++;
		if (Tail==MainBuf_SIZE) Tail = 0;
	}

/* If the string is found, copy it and return 1
 * or else goto again: and keep copying
 */
	while (MainBuf[Tail] == string [so_far])
	{
		so_far++;
		buffertocopyinto[indx++] = MainBuf[Tail++];
		if (Tail==MainBuf_SIZE) Tail = 0;
		if (so_far == len) return 1;
	}

//	HAL_Delay (100);

	if ((so_far!=len)&&isDataAvailable)
	{
		isDataAvailable = FALSE;
//		so_far = 0;
		goto again;
	}
	else
	{
		so_far = 0;
		goto again;
	}
    return 0;
}

/************************************************************************************
FUNCTION		:getAfter
DESCRIPTION		:Copies the entered number of characters, after the entered string (from the incoming buffer), into the buffer
INPUT			:string: header string; numberofchars: number to chars; buffertocopyinto: target buffer; Timeout:
OUTPUT			:1: if the string is copied; 0: in case of the timeout
UPDATE			:2022/5/17
*************************************************************************************/
uint8_t getAfter (char *string, uint8_t numberofchars, char *buffertocopyinto, uint32_t Timeout)
{
	if ((waitFor(string, Timeout)) != 1) return 0;
//	TIMEOUT = Timeout/3;
//	while (TIMEOUT > 0);
//	HAL_Delay (100);
	for (int indx=0; indx<numberofchars; indx++)
	{
		if (Tail==MainBuf_SIZE) Tail = 0;
		buffertocopyinto[indx] = MainBuf[Tail++];  // save the data into the buffer... increments the tail
	}
	return 1;
}


/************************************************************************************
FUNCTION		:getDataFromBuffer
DESCRIPTION		:Copies the data between the 2 strings from the source buffer into the destination buffer
INPUT			:startString: start string; endString: end string; buffertocopyfrom: source buffer; buffertocopyinto: target buffer;
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void getDataFromBuffer (char *startString, char *endString, char *buffertocopyfrom, char *buffertocopyinto)
{
	int startStringLength = strlen (startString);
	int endStringLength   = strlen (endString);
	int so_far = 0;
	int indx = 0;
	int startposition = 0;
	int endposition = 0;

repeat1:

	while (startString[so_far] != buffertocopyfrom[indx]) indx++;
	if (startString[so_far] == buffertocopyfrom[indx])
	{
		while (startString[so_far] == buffertocopyfrom[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far == startStringLength) startposition = indx;
	else
	{
		so_far =0;
		goto repeat1;
	}

	so_far = 0;

repeat2:

	while (endString[so_far] != buffertocopyfrom[indx]) indx++;
	if (endString[so_far] == buffertocopyfrom[indx])
	{
		while (endString[so_far] == buffertocopyfrom[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far == endStringLength) endposition = indx-endStringLength;
	else
	{
		so_far =0;
		goto repeat2;
	}

	so_far = 0;
	indx=0;

	for (int i=startposition; i<endposition; i++)
	{
		buffertocopyinto[indx] = buffertocopyfrom[i];
		indx++;
	}
}

/************************************************************************************
FUNCTION		:HAL_UARTEx_RxEventCallback
DESCRIPTION		:receive the data using the DMA until an IDLE event occurs, or all the data have been received.
				The incoming data are saved into the RxBuf, which will be processed later. When we enable DMA
				transfer using HAL, all the interrupts associated with it are also enabled. Once the IDLE event
				occurs, an interrupt will be triggered, and the Rx event callback will be called. Then we will
				process the data inside this callback.
INPUT			:huart: uart name; Size: data size:
OUTPUT			:None
UPDATE			:2022/5/17
*************************************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
		isDataAvailable = TRUE;

		oldPos = newPos;  // Update the last position before copying new data

		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		 * This is to maintain the circular buffer
		 * The old data in the main buffer will be overlapped
		 */
		if (oldPos+Size > MainBuf_SIZE)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy = MainBuf_SIZE-oldPos;  // find out how much space is left in the main buffer
			memcpy ((char *)MainBuf+oldPos, (char *)RxBuf, datatocopy);  // copy data in that remaining space

			oldPos = 0;  // point to the start of the buffer
			memcpy ((char *)MainBuf, (char *)RxBuf+datatocopy, (Size-datatocopy));  // copy the remaining data
			newPos = (Size-datatocopy);  // update the position
		}

		/* if the current position + new data size is less than the main buffer
		 * we will simply copy the data into the buffer and update the position
		 */
		else
		{
			memcpy ((char *)MainBuf+oldPos, (char *)RxBuf, Size);
			newPos = Size+oldPos;
		}

		/* Update the position of the Head
		 * If the current position + new size is less then the buffer size, Head will update normally
		 * Or else the head will be at the new position from the beginning
		 */
		if (Head+Size < MainBuf_SIZE) Head = Head+Size;
		else Head = Head+Size - MainBuf_SIZE;

		/****************** PROCESS (Little) THE DATA HERE *********************
		 * This is the PART OF  "isConfirmed" Function
		 */

		/* Let's say we want to check for the keyword "OK" within our incoming DATA */
			if (huart == device_uart){
				for (int i=0; i<Size; i++){
					if (RxBuf[i] == '\6'){
						isPresent = TRUE;
						debugLog("Motor controller present\n");
						break;
					}else if ((RxBuf[i] == '+') && (RxBuf[i+1] == '\r')){
						isSetCmdOK = TRUE;
//						debugLog("Motor controller command succeeds\n");
						break;
					}else if ((RxBuf[i] == 'C') && (RxBuf[i+1] == '=')){
						isWheelReadOK = TRUE;
//						memcpy ((uint8_t *)deck_array, (uint8_t *)&RxBuf[i+3], 8);
						Split_Str(&RxBuf[i+1], "=", &roboteq_array[0]);
						motor_read_buf = atoi(roboteq_array[0]);
//						encoder_count_1 = atoi(roboteq_array[0]);
//						encoder_count_2 = atoi(roboteq_array[1]);
						//debugLog("Encoder count read data succeeds\n");
						break;
					}else if ((RxBuf[i] == 'S') && (RxBuf[i+1] == '=')){
						isWheelReadOK = TRUE;
						Split_Str(&RxBuf[i+1], "=", &roboteq_array[2]);
						motor_read_buf = atoi(roboteq_array[2]);
//						encoder_speed_1 = atoi(roboteq_array[2]);
//						encoder_speed_2 = atoi(roboteq_array[3]);
						//debugLog("Encoder speed read data succeeds\n");
						break;
					}else if ((RxBuf[i] == 'F') && (RxBuf[i+1] == 'F') && (RxBuf[i+2] == '=')){
						isWheelReadOK = TRUE;
						Split_Str(&RxBuf[i+2], "=", &roboteq_array[4]);
						fault_flag = atoi(roboteq_array[4]);
						//debugLog("Fault flag read data succeeds\n");
						break;
					}else if ((RxBuf[i] == 'A') && (RxBuf[i+1] == '=')){
						isWheelReadOK = TRUE;
						Split_Str(&RxBuf[i+1], "=", &roboteq_array[6]);
						motor_read_buf = atoi(roboteq_array[6]);
						break;
					}else if ((RxBuf[i] == 'P') && (RxBuf[i+1] == '=')){
						isWheelReadOK = TRUE;
						Split_Str(&RxBuf[i+1], "=", &roboteq_array[6]);
						motor_read_buf = atoi(roboteq_array[6]);
						break;
					}else if ((RxBuf[i] == 'M') && (RxBuf[i+1] == 'D') && (RxBuf[i+2] == 'E') && (RxBuf[i+3] == 'C') && (RxBuf[i+4] == '=')){
						isWheelReadOK = TRUE;
						Split_Str(&RxBuf[i+4], "=", &roboteq_array[6]);
						motor_read_buf = atoi(roboteq_array[6]);
						break;
					}else if ((RxBuf[i] == 'B') && (RxBuf[i+1] == 'A') && (RxBuf[i+2] == '=')){
						isWheelReadOK = TRUE;
						Split_Str(&RxBuf[i+2], "=", &roboteq_array[6]);
						motor_read_buf = atoi(roboteq_array[6]);
						break;
					}
#if ENABLE_STREAM_QUERY
					else if ((RxBuf[i] == 'D') && (RxBuf[i+1] == 'H') && (RxBuf[i+2] == '?')){
						Split_Str(&RxBuf[i+3], "?:", &roboteq_array[0]);
						encoder_count_1 = atoi(roboteq_array[0]);		// ?C
						encoder_count_2 = atoi(roboteq_array[1]);		// ?C
						encoder_speed_1 = atoi(roboteq_array[2]);		// ?S
						encoder_speed_2 = atoi(roboteq_array[3]);		// ?S
						fault_flag = atoi(roboteq_array[4]);			// ?FF
		//				debugLog("Receiving query data from Roboteq succeeds\n");
						isQueryOK = TRUE;
						break;
					}
#endif
				}
			}else if(huart == deck_uart){
				for (int i=0; i<Size; i++){
					if (RxBuf[i] == '\6'){
						isDeckPresent = TRUE;
						debugLog("Deck controller present\n");
						break;
					}else if ((RxBuf[i] == '+') && (RxBuf[i+1] == '\r')){
						isDeckSetCmdOK = TRUE;
						debugLog("Deck controller command succeeds\n");
						break;
					}else if ((RxBuf[i] == 'C') && (RxBuf[i+1] == 'B') && (RxBuf[i+2] == '=')){
						isDeckReadOK = TRUE;
//						memcpy ((uint8_t *)deck_array, (uint8_t *)&RxBuf[i+3], 8);
						Split_Str(&RxBuf[i+2], "=", &deck_array[0]);
						hall_count_buffer = atoi(deck_array[0]);
						debugLog("Deck CB read data succeeds\n");
						break;
					}else if ((RxBuf[i] == 'B') && (RxBuf[i+1] == 'S') && (RxBuf[i+2] == '=')){
						isDeckReadOK = TRUE;
						Split_Str(&RxBuf[i+2], "=", &deck_array[2]);
						hall_speed = atoi(deck_array[2]);
						debugLog("Deck BS read data succeeds\n");
						break;
					}else if ((RxBuf[i] == 'F') && (RxBuf[i+1] == 'M') && (RxBuf[i+2] == '=')){
						isDeckReadOK = TRUE;
						Split_Str(&RxBuf[i+2], "=", &deck_array[0]);
						lifter_status = atoi(deck_array[0]);
						debugLog("Lifter FM read data succeeds\n");
						break;
					}else if ((RxBuf[i] == 'D') && (RxBuf[i+1] == 'I') && (RxBuf[i+2] == '=')){
						isDeckReadOK = TRUE;
						Split_Str(&RxBuf[i+2], "=", &deck_array[0]);
						lifter_sensor = atoi(deck_array[0]);
						debugLog("Lifter upper/lower sensor read data succeeds\n");
						break;
					}
				}
			}else if(huart == pc_uart){
				for (int i=0; i<Size; i++){
					if ((RxBuf[i] == 'G') && (RxBuf[i+1] == 'R')){
						memcpy ((uint8_t *)PcData, (uint8_t *)&RxBuf[i+2], Size-2);
//				if (getAfter("GR",Size-2,PcData,1000) == 1){
#if PC_CHECKSUM_ENABLE
						if (PcData[DataIdx+PcData[DataLengthIdx]] == uart_ucGetCheckSum(&PcData[DataLengthIdx], PcDataLength_SIZE+PcCmdID_SIZE+PcData[DataLengthIdx])){
#else
						if (1){
#endif
							if (PcData[CmdTypeIdx] == TYPE_DATA){
#if MCU_INITREQ_ENABLE
								if (PcData[CmdIDIdx] == PC_CMDID_INITPARA){
									isPCPresent = TRUE;
									debugLog("PC device present\n");
								}else{
#endif
									uart_Ack2PC(TYPE_ACK, 0);
									isPCDataOK = TRUE;
									Current_Cmd_Update();
									cmd_sts = PC_CMDSTS_INPROGRESS;
									debugLog("Acknowledge OK to PC\n");
#if MCU_INITREQ_ENABLE
								}
#endif
							}else if (PcData[CmdTypeIdx] == TYPE_ACTION){
				        		action_sts = PcData[DataIdx];
								uart_Ack2PC(TYPE_ACTION_ACK, 0);
				        		isPCActionOK = TRUE;
							}else if (PcData[CmdTypeIdx] == TYPE_STATUS){
								status_cmdid = PcData[CmdIDIdx];
#if ENABLE_PC_STATUS_REQ
								if(status_cmdid == PC_STATUS_CMDID_STATUS){
									motor_deck_status_bytes = PcData[DataIdx];
									isPCStatusOK = TRUE;
								}else
#endif
								if(status_cmdid == PC_STATUS_CMDID_QUERY){
									para_query = PcData[DataIdx];
									isPCStatusOK = TRUE;
								}else if(status_cmdid == PC_STATUS_CMDID_INITPARA){
									isPCStatusOK = TRUE;
									uart_Ack2PC(TYPE_STATUS_ACK, 0);
								}
							}else if (PcData[CmdTypeIdx] == TYPE_ACK){
								if (PcData[DataIdx] == 0){
									isPCAckOK = TRUE;
									debugLog("Acknowledge OK from PC\n");
								}else{
									isPCAckOK = FALSE;
									debugLog("Acknowledge NG from PC\n");
								}
							}
						}else{
							uart_Ack2PC(PcData[CmdTypeIdx], 1);
							if (PcData[CmdTypeIdx] == TYPE_DATA){
								cmd_sts = PC_CMDSTS_ERROR;
							}
							debugLog("Acknowledge NG to PC\n");
						}
						break;
					}
				}
			}
		/* start the DMA again */
		memset(RxBuf, '\0', RxBuf_SIZE);
		if (huart == device_uart){
			HAL_UARTEx_ReceiveToIdle_DMA(&huart7, (uint8_t *) RxBuf, RxBuf_SIZE);
			__HAL_DMA_DISABLE_IT(&DMA_UART7, DMA_IT_HT);
		}else if(huart == pc_uart){
			HAL_UARTEx_ReceiveToIdle_DMA(&huart4, (uint8_t *)RxBuf, RxBuf_SIZE);
			__HAL_DMA_DISABLE_IT(&DMA_UART4, DMA_IT_HT);
		}else if (huart == deck_uart){
			HAL_UARTEx_ReceiveToIdle_DMA(&huart8, (uint8_t *) RxBuf, RxBuf_SIZE);
			__HAL_DMA_DISABLE_IT(&DMA_UART8, DMA_IT_HT);
		}
}

