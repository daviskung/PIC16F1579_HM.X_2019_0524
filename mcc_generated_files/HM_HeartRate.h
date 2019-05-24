#include <xc.h>		// Important Head File
#include "tmr2.h"
#include "adc1.h"
#include "pin_manager.h"
#include "eusart.h"

#define		HEART_RATE_DUG_MSG_FUN	0
#define		STOP_DUG_MSG_FUN	0
#define		AGC_MCP4011_DUG_MSG_FUN	0


#define	NS1_PIN		IO_RB4_LAT	//LATB4
#define	NS2_PIN		IO_RB5_LAT	//LATB5
#define	NS3_PIN		IO_RB6_LAT	//LATB6
#define	NS4_PIN		IO_RB7_LAT	//LATB7
#define	All_NS_pin	LATB

#define GUD_PIN		IO_RA5_LAT	//LATB4	//IO_RA5_LAT
#define GCS_PIN		IO_RA4_LAT	//LATB4	//IO_RA4_LAT

#define	SAMP_PIN	IO_RC6_LAT	//LATC6

#define	NDISCH_PIN			IO_RC3_LAT	//LATC3
#define	NDISCH_portSet		IO_RC3_TRIS	//TRISC3

#define MAXGAIN 63
#define MINGAIN 0
#define MIDGAIN 32

#define ADCON0_ADC_ENABLE	0x29
#define ADCON0_ADC_DISABLE	0x28


#define		GO_RESET	2
#define		GO_UP		1
#define		GO_DOWN		0

#define		TRUE		1
#define		FALSE		0

#define		WAIT		1
#define		KEEP_GO		0


#define		SET_FLAG	1
#define		CLEAR_FLAG	0


#define		IN_prepare_STATUS		0
#define		IN_GET_AVG_STATUS		1
#define		IN_NOT_RANGE_STATUS		2

#define		GAIN_MIN_LIMIT		14

#define		AUTO_FIND_SET_POINT		0

#define		HM_ADC_CH_AR	channel_AN4	
#define		HM_ADC_CH_AN0	channel_AN6	
#define		HM_ADC_CH_AN2	channel_AN5	
#define 	AN0_SAMPLE_QUEUE_SIZE 	5
#define 	AN2_SAMPLE_SIZE 		500 // for 4ms * 500 = 2000ms(too large)
#define 	AN2_ARY_SAMPLE_SIZE 		100 // for 4ms per 5 times(20ms) * 100 = 2000ms

#define 	AN2_SAMPLE_Inv4s_SIZE 		200

#define 	AN2_SAMPLE_AVGBUF_SIZE 		8
#define 	AN2_PULSE_COUNT_SIZE 		10
#define		AN2HR_BUF_SIZE				5
#define		OVER_TH_LIMIT				100

#define		VAL_1MIN_MS			60000
#define		VAL_2SEC_MS			2000
#define		VAL_30_MIN_BPM		30		// 2000ms
#define		VAL_210_MAX_BPM		210		// 285ms

#define		VAL_ERR_RANGE_PERCENT		20
#define		VAL_NN_INTERVAL_DIFF		100	// 100ms 
// 每次心跳與心跳的間隔均有幾十毫秒以內的微小差異

#define		V3P3_VALVE		1023	// 3.3V ADC
#define		V3P3max_VALVE	1010	// 3.3V ADC
#define		V3P0_VALVE		930		// 3.0V ADC
#define		V2P8_VALVE		868		// 2.8V ADC
#define		V2P5_VALVE		775		// 2.5V ADC
#define		V2P2_VALVE		682		// 2.2V ADC
#define		V2P0_VALVE		612		// 2V ADC
#define		V1P8_VALVE		558		// 1.8V ADC
#define		V1P65_VALVE		512		// 1.65V ADC
#define		V1P5_VALVE		465		// 1.5V ADC
#define		V1P0_VALVE		310		// 1.0V ADC
#define		V0P6_VALVE		186		// 0.6V ADC
#define		V0P5_VALVE		155		// 0.5V ADC
#define		V0P4_VALVE		124		// 0.4V ADC
#define		V0P3_VALVE		93		// 0.3V ADC
#define		V0P2_VALVE		62		// 0.2V ADC
#define		V0P1_VALVE		31		// 0.1V ADC
#define		V0min_VALVE		10		// 0V min ADC

#define		SEND_toRTL_DUG_CMD_SIZE	10


uint8_t		PIC_DUG_cmd_OUT[SEND_toRTL_DUG_CMD_SIZE] = { 0xff,'D', 'U' , 'G' , '=' ,'N','S','1'
			,'\r' , '\n'} ; // PIC do not know this \n \r


void FindSetPoint(uint16_t AN0Value);
void initialHWset(void);
void StopRunHW(void);
uint8_t MCP_setVal(uint8_t UP_DOWN_dir);

void GainStepDown (void);
void GainStepUp (void);
void GainDelay (void);

void InitGain(uint8_t val);
void GainUp (void);
void GainDown (void);
void DugDataMsg(uint8_t DugCma,uint8_t DugCm0,uint8_t DugCm1,uint16_t DugCm2);
void DugCmdMsg(uint8_t DugCm0,uint8_t DugCm1,uint8_t DugCm2);
void DugHRMsg(uint8_t DugCmA,uint8_t GainV10,uint8_t GainV1,uint8_t DugCm0,uint8_t DugCm1,uint8_t DugCm2);




const uint8_t HexNumTable[16]={
 '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};


typedef enum
{
    PowerOn_Event = 0,	
	RTL_PwrOncmd_Event,
	RTL_HWEnablecmd_Event,
	HW_running_Event,
	HW_STOP_run_Event,
	RTL_HWStopcmd_Event,	
	RTL_cmdError_Event
} SystemControlStatus_Event;


typedef enum
{
    AN0_OVER_RANGE_Event = 0,
	AN0_IN_RANGE_Event,
	AN0_UNDER_RANGE_Event
    
} AN0ControlStatus_Event;



typedef enum
{
	ADC_CH_AR_Event = 0,
	ADC_CH_AN0_Event,
	ADC_CH_AN2_Event
    
} ADCcurrentChan_Event;

