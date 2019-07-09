//*************************************************************
//	2018.12.25	set NSTROBE_LOW_EndSet = 0 error
//*************************************************************
// test 06010-2
#include "HM_HeartRate.h"

#define		SEND_toRTL_HR_MSG_SIZE	11
#define		AN0_InRange_LIMIT				20
#define		AN0_MONITOR_OFFSET				V1P5_VALVE



uint16_t NSTROBE_PWM_cnt;
extern uint8_t AGC_MCP4011_Gain;  // current setting

ADCcurrentChan_Event	CurrentChan;
uint16_t NSTROBE_LOW_EndSet;
int8_t NSTROBE_Rset;

bit bAN0_ADC_DugOn;
bit bADC_AN0_Delay ;

bit bAN0_InRange;
AN0_Status_Event AN0_state_control,Pre_AN0_state_control; 
adc_result_t	AN0SampAvgValue_1st;

int8_t FindSetPoint_AN0_exitLoop;  


// 取消 看 AN0_IN_RANGE_Event 時間 , 直接看 AN0Value 極小時
// 直接 reset AGC_MCP4011_Gain ->  InitGain(MIDGAIN);
//uint16_t FindSetP_cnt; 
//uint16_t FindSetP_TMR_cnt;
uint8_t AN0_InRange_cnt;

uint16_t an2SampleCnt;
bit bAN2_GainContWaitFlag;
uint8_t AN2_GainContInRange_cnt;
uint8_t AGC_MCP4011_GainLimitSet,AN2_GainContOutOfRange_CycleCnt;

uint8_t InRangeStatus,AN2_HRBufferIndex;

uint16_t AN2_oldPulseCnt;
//uint8_t AN0_OverRange_Cnt;

//uint8_t		PIC_HR_Msg_OUT[SEND_toRTL_HR_MSG_SIZE] = {0xff,'H', '=' ,'1','N','S','1','\n' , '\r'} ;
uint8_t		PIC_HR_Msg_OUT[SEND_toRTL_HR_MSG_SIZE] = {'H', '0' ,'=','G','V','-','N','S','1','\r' , '\n'} ;

/*******************************************************************************
 * Fiddle with the gain pot functions
 * THIS DRIVES A MICROCHIP 4011 WITH 64 STEPS
 *******************************************************************************/

void GainDelay (void)	// changing GUD or GCS requires a delay depending on digital pot chip
{
	volatile uint8_t dummy;
	
	for (dummy=0;dummy<5;dummy++);
}

void GainStepUp (void)
{
			
	GUD_PIN = LOW;  GainDelay();	//GPIO_WriteBit(GPIO_GetPin(GUD_PIN),Bit_RESET); 	GainDelay(); // GUD = 0;
	GCS_PIN = LOW;  GainDelay();	//GPIO_WriteBit(GPIO_GetPin(GCS_PIN),Bit_RESET); 	GainDelay(); // GCS = 0;
	GUD_PIN = HIGH; GainDelay();	//GPIO_WriteBit(GPIO_GetPin(GUD_PIN),Bit_SET);   	GainDelay(); // GUD = 1;
	GUD_PIN = LOW;  GainDelay();	//GPIO_WriteBit(GPIO_GetPin(GUD_PIN),Bit_RESET); 	GainDelay(); // GUD = 0;
	GCS_PIN = HIGH; GainDelay();	//GPIO_WriteBit(GPIO_GetPin(GCS_PIN),Bit_SET);   	GainDelay(); // GCS = 1;
	if (AGC_MCP4011_Gain < MAXGAIN) AGC_MCP4011_Gain++;
	
	//DBG_BUFFER(MODULE_APP, LEVEL_INFO, " ** GainStepUp AGC value = %d \n", 1,AGC_MCP4011_Gain);
}

void GainStepDown (void)
{
	GUD_PIN = HIGH; GainDelay();	//GPIO_WriteBit(GPIO_GetPin(GUD_PIN),Bit_SET);    GainDelay(); // GUD = 1;
	GCS_PIN = LOW;  GainDelay();	//GPIO_WriteBit(GPIO_GetPin(GCS_PIN),Bit_RESET);  GainDelay(); // GCS = 0;
	GUD_PIN = LOW;  GainDelay();	//GPIO_WriteBit(GPIO_GetPin(GUD_PIN),Bit_RESET);  GainDelay(); // GUD = 0;
	GUD_PIN = HIGH; GainDelay();	//GPIO_WriteBit(GPIO_GetPin(GUD_PIN),Bit_SET);    GainDelay(); // GUD = 1;
	GCS_PIN = HIGH; GainDelay();	//GPIO_WriteBit(GPIO_GetPin(GCS_PIN),Bit_SET);    GainDelay(); // GCS = 1;
	if (AGC_MCP4011_Gain > MINGAIN) AGC_MCP4011_Gain--;
	
	//DBG_BUFFER(MODULE_APP, LEVEL_INFO, " ** GainStepDown AGC value = %d \n", 1,AGC_MCP4011_Gain);
}

void InitGain(uint8_t val)
{
    uint8_t i;
	
    AGC_MCP4011_Gain = MAXGAIN+1;

    for (i=0;i<MAXGAIN+1;i++) GainStepDown();	// run into zero
    
    for (i=0;i<val;i++) GainStepUp();

}

/*******************************************************************************
 * GainUp and GainDown -- These are the main gain setting functions.
 * Implement an aproximate log behavior; 32 virtual steps to 64 physical steps
 *******************************************************************************/

void GainUp (void)
{
	if (AGC_MCP4011_Gain < 16) {
		GainStepUp();
	} else if (AGC_MCP4011_Gain < 40) {
		GainStepUp(); GainStepUp();
	} else {
		GainStepUp(); GainStepUp(); GainStepUp();
	}
}

void GainDown (void)
{
	if (AGC_MCP4011_Gain < 16) {
		GainStepDown();
	} else if (AGC_MCP4011_Gain < 40) {
		GainStepDown(); GainStepDown();
	} else {
		GainStepDown(); GainStepDown(); GainStepDown();
	}
}
uint8_t MCP_setVal(uint8_t UP_DOWN_dir)
{
	if (( MAXGAIN == AGC_MCP4011_Gain )&& ( UP_DOWN_dir == GO_UP )){
    	GainDown();
		UP_DOWN_dir=GO_DOWN;
	//	DBG_BUFFER(MODULE_APP, LEVEL_INFO, " **GainStepDown = %d \n", 1,AGC_MCP4011_Gain);
    			
   	}
    else if(( MINGAIN == AGC_MCP4011_Gain ) && ( UP_DOWN_dir == GO_DOWN )){
		GainUp();
		UP_DOWN_dir=GO_UP;
		//DBG_BUFFER(MODULE_APP, LEVEL_INFO, " **GainStepUp = %d \n", 1,AGC_MCP4011_Gain);
    			
    }
	 else if( UP_DOWN_dir == GO_RESET ){
		 InitGain(MIDGAIN);
		UP_DOWN_dir=GO_UP;
		//DBG_BUFFER(MODULE_APP, LEVEL_INFO, " **GainStepUp = %d \n", 1,AGC_MCP4011_Gain);
    			
    }
	else{
    	if( UP_DOWN_dir == GO_UP ){ 
			GainUp();
		//DBG_BUFFER(MODULE_APP, LEVEL_INFO, " **GainStepUp = %d \n", 1,AGC_MCP4011_Gain);
		}
		else{
			GainDown();
			//DBG_BUFFER(MODULE_APP, LEVEL_INFO, " **GainStepDown = %d \n", 1,AGC_MCP4011_Gain);
   		}
	}
	return UP_DOWN_dir;
}


void initialHWset(void)
{
	NSTROBE_PWM_cnt = 0;
	NSTROBE_Rset = 0;
	NSTROBE_LOW_EndSet = 20;
	
	//FindSetP_cnt = 0;
	//FindSetP_TMR_cnt =0;
	AN0_InRange_cnt = 0;
	
    ADCON0 = ADCON0_ADC_ENABLE; //  ADON Enabled
	InitGain(MIDGAIN);
	TMR2_StartTimer();	// start 250us counter	
	ADC1_SelectChannel(HM_ADC_CH_AN0);
	CurrentChan = ADC_CH_AN0_Event;
		return;
}

void StopRunHW(void)
{
	ADCON0 = ADCON0_ADC_DISABLE; //  ADON DISabled	
	All_NS_pin = 0xff;
	TMR2_StopTimer();  // stop 250us counter  
		   return;

}


void DugHRMsg(uint8_t DugCmA,uint8_t GainV10,uint8_t GainV1,uint8_t DugCm0
			,uint8_t DugCm1,uint8_t DugCm2,uint8_t NSTROBE,uint8_t NSTROBE_R)
{
					
	PIC_HR_Msg_OUT[0] = 'H';
	PIC_HR_Msg_OUT[1] = DugCmA;
	//PIC_HR_Msg_OUT[2] = '=';
	PIC_HR_Msg_OUT[2] = NSTROBE_R;
	PIC_HR_Msg_OUT[3] = GainV10;
	PIC_HR_Msg_OUT[4] = GainV1;
	//PIC_HR_Msg_OUT[5] = '-';
	PIC_HR_Msg_OUT[5] = NSTROBE;	// 送出 NSTROBE set value
	PIC_HR_Msg_OUT[6] = DugCm0;
	PIC_HR_Msg_OUT[7] = DugCm1;
	PIC_HR_Msg_OUT[8] = DugCm2;
	for( int i=0; i < SEND_toRTL_HR_MSG_SIZE ;i++)
		EUSART_Write( PIC_HR_Msg_OUT[i] );
}


void DugCmdMsg(uint8_t DugCm0,uint8_t DugCm1,uint8_t DugCm2)
{
					
	PIC_DUG_cmd_OUT[1] = 'D';
	PIC_DUG_cmd_OUT[2] = 'U';
	PIC_DUG_cmd_OUT[3] = 'G';
	PIC_DUG_cmd_OUT[4] = '=';
	PIC_DUG_cmd_OUT[5] = DugCm0;
	PIC_DUG_cmd_OUT[6] = DugCm1;
	PIC_DUG_cmd_OUT[7] = DugCm2;
	for( int i=0; i < SEND_toRTL_DUG_CMD_SIZE ;i++)
		EUSART_Write( PIC_DUG_cmd_OUT[i] );
}


void DugDataMsg(uint8_t DugCma,uint8_t DugCm0,uint8_t DugCm1,uint16_t DugCm2)
{
	PIC_DUG_cmd_OUT[1] = DugCma;
	PIC_DUG_cmd_OUT[2] = DugCm0;
	PIC_DUG_cmd_OUT[3] = DugCm1;
	
	PIC_DUG_cmd_OUT[4] = HexNumTable[ (DugCm2>>12) & 0x0f];				
	PIC_DUG_cmd_OUT[5] = HexNumTable[ (DugCm2>>8) & 0x0f];				
	PIC_DUG_cmd_OUT[6] = HexNumTable[ (DugCm2>>4) & 0x0f];				
	PIC_DUG_cmd_OUT[7] = HexNumTable[ DugCm2 & 0x0f];			
	for( int i=0; i < SEND_toRTL_DUG_CMD_SIZE ;i++)
		EUSART_Write( PIC_DUG_cmd_OUT[i] );
}


void FindSetPoint(uint16_t AN0Value)	 
{
	
	//		AN0_state_control = AN0_NO_MAN_state_Event; 從 main.c 設定

	// AN0 已進入"目標區" 監控狀態
	if(AN0_state_control == AN0_GET_RIGHT_POINT_Event){

		FindSetPoint_AN0_exitLoop = 0;
		
		//DugDataMsg('A','S','=',AN0Value) ;
		
		
		if((AN0Value > (AN0SampAvgValue_1st + V1P5_VALVE)) ||
			(AN0Value > V3P0_VALVE ) ){
		//if(AN0Value >  AN0_MONITOR_OFFSET){
			
			AN0_state_control = AN0_GO_DOWN_Event;
			DugDataMsg('A','S','-',AN0Value) ;
		}
		else if (AN0Value < V0P5_VALVE) {
			AN0_state_control = AN0_GO_UP_Event;
			DugDataMsg('A','S','+',AN0Value) ;
		}
	}
	// "目標區"外 AN0 電壓檢查 
	else{
		if(AN0Value >= V1P65_VALVE){ 	
			AN0_state_control = AN0_GO_DOWN_Event;
		}
	
		else if(AN0Value < V0P5_VALVE){	
			AN0_state_control = AN0_GO_UP_Event;
		}

		// find the RIGHT point
		
		else if(( AN0Value > V0P5_VALVE ) && (AN0Value < V1P65_VALVE)){
			Pre_AN0_state_control = AN0_GET_RIGHT_POINT_Event;
			AN0SampAvgValue_1st = AN0Value;
			AN0_state_control = AN0_GET_RIGHT_POINT_Event;
			bADC_AN0_Delay = TRUE;
		
			DugDataMsg('A','S','I',AN0SampAvgValue_1st) ; 
		}
	}
	switch (AN0_state_control)
	{
			
		case AN0_GO_UP_Event: 
			AN0_InRange_cnt = 0;
			FindSetPoint_AN0_exitLoop++;
			
			NSTROBE_LOW_EndSet = NSTROBE_LOW_EndSet + 1;
			if( NSTROBE_LOW_EndSet > 39 ){
				NSTROBE_LOW_EndSet=1;
				NSTROBE_Rset++;
				if( NSTROBE_Rset > 3 ) NSTROBE_Rset = 0;
			}
			
			
			bADC_AN0_Delay = TRUE;
			bAN0_InRange = FALSE;
			
			if( bAN0_ADC_DugOn == TRUE ){
					DugDataMsg('A','g','U',AN0Value) ; 
					DugDataMsg('A','U','N',NSTROBE_LOW_EndSet) ; 
					DugDataMsg('A','U','R',NSTROBE_Rset) ; 
					DugDataMsg('u','u','u',0) ; 
			}
			
				break;
		case AN0_GO_DOWN_Event: 
			AN0_InRange_cnt = 0;
			FindSetPoint_AN0_exitLoop++;
		
			// SRS 趨近法
			if(NSTROBE_Rset <= 1){
				NSTROBE_LOW_EndSet = NSTROBE_LOW_EndSet - 1;
				if(NSTROBE_LOW_EndSet <= 1) {
					NSTROBE_LOW_EndSet = 39;
					NSTROBE_Rset--;
					if( NSTROBE_Rset < 0 ) NSTROBE_Rset = 0;
				}
			}
			else if(NSTROBE_Rset > 1)
			{
				NSTROBE_LOW_EndSet = NSTROBE_LOW_EndSet-3;
				if(NSTROBE_LOW_EndSet <= 1){
					NSTROBE_LOW_EndSet = 39;
					NSTROBE_Rset--;
					if( NSTROBE_Rset < 0 ) NSTROBE_Rset = 0;
				}
			}
			bADC_AN0_Delay = TRUE;
			bAN0_InRange = FALSE;
			
			if( bAN0_ADC_DugOn == TRUE ){
					DugDataMsg('A','g','d',AN0Value) ; 
					DugDataMsg('A','D','N',NSTROBE_LOW_EndSet) ; 
					DugDataMsg('A','D','R',NSTROBE_Rset) ; 
					DugDataMsg('d','d','d',0) ; 
			}
			
			Pre_AN0_state_control = AN0_GO_DOWN_Event;
				break;
		
		case AN0_GET_RIGHT_POINT_Event:
			//
			// 近入目標區 但 未穩定 !!
			//
			if( AN0_InRange_cnt < AN0_InRange_LIMIT ){

				if(AN0Value > V1P65_VALVE)	AN0_state_control = AN0_GO_DOWN_Event;
									
				else if(AN0Value < V0P5_VALVE) AN0_state_control = AN0_GO_UP_Event;
				
				else AN0_InRange_cnt++;
				
				DugDataMsg('I','N','+',AN0_InRange_cnt) ; 
			}
				break;

		case AN0_NO_MAN_state_Event: 
			DugDataMsg('n','o','m',AN0Value) ; 
			if(( AGC_MCP4011_Gain != MIDGAIN ) &&( AN0Value <= V0min_VALVE )) {
						InitGain(MIDGAIN);
						DugDataMsg('M','C','N',0x32) ; 
				#if AGC_MCP4011_DUG_MSG_FUN 
						DugDataMsg('M','C','P',0x32) ; 
				#endif
			}
			
				break;
				
		default:
				break;
	}


	//
	// 入目標區 已達穩定 !!
	//
	if (AN0_InRange_cnt == AN0_InRange_LIMIT) {
		bAN0_InRange = TRUE; // AN0 In Range over 500 msec
		bAN2_GainContWaitFlag = KEEP_GO;	
		AN2_GainContInRange_cnt = 0;
		InRangeStatus = IN_prepare_STATUS;
		AGC_MCP4011_GainLimitSet = GAIN_MIN_LIMIT;
		AN2_GainContOutOfRange_CycleCnt = 0;
		AN2_HRBufferIndex = 0;
		AN2_oldPulseCnt = 0;
		AN0_InRange_cnt = AN0_InRange_LIMIT+1;
		
		DugDataMsg('I','N','c',AN0_InRange_cnt) ; 
		DugDataMsg('I','N','n',NSTROBE_LOW_EndSet) ; 
		DugDataMsg('I','N','r',NSTROBE_Rset) ; 

		InitGain(MIDGAIN);
		DugDataMsg('M','C','N',0x32) ; 
		
	}

	if( FindSetPoint_AN0_exitLoop > 60){
		NSTROBE_Rset = 0;
		NSTROBE_LOW_EndSet = 20;
		FindSetPoint_AN0_exitLoop = 0;
		DugDataMsg('e','x','t',FindSetPoint_AN0_exitLoop) ; 
	}


	//if( bAN0_ADC_DugOn == TRUE ) DugDataMsg('A','N','0',AN0Value_max) ; 
}



