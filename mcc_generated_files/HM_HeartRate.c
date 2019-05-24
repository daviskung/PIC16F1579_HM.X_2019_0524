//*************************************************************
//	2018.12.25	set NSTROBE_LOW_EndSet = 0 error
//*************************************************************

#include "HM_HeartRate.h"

#define		SEND_toRTL_HR_MSG_SIZE	11


uint8_t NSTROBE_PWM_cnt;
extern uint8_t AGC_MCP4011_Gain;  // current setting

ADCcurrentChan_Event	CurrentChan;
uint8_t NSTROBE_LOW_EndSet;
uint8_t NSTROBE_Rset;
bit bAN0_ADC_DugOn;
bit bADC_AN0_Delay ;
bit bAN0_InRange;

// 取消 看 AN0_IN_RANGE_Event 時間 , 直接看 AN0Value 極小時
// 直接 reset AGC_MCP4011_Gain ->  InitGain(MIDGAIN);
//uint16_t FindSetP_cnt; 
//uint16_t FindSetP_TMR_cnt;
uint8_t AN0_InRange_cnt;

uint16_t an2SampleCnt;
bit bAN2_GainContWaitFlag;
bit bAN2_LowAmplitudeFlag;
uint8_t AN2_GainContInRange_cnt;
uint8_t AGC_MCP4011_GainLimitSet,AN2_GainContOutOfRange_CycleCnt;

uint8_t InRangeStatus,AN2_HRBufferIndex;

uint16_t AN2_oldPulseCnt;

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
	NSTROBE_Rset = 2;
	NSTROBE_LOW_EndSet = 5;
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


void DugHRMsg(uint8_t DugCmA,uint8_t GainV10,uint8_t GainV1,uint8_t DugCm0,uint8_t DugCm1,uint8_t DugCm2)
{
					
	PIC_HR_Msg_OUT[0] = 'H';
	PIC_HR_Msg_OUT[1] = DugCmA;
	PIC_HR_Msg_OUT[2] = '=';
	PIC_HR_Msg_OUT[3] = GainV10;
	PIC_HR_Msg_OUT[4] = GainV1;
	PIC_HR_Msg_OUT[5] = '-';
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


void FindSetPoint(uint16_t AN0Value) // run per 20ms*5 = 100ms
{

	AN0ControlStatus_Event AN0_SetPointStatus,tmpAN0_SetPointStatus;

	//FindSetP_TMR_cnt++;
	//
	// Enlarge AN0 range to reduce DeadZone range
	//
	AN0_SetPointStatus = AN0_IN_RANGE_Event;
	if ( AN0Value >= V2P8_VALVE ) AN0_SetPointStatus = AN0_OVER_RANGE_Event;
	else if( AN0Value < V0P6_VALVE ) AN0_SetPointStatus = AN0_UNDER_RANGE_Event;
	
	tmpAN0_SetPointStatus = AN0_SetPointStatus;

	switch (AN0_SetPointStatus)
	{
			case AN0_IN_RANGE_Event: 
				if( bAN0_ADC_DugOn == TRUE ) DugDataMsg('A','m','I',AN0Value) ; 
				//FindSetP_cnt ++;
				if( AN0_InRange_cnt < 15 ) AN0_InRange_cnt++;
				
        			break; 
			
			case AN0_OVER_RANGE_Event:
				NSTROBE_Rset = 0;
				NSTROBE_LOW_EndSet = 1;
				bADC_AN0_Delay = TRUE;
			
				bAN0_InRange = FALSE;
				AN0_InRange_cnt = 0;
				an2SampleCnt = 0;
				
				if( bAN0_ADC_DugOn == TRUE ) DugDataMsg('A','m','O',AN0Value) ; 
        			break;
			
			case AN0_UNDER_RANGE_Event: 
				NSTROBE_LOW_EndSet++;
				if( NSTROBE_LOW_EndSet > 9 ){
					NSTROBE_LOW_EndSet=1;
					NSTROBE_Rset++;
					if( NSTROBE_Rset > 3 ) NSTROBE_Rset = 1;
				}
				bADC_AN0_Delay = TRUE;
				
				bAN0_InRange = FALSE;
				AN0_InRange_cnt = 0;
				
				an2SampleCnt = 0;
				
				if( bAN0_ADC_DugOn == TRUE ) DugDataMsg('A','m','U',AN0Value) ; 
				
				if(( AGC_MCP4011_Gain != MIDGAIN ) &&( AN0Value <= V0min_VALVE )) {
						InitGain(MIDGAIN);
				#if AGC_MCP4011_DUG_MSG_FUN 
						DugDataMsg('M','C','P',0x32) ; 
				#endif
				}
				
        			break;
					
			default:
					break;
				
	}

	if (AN0_InRange_cnt == 15){
		bAN0_InRange = TRUE; // AN0 In Range over 500 msec
		bAN2_GainContWaitFlag = KEEP_GO;	
		bAN2_LowAmplitudeFlag = TRUE;
		AN2_GainContInRange_cnt = 0;
		InRangeStatus = IN_prepare_STATUS;
		AGC_MCP4011_GainLimitSet = GAIN_MIN_LIMIT;
		AN2_GainContOutOfRange_CycleCnt = 0;
		AN2_HRBufferIndex = 0;
		AN2_oldPulseCnt = 0;
		AN0_InRange_cnt = 16;
	}
	

	
	
	if( bAN0_ADC_DugOn == TRUE ) DugDataMsg('A','N','0',AN0Value) ; 
}

