#include "I_collect.h"
#include "delay.h"

#define CURRENT_WIN         60
#define COLLECT_INTERVAL    5000   // 5000*200us = 1000ms
#define COLLECT_SAMPLE_NUM  100

uint16_t ADC_IMIN = 0;
uint16_t ADC_IMAX = 0;

uint16_t curr_mvData[CURRENT_WIN] = {0};
uint8_t curr_mvPtr = 0;
uint32_t last_curr_sum = 0;
uint16_t eqIndex = 0;

uint16_t m_collectTick = 0;

uint8_t adjustVrefStart = 0;

void adjustVref()
{
  adjustVrefStart = 1;  
}

void Timer1_init()
{
	TIM1_TimeBaseInit(16,TIM1_COUNTERMODE_UP,200,0);//200us
	TIM1_ARRPreloadConfig(ENABLE);
	TIM1_ITConfig(TIM1_IT_UPDATE , ENABLE);
	TIM1_Cmd(ENABLE);
}

void init_ADC()
{
	GPIO_Init(GPIOB,GPIO_PIN_2,GPIO_MODE_IN_FL_NO_IT);
	ADC1_DeInit();
	ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,ADC1_CHANNEL_2,ADC1_PRESSEL_FCPU_D8,ADC1_EXTTRIG_TIM, 
			  DISABLE,ADC1_ALIGN_RIGHT,ADC1_SCHMITTTRIG_CHANNEL2,DISABLE);
	ADC1_Cmd(ENABLE);
	ADC1_StartConversion();	
        Timer1_init();
        memset(curr_mvData, 0x00, sizeof(uint16_t) * CURRENT_WIN);
}

//the turn ratio of Sensor is 1:2000£¬sampling resistor R=20£¬Vref = 1.65V
//Vin=1.65-I/2000*20*10
//AD_value=Vin/3.3*1024=(1.65-I/2000*20*10)/3.3*1024=AD_Vref-I*31
//Imax=(AD_Vref-AD_value_min)/31;
//Iefc=Imax*0.707
//Iefc=(AD_Vref-AD_value_min)*0.707/31*100=(AD_Vref-AD_value_min)*2.28;Magnified 100 times
uint16_t CalcEffectiveValue()
{	 
     uint16_t EffectiveValue = 0;
     if(ADC_IMAX > ADC_IMIN)
     {
        EffectiveValue = ((ADC_IMAX-ADC_IMIN)*21-11)/10;
     }
     else
     {
        EffectiveValue = 0;
     }
     return EffectiveValue;
}

void read_ADC_value()
{
	static unsigned char index=0;
        static uint32_t curr_sum = 0;
	// Wait convert finished
	while(ADC1_GetFlagStatus(ADC1_FLAG_EOC) == RESET);
	// Get value
	uint16_t adc_value = ADC1_GetConversionValue();
	// Clear flag
	ADC1_ClearFlag(ADC1_FLAG_EOC);
        if(index == 0)
        {
                ADC_IMIN = adc_value;
                ADC_IMAX = adc_value;
                index++;
        }	
	else if(index<COLLECT_SAMPLE_NUM)
	{
                if(adc_value < ADC_IMIN)
                {
                  ADC_IMIN = adc_value;
                }
                else if(adc_value > ADC_IMAX)
                {
                  ADC_IMAX = adc_value;
                }
		index++;
	}
	else
	{       
                if(m_collectTick >= COLLECT_INTERVAL)
                { // almost 1s
                  m_collectTick = 0;
                  uint16_t current = CalcEffectiveValue();
                  if(curr_mvPtr == 0)
                  {
                    curr_sum = 0;
                  }                  
                  curr_mvData[curr_mvPtr] = current;
                  curr_mvPtr = (curr_mvPtr+1)%CURRENT_WIN;
                  curr_sum += current;
                  if(curr_mvPtr == CURRENT_WIN-1)
                  {
                    eqIndex = (eqIndex+1)%1000;
                    last_curr_sum = curr_sum;
                  }
                }
		index=0;
	}
	// Start next conversion
	ADC1_StartConversion();
}

uint16_t GetCurrent()
{
  if(curr_mvPtr == 0)
  {
    return curr_mvData[CURRENT_WIN-1];
  }
  else
  {
    return curr_mvData[curr_mvPtr-1];
  }
}

uint16_t GetMinuteEQ(uint16_t* index)
{  
  *index = eqIndex;
  return last_curr_sum/CURRENT_WIN*220/60;
}

/**
  * @brief Timer1 Update/Overflow/Trigger/Break Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
	TIM1_ClearITPendingBit(TIM1_IT_UPDATE);
	read_ADC_value();
        m_collectTick++;
}

