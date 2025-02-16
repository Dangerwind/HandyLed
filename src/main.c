#include "stm8s.h"
#include "stm8s_flash.h"
#include "stm8s_spi.h"
#include "stm8s_adc1.h"
	
uint8_t LightMode;
uint8_t Old_LightMode;
@eeprom uint8_t EEPROM_data @0x4010;

void eeprom_RW(uint8_t LightMode_RW) 
{		
	FLASH_Unlock(FLASH_MEMTYPE_DATA); //Разблокировка EEPROM
	FLASH_EraseByte(0x4010);
	FLASH_ProgramByte(0x4010, LightMode_RW);    
	FLASH_Lock(FLASH_MEMTYPE_DATA);	
}

@far @interrupt void tim1UpdateInterrupt(void)
{
        TIM1_ClearITPendingBit(TIM1_IT_UPDATE);	
}	
	
uint8_t az=1;
uint8_t Colors[24*6];
uint8_t position=0;
uint32_t NewModeCounter=0;
uint8_t begunok;

uint8_t Modes[10][3]= {
        0,	0,	0,  // еле светит привыключении
        0,	128,0,  //красный
        25, 100,0,  // о
        40,	100,0,	//ж 
        128,0,	0, //з
        30,	30,	70, // г
        0,	0,	115, // с
        0,	80,60,  // ф
        40,	60,	40,  // б
        150,220,150  // я б
};

uint8_t ColR=5;
uint8_t ColG=0;
uint8_t ColB=0;

int8_t DelR=1;
int8_t DelG=0;
int8_t DelB=0;

uint8_t ColZ=20;
int8_t DelZ=0; 

uint32_t resultat;
uint32_t Battary;

uint32_t PauzaCounter;

uint8_t OffFlag=0;

float ChargeLed=0;
float ChargeDel=0.5;
uint8_t ChargeStatus=0;

uint8_t OffModeCounter;

uint32_t aaasss;

void ResetLed(void) {
	uint32_t azz=0;
	while(azz<12000) {
		azz++;
	}
}



void MakeColor(uint8_t GREEN, uint8_t RED, uint8_t BLUE, uint8_t LED) {
	 uint8_t cik;
        begunok=LED*24;
        for (cik=0; cik<=7; cik++) {
                Colors[begunok++]=0x08 * ((GREEN>>(7-cik))&0x01);
        }		
        for (cik=0; cik<=7; cik++) { 
                Colors[begunok++]=0x08 * ((RED>>(7-cik))&0x01);
        }	 	
        for (cik=0; cik<=7; cik++) {
                Colors[begunok++]=0x08 * ((BLUE>>(7-cik))&0x01);
        }		
}
	
main() {	 
        CLK_DeInit();
                
        CLK_HSECmd(DISABLE);
        CLK_LSICmd(DISABLE);
        CLK_HSICmd(ENABLE);

        CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
        CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
	
        GPIO_Init( GPIOC, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_FAST); // на LED ленту
	GPIO_Init(GPIOA,GPIO_PIN_1, GPIO_MODE_IN_PU_NO_IT); // вход геркона
	GPIO_ExternalPullUpConfig(GPIOA,GPIO_PIN_1,ENABLE); 
	GPIO_Init(GPIOA,GPIO_PIN_3, GPIO_MODE_IN_PU_NO_IT); // вход с зарядника
	GPIO_ExternalPullUpConfig(GPIOA,GPIO_PIN_1,ENABLE); 
	GPIO_Init(GPIOA,GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_SLOW); // питание велючить
	
	ADC1_DeInit();

	//Настраиваем порт для ADC вывод GPIOB pin3.
	GPIO_Init( GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);
	//Настраиваем сам ADC.
	ADC1_Init( ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_6, ADC1_PRESSEL_FCPU_D6, ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL3, DISABLE);

        LightMode=1;
        
        LightMode=FLASH_ReadByte(0x4010);

        if (LightMode>12) LightMode=1;
        if (LightMode==0) LightMode=1;
        
        Old_LightMode=LightMode;
	
	for (az=0; az<=(24*6); az++) {
		Colors[az]=0x00;
	}
						
        for (aaasss=0; aaasss<=2000; aaasss++) {
                Colors[0]=0x00;
        }
	
	OffModeCounter=0;
	
	OffFlag=0;
	ChargeStatus=0; // для зарядки типа разряжен
	
	if (GPIO_ReadInputPin(GPIOA,GPIO_PIN_1)==0)  GPIO_WriteHigh(GPIOA,GPIO_PIN_2);  // питание включить если его включили магнитом
	
	if (GPIO_ReadInputPin(GPIOA,GPIO_PIN_3)==0) { // если поставили на зарядник
                for (az=0; az<=250; az++) {
                        Colors[0]=0x00;
                }
	
		if (GPIO_ReadInputPin(GPIOA,GPIO_PIN_3)==0) {
			LightMode=0xFF;
			GPIO_WriteHigh(GPIOA,GPIO_PIN_2); // питание включить
		}
	}

	GPIOC->ODR &= (uint8_t)(~GPIO_PIN_3); // управление светодиодами - скинуть линию
	ResetLed();

        while(1) {
                az=0;
                while(az<(24*6)) {
                        GPIOC->ODR =0x08; 
                        GPIOC->ODR =Colors[az]; 
                        GPIOC->ODR =Colors[az]; 
                        az++;
                        GPIOC->ODR =0x00; 
                }
			 
                if (GPIO_ReadInputPin(GPIOA,GPIO_PIN_3)==0) { // если поставили на зарядник
                        LightMode=0xFF;
                        GPIO_WriteHigh(GPIOA,GPIO_PIN_2); // питание включить
                }
	
                if (LightMode==0xFF) {  // заряд батареи
                        ADC1_ClearFlag( ADC1_FLAG_EOC);
                        ADC1_StartConversion();
                        while( !ADC1_GetFlagStatus( ADC1_FLAG_EOC));
                        resultat=ADC1_GetConversionValue();
                        
                        ADC1_ClearFlag( ADC1_FLAG_EOC);
                        ADC1_StartConversion(); 
                        while( !ADC1_GetFlagStatus( ADC1_FLAG_EOC));
                        resultat=resultat+ADC1_GetConversionValue();
                        
                        ADC1_ClearFlag( ADC1_FLAG_EOC);
                        ADC1_StartConversion();
                        while( !ADC1_GetFlagStatus( ADC1_FLAG_EOC));
                        resultat=resultat+ADC1_GetConversionValue();
                        
                        Battary=resultat/3;
                        
                        if ((Battary<=515)&(ChargeStatus==0)) ChargeStatus=1;
                        if ((Battary<=458)&(ChargeStatus==1)) ChargeStatus=2;
                        if ((Battary<=450)&(ChargeStatus==2)) ChargeStatus=3;
                        
                        if ((Battary>=525)&(ChargeStatus==1)) ChargeStatus=0;
                        if ((Battary>=462)&(ChargeStatus==2)) ChargeStatus=1;
                        if ((Battary>=460)&(ChargeStatus==3)) ChargeStatus=2;
                        
                        ChargeLed=ChargeLed+ChargeDel;
                        if (ChargeLed>=10) ChargeDel=-1;
                        if (ChargeLed<=1) ChargeDel=1;
                        
                        
                        if ((ChargeStatus==0)&(ChargeLed>0)) {
                                MakeColor(0x0,0x0,0,0);
                                MakeColor(0x0,0x0,0,1);
                                MakeColor(0x0,0x0,0,2);
                                MakeColor(0x0,0x0,0,3);
                                MakeColor(0x0,ChargeLed,0,4);
                                MakeColor(0x0,ChargeLed,0,5);
                        }
                        if ((ChargeStatus==1)&(ChargeLed>0)) {
                                MakeColor(ChargeLed/2+1,ChargeLed/2+1,0,0);
                                MakeColor(0x0,0x0,0,1);
                                MakeColor(0x0,0x0,0,2);
                                MakeColor(ChargeLed/2+1,ChargeLed/2+1,0,3);
                                MakeColor(ChargeLed/2+1,ChargeLed/2+1,0,4);
                                MakeColor(ChargeLed/2+1,ChargeLed/2+1,0,5);
                        }
                        if ((ChargeStatus==2)&(ChargeLed>0)) {
                                MakeColor(ChargeLed,0,0,0);
                                MakeColor(ChargeLed,0,0,1);
                                MakeColor(ChargeLed,0,0,2);
                                MakeColor(ChargeLed,0,0,3);
                                MakeColor(ChargeLed,0,0,4);
                                MakeColor(ChargeLed,0,0,5);
                        }		
                        if (ChargeStatus==3) {
                                MakeColor(2,0,0,0);
                                MakeColor(2,0,0,1);
                                MakeColor(2,0,0,2);
                                MakeColor(2,0,0,3);
                                MakeColor(2,0,0,4);
                                MakeColor(2,0,0,5);
                        }	
                        if ((GPIO_ReadInputPin(GPIOA,GPIO_PIN_3)!=0)&(ChargeLed<=1)) { // если сняли с зарядник
                                GPIO_WriteLow(GPIOA,GPIO_PIN_2); // питание выключить
                        }			
                        
                }
	
                if ((LightMode<=9)) {
                        for (az=0; az<=5; az++) {
                                MakeColor(Modes[LightMode][0],Modes[LightMode][1],Modes[LightMode][2],az);
                        }
                        ColG=3; ColR=3; ColB=3;
                        DelG=0; DelR=0; DelB=3;
                }
	
	
                if (LightMode==10) {  // пульсирующий белый
                        ColZ=ColZ+DelZ;
                        
                        if (ColZ<=28) DelZ=2;
                        if (ColZ>=128) { DelZ=-2; }

                        for (az=0; az<=5; az++) {
                                MakeColor((uint8_t)ColZ,(uint8_t)ColZ,(uint8_t)ColZ,az);
                        }
                        
                        if (DelZ==0) DelZ=1;
                }
	
	        if (LightMode==11) {
			ColG=ColG+DelG;
			ColR=ColR+DelR;
			ColB=ColB+DelB;
			
			if (ColR<=2) DelR=1;
			if (ColG<=2) DelG=1;
			if (ColB<=2) DelB=1;
		
		        if (ColR>=250) { DelR=-2; DelG=1; }
			if (ColG>=250) { DelG=-2; DelB=1; }
			if (ColB>=250) { DelB=-2; DelR=1; }
			for (az=0; az<=5; az++) {
				MakeColor((uint8_t)ColG,(uint8_t)ColR,(uint8_t)ColB,az);
			}
			
			if ((DelR==0)&(DelG==0)&(DelB==0)) DelR=1;
	        }
	
                if (LightMode==12) {
                        MakeColor(128,0,0,0);
                        MakeColor(128,0,0,3);
                        MakeColor(0,128,0,2);
                        MakeColor(0,128,0,1);
                        MakeColor(0,0,128,4);
                        MakeColor(0,0,128,5);
                }
	
                PauzaCounter=0;
                while(PauzaCounter<=5000) {
                        PauzaCounter++;
                }

                if ((GPIO_ReadInputPin(GPIOA,GPIO_PIN_1)==0)&(LightMode<0xE0)&(OffFlag==0)) NewModeCounter++;
                else { NewModeCounter=0; OffFlag=1; }

                if ((GPIO_ReadInputPin(GPIOA,GPIO_PIN_1)==0)&(LightMode<0xE0)&(OffFlag==1)) {
		// запись по флэш тут
                        if ((Old_LightMode!=LightMode)&(LightMode<0xE0)&(LightMode>0))
                        { 	
                                // LightMode=6;
                                                eeprom_RW(LightMode); 
                                                Old_LightMode=LightMode; 
                        }
	
	                LightMode=0; // выклоючить все
                        MakeColor(0,0,0,0);
                        MakeColor(0,0,0,1);
                        MakeColor(0,0,0,2);
                        MakeColor(0,0,0,3);
                        MakeColor(0,0,0,4);
                        MakeColor(0,0,0,5);
	
	                GPIO_WriteLow(GPIOA,GPIO_PIN_2);
	
	                OffModeCounter++;
	                if (OffModeCounter>=200) LightMode=0xEF;
                }


                if (LightMode==0xEF)  // заряд батареи
                {
			ADC1_ClearFlag( ADC1_FLAG_EOC);
			ADC1_StartConversion();
			while( !ADC1_GetFlagStatus( ADC1_FLAG_EOC));
			resultat=ADC1_GetConversionValue();
			
			ADC1_ClearFlag( ADC1_FLAG_EOC);
			ADC1_StartConversion(); 
			while( !ADC1_GetFlagStatus( ADC1_FLAG_EOC));
			resultat=resultat+ADC1_GetConversionValue();
			
			ADC1_ClearFlag( ADC1_FLAG_EOC);
			ADC1_StartConversion();
			while( !ADC1_GetFlagStatus( ADC1_FLAG_EOC));
			resultat=resultat+ADC1_GetConversionValue();
			
			Battary=resultat/3;
			
			if ((Battary<=546)&(ChargeStatus==0)) ChargeStatus=1;
			if ((Battary<=498)&(ChargeStatus==1)) ChargeStatus=2;
			
			if ((Battary>=552)&(ChargeStatus==1)) ChargeStatus=0;
			if ((Battary>=504)&(ChargeStatus==2)) ChargeStatus=1;
			
			ChargeLed=ChargeLed+ChargeDel;
			if (ChargeLed>=20) ChargeDel=-0.5;
			if (ChargeLed<=1) ChargeDel=0.5;
			
			if (ChargeStatus==0) {
                                MakeColor(0x0,0x0,0,0);
                                MakeColor(0x0,0x0,0,3);
                                MakeColor(0x0,0x0,0,4);
                                MakeColor(0x0,0x0,0,5);
                                MakeColor(0x0,ChargeLed,0,1);
                                MakeColor(0x0,ChargeLed,0,2);
			}
			if (ChargeStatus==1) {
                                MakeColor(ChargeLed/2,ChargeLed/2,0,0);
                                MakeColor(0x0,0x0,0,4);
                                MakeColor(0x0,0x0,0,5);
                                MakeColor(ChargeLed/2,ChargeLed/2,0,1);
                                MakeColor(ChargeLed/2,ChargeLed/2,0,2);
                                MakeColor(ChargeLed/2,ChargeLed/2,0,3);
			}
			if (ChargeStatus==2) {
                                MakeColor(ChargeLed,0,0,0);
                                MakeColor(ChargeLed,0,0,1);
                                MakeColor(ChargeLed,0,0,2);
                                MakeColor(ChargeLed,0,0,3);
                                MakeColor(ChargeLed,0,0,4);
                                MakeColor(ChargeLed,0,0,5);
			}		
			if (ChargeStatus==3) {
                                MakeColor(10,0,0,0);
                                MakeColor(10,0,0,1);
                                MakeColor(10,0,0,2);
                                MakeColor(10,0,0,3);
                                MakeColor(10,0,0,4);
                                MakeColor(10,0,0,5);
			}	
	        }

                if ((NewModeCounter>250)&(OffFlag==0))  {
                        LightMode++;
                        if (LightMode>=13) LightMode=1;
                        NewModeCounter=200;
                }
        }

	while (1);
}
