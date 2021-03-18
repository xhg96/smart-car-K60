#include "common.h"
#include "include.h"
#include "control.h"
#include "motor.h"
#include "m_Motro.h"
                                /*�����ʾͼ����ʾ����������ӱ���*/
// ����EEPROM�Ŀ�ʼ��ַ
int EEPROM_ADDR = 0x14000000;

// ����LCD��ҳ��ʾ��Ŀ��
#define PAGE_DISP_NUM 10

/*
 *	�ⲿ����
 */

extern uint16 param0;           //ͼ������ ��
extern uint16 obOffset;
extern uint16 obB;
extern uint16 rampDelayDown;
extern uint16 rampDelayUp;
extern uint16 RuningTime;            //���ж�ʱ(��ʱģʽ��Ч)
extern RUNMode_e RunMode;            //����ģʽ��־
extern uint16 ConstantSpeed;
extern uint8 *imgbuff;          //ͼ��
extern uint16 lineAmend,speedLineK,speedLineB;
extern uint16 picTotalNumSet;         //��ͼ����������LCD�е���
extern uint16 skipNumSet;                //����ѭ����
extern uint16 savePicSwitch;              //��ͼ�񿪹�
extern uint16 brake_test;
extern uint16 ki;
/*
 *	ȫ�ֱ���
 */
volatile KEY_MSG_t keymsg = {KEY_U,KEY_UP};  //������Ϣ�ṹ��

uint8 ExitMenu_flag = 0;

uint16 *EEPROM_data[] = { &CP.k,        //������ֲ���
                          &CP.b,
                          &CP.left_k,
                          &CP.right_k,
                          &Servo_PID.Kdin,  
                          &Servo_PID.Kdout,
                          &taw,
                          
                          &Motro_PID.Kp,        //���PID����
                          &ki,
                          &Motro_PID.Kd,
                          &encoderThreshold,
                          
                          &change_ki,
                          &change_kib,
                          &ov7725_cnst_set,     //��ֵ
                          
                          &rampDelayDown ,
                          &rampDelayUp,
                          
                          &RuningTime,          //��ʱ���в���
                          &ConstantSpeed,
                                               //�ٶȿ��Ʋ���
                          &picTotalNumSet,      //��ͼ�����
                          &skipNumSet,
                          &savePicSwitch,
                          
                          &param0,
                          &obOffset,
                          &obB,
                          
                          &Ann_k,               //Բ�����
                          &annulus_speed,    
                          &ann_speed_k,
                          &ann_min_speed,
                          
                          &max_speed,
                          &max2_speed,
                          &wan_speed,
                          &min_speed,
                          &base_speed,
                          &outm_speed,
                          &add_speed,
                          &ra_speed,
                          &obcutsp,
                          &speed_k,
                          &speed_k2,
                          &extraStopVal_Long,
                          &extraStopVal_Short,
                          &extraStopVal_Turnin,
                          &bump_speed,
                          &turnILimitsub,
                          
                          &bra_speed1,
                          &bra_speed2,
                          &bra_speed3,
                          &bra_speed4,
                                          //�����жϲ���
                          &zhidao_ade,
                          &wan_ade,
                          &dzhi_top,
                          &set_othertop,
                          &set_cothertop,
                          &wanru_ade,
                          &lineAmend,
                          &speedLineK,
                          &speedLineB,
                          &set_yh_top,
                          
                          &MID_PWM,     //�����ֵ
                          
                          &bumpEncoderThreshold,
                         };


/*
 *	�˵�ִ�к�������
 */
void Read_EEPROM();
void Write_EEPROM();
void Show_ResetReason();

void Menu_Null();

void Menu_Servo();  
void ServoMidCNV_Debug();
void ServoMax_Debug();
void ServoMin_Debug();
void Menu_Motro();
void Menu_Speed();
void Menu_Speed_Detect();
void Menu_Img();
void Menu_Annuulus();
void Menu_SaveImg();
void Menu_ov7725_cnst_set();

void Menu_SelectMode();
void Normal_Run();
void Constant_Run();
void Timing_Run();
void Quadra_Run();

void Menu_SetRampDelayDown();
void Menu_SetRampDelayUp();
void Menu_UploadData();
void Upload_Speed();
void Menu_Read_Flash();
void Menu_Write_Flash();
void Read_Flash1();
void Read_Flash2();
void Read_Flash3();
void Read_WenRuLaoGou();
void Read_XuHuanYiQiang();

void Write_Flash1();
void Write_Flash2();
void Write_Flash3();
void Write_WenRuLaoGou();
void Write_XuHuanYiQiang();
//----------------------------------   ���˵�   -------------------------------

// һ���˵�
MENU_PRMT MainMenu_Prmt;

MENU_TABLE MainMenu_Table[] = 
{
	
	
	{"1.ServoDebug        ", Menu_Servo, NULL},
		
	{"2.MotroDebug        ", Menu_Motro, NULL},

	{"3.ImgDebug          ", Menu_Img, NULL},
		
	{"4.RunMode           ", Menu_SelectMode, NULL},

	{"5.SpeedParam        ", Menu_Speed, NULL}, 
        
        {"6.SpeedJudgeParam   ", Menu_Speed_Detect, NULL}, 
        
        {"7.Annulus           ", Menu_Annuulus, NULL}, 
        
        {"8.ResetReason       ", Show_ResetReason, NULL}, 
        
        {"0. ReadFlash         ", Menu_Read_Flash, NULL},
        {"-1.WriteFlash        ", Menu_Write_Flash,NULL},
	//{"8.UploadData        ", Menu_UploadData, NULL},
};

//---------------------------------   �����˵�  -------------------------------

// �����˵�1  �����������
MENU_PRMT Servo_Prmt;

MENU_TABLE Servo_MenuTable[] = 
{
	{"1.k  * 10     ", Menu_Null, &CP.k},
	{"2.b  * 100    ", Menu_Null, &CP.b},
	{"3.left_k * 10 ", Menu_Null, &CP.left_k},
        {"4.right_k * 10", Menu_Null, &CP.right_k},
        {"5.MID_PWM    ", ServoMidCNV_Debug, &MID_PWM},
        {"5.Kdin       ", Menu_Null, &Servo_PID.Kdin},
        {"6.Kdout      ", Menu_Null, &Servo_PID.Kdout},
        {"6.taw        ", Menu_Null, &taw},
         
        {"7.SER_MAX    ", ServoMax_Debug, &SERVOPWM_MAX},
        {"8.SER_MIN    ", ServoMin_Debug, &SERVOPWM_MIN},
        
        
     

};

// �����˵�2  �����������
MENU_PRMT Motro_Prmt;

MENU_TABLE Motro_MenuTable[] = 
{
	{"1.MOTRO_KP          ", Menu_Null, &Motro_PID.Kp},

	{"2.brake_ki          ", Menu_Null, &ki},

	{"3.MOTRO_KD          ", Menu_Null, &Motro_PID.Kd},
        
        {"4.encoderLimit      ", Menu_Null, &encoderThreshold},
        {"5.bumpEnLimit      ", Menu_Null, &bumpEncoderThreshold},
        
        
        {"5.change_ki         ", Menu_Null, &change_ki},
        
        {"6.change_kib        ", Menu_Null, &change_kib},
        
        {"7.turnILimitsub      ", Menu_Null, &turnILimitsub},
        
        {"8.rampDelayDown      ", Menu_SetRampDelayDown, &rampDelayDown},
          
        {"9.rampDelayUp        ", Menu_SetRampDelayUp, &rampDelayUp},
};

// �����˵�3  ͼ���������
MENU_PRMT Img_Prmt;

MENU_TABLE Img_MenuTable[] =
{
      {"1.param0      ", Menu_Null, &param0},
      {"2.obOffset    ", Menu_Null, &obOffset},
      {"3.obB         ", Menu_Null, &obB},
      {"4.cnst        ", Menu_ov7725_cnst_set, &ov7725_cnst_set},
};



// �����˵�4  С��ģʽѡ��
MENU_PRMT RunMode_Prmt;

MENU_TABLE RunMode_MenuTable[] = 
{
		
	{"1.Normal            ", Normal_Run, NULL},

	{"2.TimingConSpeed    ", Constant_Run, &ConstantSpeed},
    
        {"3.Timing            ", Timing_Run, &RuningTime},
        
        {"4.QruadraSpeed      ", Quadra_Run, &RuningTime},
        {"5.brake_test        ", Menu_Null, &brake_test},
        

};

// �����˵�5  �ٶȿ��Ʋ�������
MENU_PRMT Speed_Prmt;

MENU_TABLE Speed_MenuTable[] = 
{
	{"1.max_speed       ", Menu_Null, &max_speed},
        {"2.wan_speed       ", Menu_Null, &wan_speed},
        {"3.min_speed       ", Menu_Null, &min_speed},
        {"4.base_speed      ", Menu_Null, &base_speed},
        {"5.outm_speed      ", Menu_Null, &outm_speed},
        {"6.add_speed       ", Menu_Null, &add_speed},
        {"7.ra_speed        ", Menu_Null, &ra_speed},
        {"8.obcutsp         ", Menu_Null, &obcutsp},
        {"9.max2_speed      ", Menu_Null, &max2_speed},
        {"10.speed_k        ", Menu_Null, &speed_k},
        {"10.speed_k2       ", Menu_Null, &speed_k2},
        {"11.ex_Turn        ", Menu_Null, &extraStopVal_Turnin},
        {"12.ex_Short       ", Menu_Null, &extraStopVal_Short},
        {"13.ex_Long        ", Menu_Null, &extraStopVal_Long},
        {"14.bump_speed     ", Menu_Null, &bump_speed},
        {"15.bra_speed1    ", Menu_Null, &bra_speed1},
        {"16.bra_speed2    ", Menu_Null, &bra_speed2},
        {"17.bra_speed3    ", Menu_Null, &bra_speed3},
        {"18.bra_speed4    ", Menu_Null, &bra_speed4},
                                  
};

// �����˵�  �ٶ��жϲ�������
MENU_PRMT Speed_Detect;

MENU_TABLE Speed_Detect_MenuTable[] = 
{
	{"1.zhidao_ade       ", Menu_Null, &zhidao_ade},
        {"2.wan_ade          ", Menu_Null, &wan_ade},
        {"3.dzhi_top         ", Menu_Null, &dzhi_top},
        {"4.set_othertop     ", Menu_Null, &set_othertop},
        {"5.set_cothertop    ", Menu_Null, &set_cothertop},
        {"6.wanru_ade        ", Menu_Null, &wanru_ade},
        {"7.lineAmend        ", Menu_Null, &lineAmend},
        {"8.speedLineK       ", Menu_Null, &speedLineK},
        {"9.speedLineB       ", Menu_Null, &speedLineB},
        {"10.yh_top          ", Menu_Null, &set_yh_top},

};


MENU_PRMT Annuulus_About;

MENU_TABLE Annuulus_About_MenuTable[] = 
{
	{"1.Ann_Servo_k      ", Menu_Null, &Ann_k},
        {"2.Ann_Speed        ", Menu_Null, &annulus_speed},
        {"3.Ann_SpK          ", Menu_Null, &ann_speed_k},
        {"4.Ann_Sp_min       ", Menu_Null, &ann_min_speed},
     
};

// �����˵�  ѡȡFlash����
MENU_PRMT Read_Flash_Prmt;

MENU_TABLE Read_Flash_MenuTable[] = 
{
	{"1.FlashSectorXHG  ", Read_Flash1, NULL}, 
        {"2.FlashSectorCZH  ", Read_Flash2, NULL}, 
        {"3.FlashSectorLC   ", Read_Flash3, NULL},
        {"4.WenRuLaoGou     ", Read_WenRuLaoGou, NULL},
        {"5.XuHuanYiQiang   ", Read_XuHuanYiQiang, NULL},
        
          
        
};


MENU_PRMT Write_Flash_Prmt;

MENU_TABLE Write_Flash_MenuTable[] = 
{
	{"1.FlashSectorXHG  ", Write_Flash1, NULL}, 
        {"2.FlashSectorCZH  ", Write_Flash2, NULL}, 
        {"3.FlashSectorLC   ", Write_Flash3, NULL}, 
        {"4.WenRuLaoGou     ", Write_WenRuLaoGou, NULL}, 
        {"5.XuHuanYiQiang   ", Write_XuHuanYiQiang, NULL}, 
        
};
// �����˵�6  �ϴ�����ѡ��
MENU_PRMT UploadData_Prmt;

MENU_TABLE Upload_MenuTable[] = 
{
	{"1.SpeedWave         ", Upload_Speed, NULL}, 
};



MENU_PRMT SaveImg_Prmt;

MENU_TABLE SaveImg_MenuTable[] = 
{
	{"1.ImgNum          ", Menu_Null, &picTotalNumSet}, 
        {"2.SkipNum         ", Menu_Null, &skipNumSet}, 
        {"3.SaveSwitch      ", Menu_Null, &savePicSwitch}, 

};

/*!
*  @brief      PORTA�жϷ�����(�����½����ж�)
*/
void PORTA_IRQHandler(void)
{
    uint32 flag;
    
    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR = ~0;    //���жϱ�־λ
    
    if(flag & (1 << 7))
    {
      DELAY_MS(10);    //������ʱ����       
		// ����1����
      if ( !PTA7_IN )
      {

        keymsg.status = KEY_DOWN;
        keymsg.key = KEY_STOP;
      }
    }
    else if(flag & (1 << 6))
    {
      DELAY_MS(10);    //������ʱ����
        
		//����2����
      if( !PTA6_IN )
      {   
          keymsg.status = KEY_DOWN;
          keymsg.key = KEY_START;
        
        //ExitMenu_flag = 1;
      }
    }
}

/******************************************************************************
* FunctionName   : KeySan()
* Description    : ������ȡ
* EntryParameter : None
* ReturnValue    : ����ֵ
*******************************************************************************/
KEY_e KeySan(void)
{
    while(keymsg.status == KEY_UP);    //�ȴ���������
    
    keymsg.status = KEY_UP;
    
	//��������50ms(����������)
	//BEEP_ON;
	//DELAY_MS(30);
	//BEEP_OFF;
	
    return keymsg.key;
}

/******************************************************************************
*  @brief  ������������ͨ�ú���
******************************************************************************/
void adjustParam(Site_t site, uint16 *param, uint8 max_param_bit, uint16 Color, uint16 bkColor)
{
	do{
        
		KeySan();

	    switch(keymsg.key)
	    {
	        case(KEY_U):
	            (*param)++;
	            LCD_num_BC(site, (uint32)(*param), max_param_bit, Color, bkColor);
	            break;
	                    
	        case(KEY_D):
	            (*param)--;
	            LCD_num_BC(site, (uint32)(*param), max_param_bit, Color, bkColor);
	            break;
	                    
	        case(KEY_L):
	            (*param) -= 10;
	            LCD_num_BC(site, (uint32)(*param), max_param_bit, Color, bkColor);
	            break;
	                    
	        case(KEY_R):
	            (*param) += 10;
	            LCD_num_BC(site, (uint32)(*param), max_param_bit, Color, bkColor);
	            break;
	                    
	        default:
	            break;
	    }
		if(*param <= 0)  *param = 0; 
                if(*param >= 65535) *param = 0;
    }while(keymsg.key != KEY_B);	
}

void adjustParam_Big(Site_t site, uint16 *param, uint8 max_param_bit, uint16 Color, uint16 bkColor)
{
	do{
        
		KeySan();

	    switch(keymsg.key)
	    {
	        case(KEY_U):
	            (*param)+=100;
	            LCD_num_BC(site, (uint32)( (*param)/100 ), max_param_bit, Color, bkColor);
	            break;
	                    
	        case(KEY_D):
	            (*param)-=100;
	            LCD_num_BC(site, (uint32)( (*param)/100 ), max_param_bit, Color, bkColor);
	            break;
	                    
	        case(KEY_L):
	            (*param) -= 500;
	            LCD_num_BC(site, (uint32)( (*param)/100 ), max_param_bit, Color, bkColor);
	            break;
	                    
	        case(KEY_R):
	            (*param) += 500;
	            LCD_num_BC(site, (uint32)( (*param)/100 ), max_param_bit, Color, bkColor);
	            break;
	                    
	        default:
	            break;
	    }
		if(*param <= 0)  *param = 0; 
            if(*param >= 65535) *param = 0;
    }while(keymsg.key != KEY_B);	
}

/******************************************************************************
* FunctionName   : Write_EEPROM()
* Description    : ������д��EEPROM����
* EntryParameter : None
* ReturnValue    : None
*******************************************************************************/
void Write_EEPROM()
{	
	uint32 write_addr = EEPROM_ADDR;

	//����EEPROM�洢�������������ɳ���������EEPROM�Ĵ�С(128B/2=64)
	uint8 EEPROM_DataNum = sizeof(EEPROM_data)/sizeof(EEPROM_data[0]);  
	
	//����д��EEPROM����
	for(uint8 i = 0; i < EEPROM_DataNum; i++)
	{
		/* Make sure the EEE is ready. If not wait for the command to complete */
    	while(!(FTFE_FCNFG & FTFE_FCNFG_EEERDY_MASK));
    
    	*((uint16 *)(write_addr)) = *EEPROM_data[i];

		/* Make sure the EEE is ready. If not wait for the command to complete */
    	while(!(FTFE_FCNFG & FTFE_FCNFG_EEERDY_MASK));

		write_addr += 0x02;
	}
	
    LCD_clear(WHITE);
    mLCD_str(0, 56, "Write EEPROM OK!", RED,WHITE);
    DELAY_MS(500);
}

/******************************************************************************
* FunctionName   : Read_EEPROM()
* Description    : ��EEPROM�ж�ȡ����
* EntryParameter : None
* ReturnValue    : None
*******************************************************************************/

void Read_EEPROM()
{
	uint32 read_addr = EEPROM_ADDR;
    
    //����EEPROM�洢�������������ɳ���������EEPROM�Ĵ�С(128B/2=64)
	uint8 EEPROM_DataNum = sizeof(EEPROM_data)/sizeof(EEPROM_data[0]);
		
	for(uint8 i = 0; i < EEPROM_DataNum; i++)
	{
		/* Make sure the EEE is ready. If not wait for the command to complete */
    	while(!(FTFE_FCNFG & FTFE_FCNFG_EEERDY_MASK));
		
		*EEPROM_data[i] = *((uint16 *)(read_addr));

		/* Make sure the EEE is ready. If not wait for the command to complete */
    	while(!(FTFE_FCNFG & FTFE_FCNFG_EEERDY_MASK));

		read_addr += 0x02;
	}
	
        //FTM1_C1V = ServoMidPWM_CNV;  //�����ֵ
	
	LCD_clear(WHITE);
	mLCD_str(0,56,"Read EEPROM OK!",RED,WHITE);
	DELAY_MS(200);
}

/****************************************************************************** 
 *  ��ȡRCM�Ĵ�����ȡ��Ƭ����λԭ��
*******************************************************************************/

void Show_ResetReason()
{
	LCD_clear(WHITE);
	
    if (RCM_SRS0 & RCM_SRS0_POR_MASK)     mLCD_str(0,20,"Power-on reset",RED,WHITE);
    if (RCM_SRS0 & RCM_SRS0_LVD_MASK)     mLCD_str(0,20,"Low-voltage detect",RED,WHITE);
    if (RCM_SRS0 & RCM_SRS0_PIN_MASK)     mLCD_str(0,20,"External reset pin",RED,WHITE);
    if (RCM_SRS0 & RCM_SRS0_WDOG_MASK)    mLCD_str(0,20,"Watchdog timeout",RED,WHITE);
    if (RCM_SRS0 & RCM_SRS0_LOC_MASK)     mLCD_str(0,20,"A loss of external clock",RED,WHITE);
    if (RCM_SRS0 & RCM_SRS0_WAKEUP_MASK)  mLCD_str(0,20,"Low leakage wakeup reset",RED,WHITE);

    if (RCM_SRS1 & RCM_SRS1_SACKERR_MASK) mLCD_str(0,20,"Peripheral failure to acknowledge attempt to enter stop mode",RED,WHITE);
    if (RCM_SRS1 & RCM_SRS1_EZPT_MASK)    mLCD_str(0,20,"EzPort receiving the RESET command while the device is in EzPort mode",RED,WHITE);
    if (RCM_SRS1 & RCM_SRS1_MDM_AP_MASK)  mLCD_str(0,20,"Host debugger system setting of the System Reset Request bit",RED,WHITE);
    if (RCM_SRS1 & RCM_SRS1_SW_MASK)      mLCD_str(0,20,"Software setting of SYSRESETREQ bit",RED,WHITE);
    if (RCM_SRS1 & RCM_SRS1_LOCKUP_MASK)  mLCD_str(0,20,"Core LOCKUP event",RED,WHITE);
    if (RCM_SRS1 & RCM_SRS1_JTAG_MASK)    mLCD_str(0,20,"JTAG",RED,WHITE);

	DELAY_MS(100);
    while(keymsg.status == KEY_UP);
	LCD_clear(WHITE);
}

void Menu_Null()
{
    DELAY_MS(100);
}

/******************************************************************************
* FunctionName   : Menu_PrmtInit()
* Description    : ��ʼ���˵�����
* EntryParameter : prmt - �˵�����, num - ÿҳ��ʾ����, page - �����ʾҳ��
* ReturnValue    : None
*******************************************************************************/
void Menu_PrmtInit(MENU_PRMT *prmt, uint8 num, uint8 page)
{
    prmt->ExitMark = 0;       //����˳��˵���־
       
    prmt->Cursor   = 0;       //�������
    prmt->PageNo   = 0;       //ҳ����
    prmt->Index    = 0;       //��������
    prmt->DispNum  = num;     //ҳ�����ʾ��Ŀ��

    prmt->MaxPage  = page;    //���ҳ��
}

/******************************************************************************
* FunctionName   : Menu_Display()
* Description    : ��ʾ�˵���
* EntryParameter : menuTable - ��ʾҳ��dispNum - ÿһҳ����ʾ������cursor - ���λ��
* ReturnValue    : None
*******************************************************************************/
void Menu_Display(MENU_TABLE *menuTable, uint8 pageNo, uint8 dispNum, uint8 cursor)
{
    uint8 i;
    Site_t site;

 	for (i=0; i<dispNum; i++)
 	{	
          if (cursor == i)
          {
  			/* ������ʾ��ǰ���ѡ�в˵��� */
  		site.x = 0;    site.y = (i+1)*16;
                LCD_str(site,menuTable[pageNo+i].MenuName, WHITE,BLUE);

			/* ���˲˵�������Ҫ���Ĳ���������ʾ�ò��� */
			if(menuTable[pageNo+i].DebugParam != NULL)
			{
                            site.x = 128;  
                            uint32 num_t = (*(menuTable[pageNo+i].DebugParam));
                            LCD_num(site, num_t, GREEN, BLUE);
			}
          }
          else
          {
  			/* ������ʾ����˵��� */
  		site.x = 0;    site.y = (i+1)*16;
                LCD_str(site,menuTable[pageNo+i].MenuName, BLUE,WHITE); 
			
			/* ���˲˵�������Ҫ���Ĳ���������ʾ�ò��� */
			if(menuTable[pageNo+i].DebugParam != NULL)
			{
                            site.x = 128;
                            uint32 num_t = (*(menuTable[pageNo+i].DebugParam));
                            LCD_num(site, num_t, GREEN, WHITE);
			}
          }
	}
}

/******************************************************************************
* FunctionName   : Menu_Move()
* Description    : �˵��ƶ�
* EntryParameter :  prmt - �˵�����, key - ����ֵ
* ReturnValue    : ��ȷ�Ϸ���0�����򷵻�1
******************************************************************************/
uint8 Menu_Move(MENU_PRMT *prmt, KEY_e key)
{
    uint8 rValue = 1;

    switch (key) 
 	{
    	case KEY_U:                   // ����
      	{ 
          if (prmt->Cursor != 0)              // ��겻�ڶ���
          {
              prmt->Cursor--;                 // �������
          }
          else                                // ����ڶ���
          {
              if (prmt->PageNo != 0)          // ҳ��û�е���С
              {
                  prmt->PageNo--;             // ���Ϸ�
              }
              else
              {
                  prmt->Cursor = prmt->DispNum-1;    // ��굽��
                  prmt->PageNo = prmt->MaxPage-1;    // ���ҳ
              }
          }
          break;
   	}

  	case KEY_D:                   // ����
        {
          if (prmt->Cursor < prmt->DispNum-1)        // ���û�е��ף��ƶ����
          {
              prmt->Cursor++;                        // ��������ƶ�
          }
          else                                       // ��굽��
          {
              if (prmt->PageNo < prmt->MaxPage-1)    // ҳ��û�е��ף�ҳ���ƶ�
              {
                    prmt->PageNo++;                    // �·�һҳ
              }
              else                                   // ҳ��͹�궼���ף����ؿ�ʼҳ
              {
                    prmt->Cursor = 0;
                    prmt->PageNo = 0;
              }
          }
          break;
   	}
        
  	case KEY_B:                   // ȷ��
      	{ 
        	prmt->Index = prmt->Cursor + prmt->PageNo;   //����ִ��������� 
    		rValue = 0;
			
          	break;
  	}
        
  	case KEY_L:                   // ��������ϼ��˵�
      	{
       		//prmt->Cursor = 0;
    		//prmt->PageNo = 0;
    		prmt->ExitMark = 1;
			
    		break;
   	}
  	case KEY_R:                   // �Ҽ������ײ�
      	{
    		prmt->Cursor = prmt->DispNum-1;             // ��굽��
    		prmt->PageNo = prmt->MaxPage-1;             // ���ҳ
    	
			break;
   	}

  	default:break;
        } 
	return rValue;                    // ����ִ������
}

/******************************************************************************
* FunctionName   : Menu_Process()
* Description    : ����˵���
* EntryParameter : menuName - �˵����ƣ�prmt - �˵�������table - �˵�����, num - �˵�����
* ReturnValue    : None
******************************************************************************/
void Menu_Process(uint8 *menuName, MENU_PRMT *prmt, MENU_TABLE *table, uint8 num)
{
	KEY_e key;
	Site_t site;
	uint8 page;    //��ʾ�˵���Ҫ��ҳ��

	if(num - PAGE_DISP_NUM <= 0)
            page = 1;
	else
        {
            page = num - PAGE_DISP_NUM + 1;
            num = PAGE_DISP_NUM;
        }
	
	// ��ʾ������ҳ������
	Menu_PrmtInit(prmt, num, page);   
    
	do
 	{
          mLCD_str(0, 0, menuName, ORANGE_RED, WHITE);    // ��ʾ�˵�����

		// ��ʾ�ָ���
		for(uint8 y=0; y < 176; y++)
		{
                    mLCD_point(160, y, BLACK);
		}

		// ��ʾ�˵���
  		Menu_Display(table, prmt->PageNo, prmt->DispNum, prmt->Cursor);      
        
		key = KeySan();                       // ��ȡ����
		
  		if ( Menu_Move(prmt, key) == 0 )           // �˵��ƶ�������ȷ�ϼ�
  		{
			// �жϴ˲˵���������Ҫ���ڵĲ�������������������
			if(table[prmt->Index].DebugParam != NULL && table[prmt->Index].ItemHook == Menu_Null)
			{
				site.x = 128;	site.y = (1 + prmt->Cursor) * 16;
	
                                LCD_num_BC(site, *(table[prmt->Index].DebugParam), 4, RED, BLUE);

				adjustParam(site, table[prmt->Index].DebugParam, 4, RED, BLUE);
			}
			// ���ǲ������ڵĻ���ִ�в˵�����
			else
			{
				table[prmt->Index].ItemHook();         // ִ����Ӧ��
			}
  		}
	} while (prmt->ExitMark == 0 && ExitMenu_flag == 0);
    
    LCD_clear(WHITE);
	DELAY_MS(50);
}


/******************************************************************************
* FunctionName   : MainMenu_Set()
* Description    : ��������
* EntryParameter : None
* ReturnValue    : None
*******************************************************************************/
void MainMenu_Set(void)
{
    LCD_clear(WHITE);

    //���ж�
    disable_irq(PORTB_IRQn);  
    disable_irq(PIT1_IRQn);              
    disable_irq(PIT0_IRQn);
    
    uint8 menuNum = sizeof(MainMenu_Table)/sizeof(MainMenu_Table[0]);    // �˵�����
		
    Menu_Process(" -= Setting =- ", &MainMenu_Prmt, MainMenu_Table, menuNum);
    
    Write_EEPROM();    //������д��EEPROM����

    LCD_clear(WHITE);
    
    //ʹ���ж�
    enable_irq(PORTB_IRQn);  
    enable_irq(PIT1_IRQn);                
    enable_irq(PIT0_IRQn);
    enable_irq(PORTA_IRQn);
}

/******************************************************************************
* FunctionName   : Menu_Servo()
* Description    : �������
* EntryParameter : None
* ReturnValue    : None
******************************************************************************/
void Menu_Servo(void)
{
	LCD_clear(WHITE); 
	
	uint8 menuNum;

        menuNum = sizeof(Servo_MenuTable)/sizeof(Servo_MenuTable[0]);         // �˵�����

        Menu_Process("-= ServoDebug =-", &Servo_Prmt, Servo_MenuTable, menuNum);
}

void ServoMidCNV_Debug()
{
    Site_t site;
    
    site.x = 128;  site.y = (1 + Servo_Prmt.Cursor) * 16;
    
    LCD_num_BC(site, (uint32)MID_PWM, 5, RED, BLUE);
                    
    do{
        
		KeySan();
        
	    switch(keymsg.key)
	    {
	        case(KEY_U):
                MID_PWM += 5;
                LCD_num_BC(site, (uint32)MID_PWM, 5, RED, BLUE);
                ftm_pwm_duty(SD5_FTM, SD5_CH,MID_PWM);
            break;
            
	        case(KEY_D):
                MID_PWM -= 5;
                LCD_num_BC(site, (uint32)MID_PWM, 5, RED, BLUE);
                ftm_pwm_duty(SD5_FTM, SD5_CH,MID_PWM);
            break;
            
	        case(KEY_L):
                MID_PWM -= 20;
                LCD_num_BC(site, (uint32)MID_PWM, 5, RED, BLUE);
                ftm_pwm_duty(SD5_FTM, SD5_CH,MID_PWM);
            break;
            
	        case(KEY_R):
                MID_PWM += 20;
                LCD_num_BC(site, (uint32)MID_PWM, 5, RED, BLUE);
                ftm_pwm_duty(SD5_FTM, SD5_CH,MID_PWM);
            break;
            
	        default:
                break;
	    }
    }while(keymsg.key != KEY_B);   
    
    
}

void ServoMax_Debug()
{
    Site_t site;
    
    site.x = 128;  site.y = (1 + Servo_Prmt.Cursor) * 16;
    
    LCD_num_BC(site, (uint32)SERVOPWM_MAX, 5, RED, BLUE);
                    
    do{
        
		KeySan();
        
	    switch(keymsg.key)
	    {
	        case(KEY_U):
                SERVOPWM_MAX += 5;
                LCD_num_BC(site, (uint32)SERVOPWM_MAX, 5, RED, BLUE);
                ftm_pwm_duty(SD5_FTM, SD5_CH,MID_PWM + SERVOPWM_MAX);
            break;
            
	        case(KEY_D):
                SERVOPWM_MAX -= 5;
                LCD_num_BC(site, (uint32)SERVOPWM_MAX, 5, RED, BLUE);
                ftm_pwm_duty(SD5_FTM, SD5_CH,MID_PWM + SERVOPWM_MAX);
            break;
            
	        case(KEY_L):
                SERVOPWM_MAX -= 20;
                LCD_num_BC(site, (uint32)SERVOPWM_MAX, 5, RED, BLUE);
                ftm_pwm_duty(SD5_FTM, SD5_CH,MID_PWM + SERVOPWM_MAX);
            break;
            
	        case(KEY_R):
                SERVOPWM_MAX += 20;
                LCD_num_BC(site, (uint32)SERVOPWM_MAX, 5, RED, BLUE);
                ftm_pwm_duty(SD5_FTM, SD5_CH,MID_PWM + SERVOPWM_MAX);
            break;
            
	        default:
                break;
	    }
    }while(keymsg.key != KEY_B);   
    
    
}

void ServoMin_Debug()
{
    Site_t site;
    
    site.x = 128;  site.y = (1 + Servo_Prmt.Cursor) * 16;
    
    LCD_num_BC(site, (uint32)SERVOPWM_MIN, 5, RED, BLUE);
                    
    do{
        
		KeySan();
        
	    switch(keymsg.key)
	    {
	        case(KEY_U):
                SERVOPWM_MIN += 5;
                LCD_num_BC(site, (uint32)SERVOPWM_MIN, 5, RED, BLUE);
                ftm_pwm_duty(SD5_FTM, SD5_CH,MID_PWM - SERVOPWM_MIN);
            break;
            
	        case(KEY_D):
                SERVOPWM_MIN -= 5;
                LCD_num_BC(site, (uint32)SERVOPWM_MIN, 5, RED, BLUE);
                ftm_pwm_duty(SD5_FTM, SD5_CH,MID_PWM - SERVOPWM_MIN);
            break;
            
	        case(KEY_L):
                SERVOPWM_MIN -= 20;
                LCD_num_BC(site, (uint32)SERVOPWM_MIN, 5, RED, BLUE);
                ftm_pwm_duty(SD5_FTM, SD5_CH,MID_PWM - SERVOPWM_MIN);
            break;
            
	        case(KEY_R):
                SERVOPWM_MIN += 20;
                LCD_num_BC(site, (uint32)SERVOPWM_MIN, 5, RED, BLUE);
                ftm_pwm_duty(SD5_FTM, SD5_CH,MID_PWM - SERVOPWM_MIN);
            break;
            
	        default:
                break;
	    }
    }while(keymsg.key != KEY_B);   
    
    
}

/******************************************************************************
* FunctionName   : Menu_Motro()
* Description    : �������
* EntryParameter : None
* ReturnValue    : None
*******************************************************************************/
void Menu_Motro(void)
{
	LCD_clear(WHITE); 
	
	uint8 menuNum;

    menuNum = sizeof(Motro_MenuTable)/sizeof(Motro_MenuTable[0]);         // �˵�����

    Menu_Process("-= MotroDebug =-", &Motro_Prmt, Motro_MenuTable, menuNum);
}


/******************************************************************************
* FunctionName   : Menu_Img()
* Description    : ͼ������
* EntryParameter : None
* ReturnValue    : None
*******************************************************************************/
void Menu_Img()
{
	LCD_clear(WHITE); 
	
	uint8 menuNum;

    menuNum = sizeof(Img_MenuTable)/sizeof(Img_MenuTable[0]);         // �˵�����

    Menu_Process("-= ImgDebug =-", &Img_Prmt, Img_MenuTable, menuNum);
}

void Menu_ov7725_cnst_set()
{
  
        Site_t site;
	
	site.x = 128;	site.y = (1 + Img_Prmt.Cursor) * 16;
        LCD_num_BC(site, ov7725_cnst_set, 4, RED, BLUE);
	
	adjustParam(site, &ov7725_cnst_set, 4, RED, BLUE);
        if(ov7725_cnst_set > 0xFF) ov7725_cnst_set = 0xFF; //�޷�
        
    uint8 Sensor_IDCode = 0;
    
    if( 0 == SCCB_ReadByte( &Sensor_IDCode, 1, OV7725_VER ) )    /* ��ȡsensor ID��*/
    {
        mLCD_str(0, 56, "Read ID Failed", RED,WHITE);
        DELAY_MS(200);
    }
    
    if(Sensor_IDCode == OV7725_ID)                            /*ȷ��ID����ȷ*/
    {

            if( 0 == SCCB_WriteByte(0x9C, ov7725_cnst_set) )
            {
              mLCD_str(0, 56, "Write Reg Failed", RED,WHITE);
              DELAY_MS(200);
            }
            else
            {
              mLCD_str(0, 56, "CNST Set Successful !", RED,WHITE);
                DELAY_MS(200);
            }
    }
    else
    {
      mLCD_str(0, 56, "ID Wrong ", RED,WHITE);
      DELAY_MS(200);
    }
	 
    DELAY_MS(200);

    LCD_clear(WHITE);    //�����˳�
}
/******************************************************************************
* FunctionName   : Menu_Motro()
* Description    : ����ģʽ����
* EntryParameter : None
* ReturnValue    : None
*******************************************************************************/
void Menu_SelectMode(void)
{
	LCD_clear(WHITE); 
	
	uint8 menuNum;

    menuNum = sizeof(RunMode_MenuTable)/sizeof(RunMode_MenuTable[0]);         // �˵�����

    Menu_Process(" -= RunMode =- ", &RunMode_Prmt, RunMode_MenuTable, menuNum);
}
//����ģʽ
void Normal_Run()
{    
	LCD_clear(WHITE);
	
	RunMode = NORMAL_RUN;
	DELAY_MS(200);

	LCD_clear(WHITE);    //�����˳�
}
//����ģʽ
void Constant_Run()
{

        Site_t site;
	
	site.x = 128;	site.y = (1 + RunMode_Prmt.Cursor) * 16;
        LCD_num_BC(site, ConstantSpeed, 3, RED, BLUE);
	
	adjustParam(site, &ConstantSpeed, 3, RED, BLUE);
        RunMode = TIMING_SPEED_RUN;
	
}
//��ʱ����
void Timing_Run()
{
	Site_t site;
	
	site.x = 128;	site.y = (1 + RunMode_Prmt.Cursor) * 16;
        LCD_num_BC(site, RuningTime, 2, RED, BLUE);
	
	adjustParam(site, &RuningTime, 2, RED, BLUE);
	
    RunMode = TIMING_RUN;
}

void Menu_SetRampDelayDown(){

  	Site_t site;
	
	site.x = 128;	site.y = (1 + Motro_Prmt.Cursor) * 16;
        LCD_num_BC(site, (uint16)(rampDelayDown/100), 4, RED, BLUE);
	
	adjustParam_Big(site, &rampDelayDown, 4, RED, BLUE);
	
}

void Menu_SetRampDelayUp(){

  	Site_t site;
	
	site.x = 128;	site.y = (1 + Motro_Prmt.Cursor) * 16;
        LCD_num_BC(site, (uint16)(rampDelayUp/100), 4, RED, BLUE);
	
	adjustParam_Big(site, &rampDelayUp, 4, RED, BLUE);
}

//���ι�ʽ����
void Quadra_Run()
{
	Site_t site;
	
	site.x = 128;	site.y = (1 + RunMode_Prmt.Cursor) * 16;
        LCD_num_BC(site, RuningTime, 2, RED, BLUE);
	
	adjustParam(site, &RuningTime, 2, RED, BLUE);
	
    RunMode = QUADRA_RUN;
}


/******************************************************************************
* FunctionName   : Menu_Speed()
* Description    : �ٶȿ��Ʋ�������
* EntryParameter : None
* ReturnValue    : None
*******************************************************************************/
void Menu_Speed()
{
	LCD_clear(WHITE);
	
	uint8 menuNum;

    menuNum = sizeof(Speed_MenuTable)/sizeof(Speed_MenuTable[0]);        // �˵�����

    Menu_Process("-= SpeedDebug =-", &Speed_Prmt, Speed_MenuTable, menuNum);
}

/******************************************************************************
* FunctionName   : Menu_Speed_Detect()
* Description    : �ٶȿ����жϲ�������
* EntryParameter : None
* ReturnValue    : None
*******************************************************************************/
void Menu_Speed_Detect()
{
	LCD_clear(WHITE);
	
	uint8 menuNum;

    menuNum = sizeof(Speed_Detect_MenuTable)/sizeof(Speed_Detect_MenuTable[0]);        // �˵�����

    Menu_Process("-= SpeedJudgeDebug =-", &Speed_Detect, Speed_Detect_MenuTable, menuNum);
}

void Menu_Annuulus()
{
    LCD_clear(WHITE);
	
	uint8 menuNum;

    menuNum = sizeof(Annuulus_About_MenuTable)/sizeof(Annuulus_About_MenuTable[0]);        // �˵�����

    Menu_Process("-= AnnulusParams =-", &Annuulus_About, Annuulus_About_MenuTable, menuNum);
}

void Menu_Read_Flash()
{
  	LCD_clear(WHITE);
	
	uint8 menuNum;

    menuNum = sizeof(Read_Flash_MenuTable)/sizeof(Read_Flash_MenuTable[0]);        // �˵�����

    Menu_Process("-=ChooseSector2Read=-", &Read_Flash_Prmt, Read_Flash_MenuTable, menuNum);
}
void Menu_Write_Flash()
{
  	LCD_clear(WHITE);
	
	uint8 menuNum;

    menuNum = sizeof(Write_Flash_MenuTable)/sizeof(Write_Flash_MenuTable[0]);        // �˵�����

    Menu_Process("-= ChooseSector2Write =-", &Write_Flash_Prmt, Write_Flash_MenuTable, menuNum);
}

  //����ͬ����
void Read_Flash1(){
  LCD_clear(WHITE);
  flash_num = 1;
  FlashRead();

  mLCD_str(10, 56, "Read XHG OK!", RED,WHITE);
  DELAY_MS(500);
   LCD_clear(WHITE);
}

void Read_Flash2(){
  LCD_clear(WHITE);
  flash_num = 2;
  FlashRead();

  mLCD_str(10, 56, "Read CZH OK!", RED,WHITE);
  DELAY_MS(500);
   LCD_clear(WHITE);
}

void Read_Flash3(){
  LCD_clear(WHITE);
  flash_num = 3;
  FlashRead();

  mLCD_str(10, 56, "Read LC OK!", RED,WHITE);
  DELAY_MS(500);
   LCD_clear(WHITE);
}

void Read_WenRuLaoGou(){
  LCD_clear(WHITE);
  flash_num = 4;
  FlashRead();

  mLCD_str(10, 56, "Read WenRuLaoGou OK!", RED,WHITE);
  DELAY_MS(500);
  LCD_clear(WHITE);
}

void Read_XuHuanYiQiang(){
  LCD_clear(WHITE);
  flash_num = 5;
  FlashRead();

  mLCD_str(10, 56, "Read XuHuanYiQiang OK!", RED,WHITE);
  DELAY_MS(500);
  LCD_clear(WHITE);
}


  //д����
void Write_Flash1(){
  LCD_clear(WHITE);
  flash_num = 1;
  FlashSave();
  mLCD_str(10, 56, "Write XHG OK!", RED,WHITE);
  DELAY_MS(500);
   LCD_clear(WHITE);
}

void Write_Flash2(){
  LCD_clear(WHITE);
  flash_num = 2;
  FlashSave();
  mLCD_str(10, 56, "Write CZH OK!", RED,WHITE);
  DELAY_MS(500);
   LCD_clear(WHITE);
}

void Write_Flash3(){
  LCD_clear(WHITE);
  flash_num = 3;
  FlashSave();
  mLCD_str(10, 56, "Write LC OK!", RED,WHITE);
  DELAY_MS(500);
   LCD_clear(WHITE);
}

void Write_WenRuLaoGou(){
  LCD_clear(WHITE);
  flash_num = 4;
  FlashSave();
  mLCD_str(10, 56, "Write WenRuLaoGou OK!", RED,WHITE);
  DELAY_MS(500);
  LCD_clear(WHITE);
}

void Write_XuHuanYiQiang(){
  LCD_clear(WHITE);
  flash_num = 5;
  FlashSave();
  mLCD_str(10, 56, "Write XuHuanYiQiang OK!", RED,WHITE);
  DELAY_MS(500);
  LCD_clear(WHITE);
}

/******************************************************************************
* FunctionName   : Menu_UploadData()
* Description    : �ϴ�����ѡ��
* EntryParameter : None
* ReturnValue    : None
*******************************************************************************/
void Menu_UploadData()
{
	LCD_clear(WHITE);
	
	uint8 menuNum;

    menuNum = sizeof(Upload_MenuTable)/sizeof(Upload_MenuTable[0]);        // �˵�����

    Menu_Process("-= UploadData =-", &UploadData_Prmt, Upload_MenuTable, menuNum);
}

void Upload_Speed()
{
	LCD_clear(WHITE);
	
	//Flag.upLoad_SpeedWave = 1;

	DELAY_MS(100);
	LCD_clear(WHITE);    //�����˳�
}

void Menu_SaveImg()
{
  	LCD_clear(WHITE);
	
	uint8 menuNum;

    menuNum = sizeof(SaveImg_MenuTable)/sizeof(SaveImg_MenuTable[0]);        // �˵�����

    Menu_Process("-= SaveImgConfig =-", &SaveImg_Prmt, SaveImg_MenuTable, menuNum);
}





