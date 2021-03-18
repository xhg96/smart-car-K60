#include "common.h"
#include "MK60_gpio.h"
#include "m_Switch.h"
#include "VCAN_KEY.H" 

void PORTD_IRQHandler(void);
extern KEY_MSG_t keymsg;   

/* ����SWITCH��Ŷ�Ӧ�Ĺܽ� */
PTXn_e SWITCH_PTxn[SWITCH_MAX] = {PTD15, PTE6, PTE7, PTE8, PTE9, PTE10,PTE11, PTE12};

/******************************************************************************
 *  @brief      ��ʼ��switch�˿�
 *****************************************************************************/
void Switch_Init()
{
    uint8 i = SWITCH_MAX;

    //��ʼ��ȫ�� ����
    while(i--)
    {
        gpio_init(SWITCH_PTxn[i], GPI, 0);
        port_init_NoALT(SWITCH_PTxn[i], PULLUP);    //���ָ��ò��䣬�����ı�����ѡ��
    }
    
    
    set_vector_handler(PORTD_VECTORn, PORTD_IRQHandler);
    enable_irq(PORTD_IRQn);               // ��PORTD�ж�(����)

}

  //  PORTD�жϷ�����(�����½����ж�)
void PORTD_IRQHandler(void)
{
    uint32 flag;
    
    while(!PORTD_ISFR);
    flag = PORTD_ISFR;  //ȡ�ж����ź�
    PORTD_ISFR = ~0;    //���жϱ�־λ
    
   if(flag & (1 << 11)){
       DELAY_MS(10);    //������ʱ����    
		// �����ϼ�����
       if ( !PTD11_IN ){
          keymsg.key = KEY_U;
          keymsg.status = KEY_DOWN;
       }
    }
    else if(flag & (1 << 14)){
        DELAY_MS(10);    
        
    // �����¼�����
	if ( !PTD14_IN ){
          keymsg.key = KEY_D;
          keymsg.status = KEY_DOWN;
        }
    }
    else if(flag & (1 << 13)){
        DELAY_MS(10);   
        
		// �����������
      if ( !PTD13_IN ){
        keymsg.key = KEY_L;
        keymsg.status = KEY_DOWN;
      }
    }
    else if(flag & (1 << 10))
    {
      DELAY_MS(10);    //������ʱ����
        
		// �����Ҽ�����
      if ( !PTD10_IN )
      {
	keymsg.key = KEY_R;
	keymsg.status = KEY_DOWN;
      }
    }
    else if(flag & (1 << 12))
    {
       DELAY_MS(10);    //������ʱ����
        
		// ȷ�ϼ�����
       if( !PTD12_IN ){
          keymsg.key = KEY_B;
          keymsg.status = KEY_DOWN;
        }
    }
}

/******************************************************************************
 *  @brief      ��ȡswitch״̬
 *  @param      SWITCH_e         SWITCH���
 *  @return     SWITCH_STATUS_e    SWITCH״̬(SWITCH_ON,SWITCH_OFF)
 ******************************************************************************/
SWITCH_STATUS_e Switch_Get(SWITCH_e i)
{
    if(gpio_get(SWITCH_PTxn[i]) == SWITCH_OFF)
    {
        return SWITCH_OFF;
    }
    return SWITCH_ON;
}

uint16 picTotalNumSet = 1200;         //��ͼ����������LCD�е���
uint16 skipNumSet = 5;                   //����ѭ����
uint16 savePicSwitch = 0;              //��ͼ�񿪹�

/*      //ԭ��main�е����ú���

void imgSave2SD_hand(int skipNum, int picNumLimit){     //���ô�ͼ��
  static int skipCount   = 0;
  static int picNumCount = 0;
  
  skipCount++;
  if(skipCount > skipNum && picNumCount < picNumLimit){
    img_sd_save(imgbuff,CAMERA_SIZE);     //��ͼ��
    skipNum = 0;
    picNumCount++;
  }
  else if(picNumCount == picNumLimit){
    skipCount = 0;      //�����
    img_sd_exit();
  }
    
}

//
//  @brief      ��ͼ����microSD��,while�е���
//  @param      skipNum,picNumLimit:  ��ͼ��������ͼ����������
//  @return     None
//


void m_imgSave2SD(int skipNum, int picNumLimit){
  static int skipCount   = 0;
  static int picNumCount = 0;
  static uint8 saveStatus = 0;  //�洢״̬ 0
  skipCount = skipCount>10000?0:skipCount;      //�����
 
  if(saveStatus == 0 && keymsg.key == KEY_START && savePicSwitch == 1)
    saveStatus = 1;     //��ʼ��ͼ��
  
  if(saveStatus == 1){
    skipCount++;
    if(skipCount > skipNum && picNumCount < picNumLimit ){
      img_sd_save(imgbuff,CAMERA_SIZE);     //��ͼ��
      skipNum = 0;
      picNumCount++;
    }
    else if(picNumCount == picNumLimit){
      savePicSwitch = 0;  //�رմ�ͼ��
      saveStatus = 0;   //��ԭ״̬
      img_sd_exit();
    }
  }
}

  //disk��ʽתFAT��sd��
void save2SD_disk(){
        if( GET_SWITCH4() == SWITCH_ON &&(GET_SWITCH5() == SWITCH_ON || keymsg.key == KEY_START ))
        {     
            send_data_to_SD_disk();  //���ݱ��浽���� 
        }
            //ͣ�����SD��
        if( RunMode == TIMING_RUN && stopFlag == 1 && GET_SWITCH4() == SWITCH_ON && keymsg.key == KEY_U)
        {
            if(SPD.nowSpeed < 20)
            {
		DISABLE_MOTOR;
                disable_irq(PIT0_IRQn);   // �ر�PIT0�ж�
                disable_irq(PIT1_IRQn);   // �ر�PIT1�ж�
                disable_irq(PORTB_IRQn);   //�ر�PTB�ĳ��ж�
               
                //��ʼ��sd�����ݱ��浽file��
                SD_disk_to_FAT_file();
                DisableInterrupts;      
                while(keymsg.key == KEY_U);
            }
        }
}


void touch_IRQHandler(){
   uint32 flag;
    
    while(!PORTC_ISFR);
    flag = PORTC_ISFR;  //ȡ�ж����ź�
    PORTC_ISFR = ~0;    //���жϱ�־λ
   // int a = PTC18_IN;
    
    if(flag & (1 << 18)){   
	DELAY_MS(10);	//�˴��ɼ���ʱ����
       if (! PTC18_IN ){
          touchFlag = 1;
          setTimeBeep_ms(200,VHIGH);
          
       }
    }
    if(flag & (1 << 19)){   
	DELAY_MS(10);	
       if (! PTC19_IN ){
          touchFlag = 1;
          setTimeBeep_ms(200,VHIGH);
          
       }
    }
    
}

void touchSwitchInit(){
    //PTC18, 19��Ϊ 
  gpio_init(PTC18, GPI, 0);
  port_init_NoALT(PTC18,  PULLUP | IRQ_FALLING );    //���ָ��ò��䣬�����ı�����ѡ��
  gpio_init(PTC19, GPI, 0);
  port_init_NoALT(PTC19,  PULLUP | IRQ_FALLING );    //���ָ��ò��䣬�����ı�����ѡ��
  
  set_vector_handler(PORTC_VECTORn, touch_IRQHandler);
  enable_irq(PORTC_IRQn);   //ʹ��
}
*/