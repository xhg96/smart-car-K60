#include "common.h"
#include "VCAN_LCD.h"
#include "VCAN_UI_VAR.h"
#include "VCAN_NRF24L0_MSG.h"
#include "MK60_FLASH.h"    //FLASH
#include "flash_of_mine.h"

#define VAR_VALUE(var_tab)      num_info[var_tab].val       //ָ����ŵı�����ֵ
#define VAR_OLDVALUE(var_tab)   num_info[var_tab].oldval    //ָ����ŵı��������ȷ��ֵ
#define VAR_MAXVALUE(var_tab)   num_info[var_tab].maxval
#define VAR_MINVALUE(var_tab)   num_info[var_tab].minval
#define VAR_SITE(var_tab)       num_info[var_tab].site
#define VAR_VALUE_OFFSET(var_tab) num_info[var_tab].changval
#define VAR_VALUE_HOLE_OFFSET(var_tab) num_info[var_tab].holdval
#define VAR_NAME(var_tab) num_info[var_tab].name
#define VAR_NAMESITE(var_tab) num_info[var_tab].namesite
#define VAR_SHOWEN(var_tab) num_info[var_tab].showen
extern uint8 flash_num ;
#define SECTOR_NUM  (FLASH_SECTOR_NUM - flash_num)         //������������������ȷ����ȫ

extern float LCD_pagenum;
extern float  k, b ,kk;
extern float steer_mid;
extern float k2;
extern float b2;
/**************�ٶȲ���**********/
extern float setRunTime;        //��ʱʱ��
extern float add_speed;          //����flash�У��ɰ������� //����
extern float min_speed;        //����flash�У��ɰ������� //�����С�ٶ�
extern float max_speed;        //����flash�У��ɰ������� //ֱ������ٶ�
extern float ra_speed;         //����flash�У��ɰ������� //�µ����ٶ�
extern float dzhi_top;          //����flash�У��ɰ������� //ֱ��ɲ�����ж�
extern float dzhi_speed;       //����flash�У��ɰ������� //������ٵ�����ٶȣ���ֱ���ٶ�
extern float wan_speed;        //����flash�У��ɰ������� //���ι�ʽ�ĳ�����
extern float outwan_speed;       //����flash�У��ɰ������� //��������ٶ�
extern float divition;
extern float wan_min_speed;
extern float brakeTime;          //ɲ����ʱʱ��
extern float ruwan_speed;       //�������Ŀ���ٶ�
extern float squareTime;        //���η���ʽ�ټ���ʱ��
extern float ob_speed;        //Բ���ٶ�
extern float speed_k;           //���ι�ʽһϵ��
extern float stop_delay_time;
extern float extra_ringStop_speed; //�����ٶ�����
extern float extra_ruWanStop_speed;
/**************���������ж���ֵ����**********/
extern float lpp;               //�ϰ���
extern float rpp;
extern float flpp;
extern float frpp;
extern float ring_min_speed;
extern float direction_save;
extern float wan_ade;           //����flash�У��ɰ������� //������ٵ��ж�
extern float zhidao_ade;        //����flash�У��ɰ������� //��ֱ�����ж�
extern float wanru_ade;         //����flash�У��ɰ������� //������ٵ��ж�
extern float set_othertop;      //����flash�У��ɰ������� //��һ���˵����ã���ֱ����
extern float set_cothertop;     //����flash�У��ɰ������� //��һ���˵����ã���ֱ����
/**************PID����**********/
extern float kp;
extern float ki;
extern float kd;  // 
extern uint16 change_ki;        //����flash�У��ɰ������� //���ٻ��ֵĻ���ki
extern uint16 change_kib;         //����flash�У��ɰ������� //���ٻ��ֵĻ���ki�ĳ�����
extern float k_de_speed;
//������ַ����
float *var_addr[VAR_MAX] = {
(float *)&LCD_pagenum,
(float *)&k, (float *)&b, (float *)&kk, (float *)&kk,(float *)&setRunTime,    //1-5
(float *)&wan_min_speed, (float *)&ring_min_speed,(float *)&steer_mid,(float *)&k_de_speed,(float *)&extra_ringStop_speed,//6-10

(float *)&min_speed,(float *)&max_speed,(float *)&dzhi_speed,(float *)&outwan_speed,(float *)&wan_speed,//11-15
(float *)&ruwan_speed,(float *)&ob_speed,(float *)&add_speed,(float *)&ra_speed,(float *)&stop_delay_time,//16-20
(float *)&extra_ruWanStop_speed,(float *)&speed_k,(float *)&dzhi_top,(float *)&wan_ade,(float *)&zhidao_ade,////21-25

(float *)&wanru_ade, (float *)&set_othertop,(float *)&set_cothertop,(float *)&k2,(float *)&b2,//26-30
(float *)&kp,(float *)&ki,(float *)&kd,(float *)&change_ki,(float *)&change_kib,//       31-35
(float *)&lpp,(float *)&rpp,(float *)&flpp,(float *)&frpp,(float *)&direction_save//          36-40
};


ui_var_info_t  num_info[VAR_MAX] =
{
  /*
    //  {val,oldval,minval,maxval,{x,y},mincg,maxcg,enshow,name,name{x,y}}
    //val,oldval,���ڵ���key_event_init��ʱ������Ӧ�����︳ֵ���������������ֵ��������д
    //��Ҫ����minval,maxval,{x,y}
    //���ע����Сֵ��Ҫ�������ֵ*/
    {1, 1, 1, 10, {75, 80}, 1, 10, 1, " ", {20,100}},           //ҳ��
                      
    {0, 0, 0, 300, {75, 0}, 1, 10, 1,         "    k:",       {25,0}}, //k ,0<k<200
    {0, 0, 0, 30, {75, 16}, 0.01, 0.1, 100,   "    b:",      {25,16}},//b , 0<b<30
    {0, 0, 0, 100, {75, 32}, 0.01, 0.1, 100,  "   kk:",      {25,32}}, //kk ,0<kk<100
    {0, 0, 0, 100 ,{75, 48}, 0.01,0.5, 100,   "    d:",        {25,48}}, //d 
    {0, 0, 0, 100, {75, 64}, 1, 7, 1,        "setRT:",       {25,64}}, //
                                                                                  //5
    {0, 0, 0, 2000, {75, 0}, 1, 10, 1,         "wminSp:",      {15,0}}, //6
    {0, 0, 0, 1000, {75, 16}, 1, 100, 1,     "rinmin:",      {15,16}},//
    {0, 0, 0, 2000, {75, 32}, 1 , 10, 1,      "  mid:",      {15,32}}, //
    {0, 0, 0, 1000 ,{75, 48}, 0.01, 0.1, 100,  "chaSu:",      {15,48}}, //
    {0, 0, 0, 1000, {75, 64}, 1, 10, 1,        "extraR:",      {15,64}}, //
                                                                                //10
    {0, 0, 0, 1000, {75, 0},  5, 10, 1,        "minSp:",      {5,0}}, //11
    {0, 0, 0, 1000, {75, 16}, 5,10, 1,       "maxSp:",      {5,16}},
    {0, 0, 0, 1000, {75, 32}, 5 ,10, 1,      "dzhSp:",      {5,32}}, 
    {0, 0, 0, 1000 ,{75, 48}, 5, 10, 1,       "otwSp :",     {5,48}}, 
    {0, 0, 0, 1000, {75, 64}, 5, 10, 1,       "wanSp:",     {5,64}}, 
                                                                           //15
    {0, 0, 0, 1000, {75, 0},  5,10, 1,         "ruWSp:",     {5,0}}, //16
    {0, 0, 0, 1000, {75, 16}, 5,10, 1,        "obSp:",     {5,16}},
    {0, 0, 0, 1000, {75, 32}, 5,10, 1,        "addSp:",     {5,32}}, 
    {0, 0, 0, 1000 ,{75, 48}, 5,10, 1,         "raSp:",     {5,48}}, 
    {0, 0, 0, 1000, {75, 64}, 1,10, 1,         "stopDT:",    {5,64}},
                                                                               
    {0, 0, 0, 1000, {75, 0}, 1, 10, 1,        "extraS:",    {5,0}}, //21
    {0, 0, 0, 1000, {75, 16}, 1, 10, 1,       " spK:",     {5,16}},
    {0, 0, 0, 1000, {75, 32}, 1 ,10, 1,      "dzTop:",    {5,32}}, 
    {0, 0, 0, 1000 ,{75, 48}, 1,10, 1,       "wande:" ,   {5,48}}, 
    {0, 0, 0, 1000, {75, 64}, 1,10, 1,       "zhide:",    {5,64}},
    //20
    {0, 0, 0, 1000, {75, 0}, 1, 10, 1,        "wanrude:",   {5,0}}, //26
    {0, 0, 0, 1000, {75, 16}, 1, 10, 1,       "sOTop:",   {5,16}},
    {0, 0, 0, 1000, {75, 32}, 1 , 10, 1,      " sCOT:",   {5,32}}, 
    {0, 0, 0, 1000 ,{75, 48}, 1, 10, 1,       "   k2:" ,   {5,48}}, 
    {0, 0, 0, 1000, {75, 64}, 0.1, 1, 100,    "   b2:",    {5,64}},
                                                                                
    {0, 0, 0, 100, {75, 0}, 0.1, 1, 10,       "   kp:",    {25,0}}, //31
    {0, 0, 0, 100, {75, 16}, 0.01, 0.2, 100,  "   ki:",    {25,16}},
    {0, 0, 0, 200, {75, 32}, 1 , 10, 1,       "   k:",     {25,32}}, 
    {0, 0, 0, 100 ,{75, 48}, 0.01, 0.2, 100,  "  cki:",    {25,48}}, 
    {0, 0, 0, 100, {75, 64}, 0.1, 2, 10,      " ckbi:",    {25,64}}, //35
                                                                              
    {0, 0, 0, 1000, {75, 0}, 0.01, 0.5, 100,         "lpp:",     {25,0}}, //36
    {0, 0, 0, 1000, {75, 16}, 0.01, 0.5, 100,       "rpp:",      {25,16}},
    {0, 0, 0, 2000, {75, 32}, 0.01 ,0.5, 100,      "flpp:",      {25,32}}, 
    {0, 0, 0, 100 ,{75, 48}, 0.01, 0.5, 100,        "rlpp:",      {25,48}}, 
    {0, 0, 0, 100, {75, 64}, 1, 10, 1,               "dreSa:",       {25,64}},            //40
//{0��0����Сֵ�����ֵ��{������ʾλ��}��1�ΰ����ı䵥λֵ�������ı�ֵ����ʾֵ*�ı�����"��ǩ"(ð��ǰһ������ַ���ѣ�ͳһ��)��{��ǩλ��} }
//    {0, 0, 0, 1000, {75, 0}, 1, 10, 1,         "rd:",       {25,0}}, 
//    {0, 0, 0, 1000, {75, 16}, 1, 10, 1,       "pwm:",      {25,16}},
//    {0, 0, 0, 2000, {75, 32}, 1 , 10, 1,      "mid:",      {25,32}}, 
//    {0, 0, 0, 100 ,{75, 48}, 1, 10, 1,        "cn:",       {25,48}}, 
//    {0, 0, 0, 100, {75, 64}, 1, 10, 1,        "no:",       {25,64}}, 

};

uint8   new_tab = 0;        //��ǰѡ��ı������
uint32  last_tab;           //�����յ��ı������



//ͬ��ָ����ֵ��tab Ϊ VAR_NUM ʱ��ʾȫ��ͬ����С����ͬ����Ӧ��
//������ͬ������ʾȫ������Ϊ�п���ͬ��ʧ�ܡ�
//static uint8 var_syn(uint8 tab);         //ͬ�����ݣ�1��ʾ�ɹ���0��ʾʧ��

void save_var2buff(var_tab_e var_num, uint8 *sendbuf);              //����Ҫ���͵ı���������д�뵽������
void var_init()
{
    uint8   var_num;
    float  vartemp;

    //FlashSave();  //ֻ��һ��

    
    flash_init();         //��ʼ��flash
    //delay_ms(100);

    FlashRead();       //��ȡflash

    for(var_num = 0; var_num < VAR_MAX; var_num++)
    {
        get_var((var_tab_e)var_num, &vartemp);
        num_info[var_num].val       = vartemp;
        num_info[var_num].oldval    = vartemp;

        //�����Сֵ�����ֵ
        ASSERT(num_info[var_num].maxval  >=  num_info[var_num].minval );
    }
}

void save_var(var_tab_e var_tal, float var_data)
{
    *((float *)(var_addr[var_tal])) = var_data;

    VAR_VALUE(var_tal) = var_data;
    VAR_OLDVALUE(var_tal) = var_data;
}

void get_var(var_tab_e var_tal, float *var_data)
{
    *var_data = (float) * ((float *)(var_addr[var_tal]));
}

void updata_var(var_tab_e var_tal)
{
    float vartemp;

    get_var(var_tal, &vartemp);

    VAR_VALUE(var_tal) = vartemp;
}

//�Ա����ļӼ����д���
void var_value(ui_var_event_e ctrl)
{
    ASSERT((new_tab <= VAR_MAX1 && new_tab >= VAR_MAX1 - 5) || new_tab == VAR_NUM);


    //�޸ĵ�ǰ������ֵ
    switch(ctrl)
    {
    case VAR_ADD:
        if(VAR_VALUE(new_tab) < VAR_MAXVALUE(new_tab))
        {
            VAR_VALUE(new_tab) += VAR_VALUE_OFFSET(new_tab);

        }
        else
        {
            VAR_VALUE(new_tab) = VAR_MINVALUE(new_tab);
        }
        break;

    case VAR_SUB:
        if(VAR_VALUE(new_tab) > VAR_MINVALUE(new_tab))
        {
            VAR_VALUE(new_tab) -= VAR_VALUE_OFFSET(new_tab);
        }
        else
        {
            VAR_VALUE(new_tab) = VAR_MAXVALUE(new_tab) ;//��Сֵ��һΪ���ֵ
        }
        break;

    case VAR_ADD_HOLD:
        if(   (VAR_MAXVALUE(new_tab) - VAR_VALUE(new_tab))  >  VAR_VALUE_HOLE_OFFSET(new_tab) )
        {
            VAR_VALUE(new_tab) += VAR_VALUE_HOLE_OFFSET(new_tab);
        }
        else
        {
            VAR_VALUE(new_tab) = VAR_MINVALUE(new_tab);
        }
        break;

    case VAR_SUB_HOLD:
        if( ( VAR_VALUE(new_tab) - VAR_MINVALUE(new_tab)) > VAR_VALUE_HOLE_OFFSET(new_tab)  )
        {
            VAR_VALUE(new_tab) -= VAR_VALUE_HOLE_OFFSET(new_tab);
        }
        else
        {
            VAR_VALUE(new_tab) = VAR_MAXVALUE(new_tab) ;//��Сֵ��һΪ���ֵ
        }
        break;

    default:                        //��Чѡ�񣬲���Ҫ�л�
        break;
    }

    var_display(new_tab);
}

//�Ա�������ѡ��
void var_select(ui_var_event_e  ctrl)
{
    ASSERT((new_tab <= VAR_MAX1 && new_tab >= VAR_MAX1 - 5) || new_tab == VAR_NUM);

    uint8 old_tab = new_tab;       //�ȱ��ݵ�ǰ�������

    //�л�����һ������
    switch(ctrl)
    {
    case VAR_NEXT:                      //��һ��
        if(new_tab == 0) new_tab += (LCD_pagenum - 1) * 5;
        new_tab++;
        if(new_tab >= (VAR_MAX1) + 1 )
        {
            new_tab = 0;               //��ͷ��ʼ
        }
        break;

    case VAR_PREV:                      //��һ��
        new_tab--;
        if(new_tab == (LCD_pagenum - 1) * 5) new_tab -= (LCD_pagenum - 1) * 5;
        if(new_tab >= (VAR_MAX1) + 1 )
        {
            new_tab = VAR_MAX1;     //��β��ʼ
        }
        break;

    default:                        //��Чѡ�񣬲���Ҫ�л�
        return;
    }

    var_display(old_tab);               //������һ������

    var_display(new_tab);              //����ǰ������

}


//ȷ�ϵ�ǰѡ���
void var_ok()
{
  Site_t site={2,-29};            //������ӵģ�ˢ����Ļ��
  Size_t size={128,128};              
    ASSERT((new_tab <= VAR_MAX1 && new_tab >= VAR_MAX1 - 5) || new_tab == VAR_NUM);

    //�Ƚ��Ƿ��иı�ֵ
    if(VAR_VALUE(new_tab) != VAR_OLDVALUE(new_tab))   //ֵ�ı��ˣ�����Ҫ����
    {
        var_syn(new_tab);          //ͬ���µ�ֵ
        /*              //�ݲ���flash
        flash_erase_sector(SECTOR_NUM);                     //��������
        //д��flash����ǰ����Ҫ�Ȳ�����Ӧ������(��Ȼ���ݻ���)

        for(uint8 var_num = 1; var_num < VAR_MAX; var_num++)
        {
            flash_write(SECTOR_NUM, 64 * (var_num - 1), (uint32)(*((float *)(var_addr[var_num])) * VAR_SHOWEN(var_num)));
        }
        */
    }
    LCD_rectangle(site,size,BLACK); //������ӵģ�ˢ����Ļ��
    var_display(new_tab);
}

//ȡ����ǰѡ���ֵ  OK
void val_cancel()
{
    ASSERT((new_tab <= VAR_MAX1 && new_tab >= VAR_MAX1 - 5) || new_tab == VAR_NUM);

    //ֱ�ӻ�ԭ��ǰֵ
    VAR_VALUE(new_tab) = VAR_OLDVALUE(new_tab);

    var_display(new_tab);
}

//��ʾָ����ֵ��tab Ϊ VAR_MAX ʱ��ʾȫ����ʾ��С������ʾ��Ӧ��

void var_display(uint8 tab)
{
#if UI_VAR_USE_LCD

    //���屳����ʱ
#define SELECT_NO_CHANGE_BG         RED   //��ǰѡ�У�����û�иı�
#define SELECT_CHANGE_BG            RED   //��ǰѡ�У����Ҹı���
#define NO_SELECT_NO_CHANGE_BG      BLACK     //û��ѡ�У�����û�иı䣨��ͨ�ľ���������
#define NO_SELECT_CHANGE_BG         BLACK     //û��ѡ�У����Ҹı���

    //����������ɫ
#define SELECT_NO_CHANGE            BLACK   //��ǰѡ�У�����û�иı�
#define SELECT_CHANGE               GREEN   //��ǰѡ�У����Ҹı���
#define NO_SELECT_NO_CHANGE         WHITE    //û��ѡ�У�����û�иı䣨��ͨ�ľ���������
#define NO_SELECT_CHANGE            GREEN   //û��ѡ�У����Ҹı���

    uint8  i = 0;
    uint16 bkColor;
    uint16 Color;

    ASSERT(((new_tab <= VAR_MAX1) && (tab <= VAR_MAX1) && new_tab >= VAR_MAX1 - 5) || new_tab == VAR_NUM);

    if(tab == VAR_MAX1)      //��ʾȫ��
    {
        i = 5;    //ѭ���Ĵ���
        tab = 0;
    }

    do
    {
        if(tab == new_tab)
        {
            //��ʾ��ǰ��ֵ���ж�ֵ�Ƿ�ı�
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //ֵû�ı䣬����Ҫ����
            {
                Color   =  SELECT_NO_CHANGE;
                bkColor =  SELECT_NO_CHANGE_BG;
            }
            else
            {
                Color   =  SELECT_CHANGE;
                bkColor =  SELECT_CHANGE_BG;
            }
        }
        else
        {
            //��ʾ�ǵ�ǰ��ֵ
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //ֵû�ı䣬����Ҫ����
            {
                Color   =  NO_SELECT_NO_CHANGE;
                bkColor =  NO_SELECT_NO_CHANGE_BG;

            }
            else
            {
                Color   =  NO_SELECT_CHANGE;
                bkColor =  NO_SELECT_CHANGE_BG;
            }
        }
        LCD_num_C(VAR_SITE(tab), (int)(VAR_VALUE(tab) * VAR_SHOWEN(tab)), Color, bkColor);

        if(tab!=0)
            LCD_str(VAR_NAMESITE(tab), (uint8*)VAR_NAME(tab), WHITE, BLACK);
        if(tab == 0)
            tab += (LCD_pagenum - 1) * 5;
        tab++;
    }
    while(i--);         //tab != VAR_MAX ��ʱ��ִ��һ�ξ�����
#else
    tab = tab;          //�������뾯��
#endif
}

//ͬ��ָ����ֵ��tab Ϊ VAR_MAX ʱ��ʾȫ��ͬ����С����ͬ����Ӧ��
uint8 var_syn(uint8 tab)
{
    ASSERT(((new_tab <= VAR_MAX1) && (tab <= VAR_MAX1) && new_tab >= VAR_MAX1 - 5) || new_tab == VAR_NUM);

    uint8  i = 0;

    if(tab == VAR_MAX1)
    {
        i = 5;
        tab = 0;
    }

    do
    {
        //��ֵ���Ƶ���Ӧ�ı���
        save_var((var_tab_e)tab, VAR_VALUE(tab));
        if(tab == 0)
            tab += (LCD_pagenum - 1) * 5;
        tab++;
    }
    while(i--);

    return 1;
}