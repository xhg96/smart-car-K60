#include "common.h"
#include "VCAN_LCD.h"
#include "VCAN_UI_VAR.h"
#include "VCAN_NRF24L0_MSG.h"
#include "MK60_FLASH.h"    //FLASH
#include "flash_of_mine.h"

#define VAR_VALUE(var_tab)      num_info[var_tab].val       //指定标号的变量的值
#define VAR_OLDVALUE(var_tab)   num_info[var_tab].oldval    //指定标号的变量的最后确认值
#define VAR_MAXVALUE(var_tab)   num_info[var_tab].maxval
#define VAR_MINVALUE(var_tab)   num_info[var_tab].minval
#define VAR_SITE(var_tab)       num_info[var_tab].site
#define VAR_VALUE_OFFSET(var_tab) num_info[var_tab].changval
#define VAR_VALUE_HOLE_OFFSET(var_tab) num_info[var_tab].holdval
#define VAR_NAME(var_tab) num_info[var_tab].name
#define VAR_NAMESITE(var_tab) num_info[var_tab].namesite
#define VAR_SHOWEN(var_tab) num_info[var_tab].showen
extern uint8 flash_num ;
#define SECTOR_NUM  (FLASH_SECTOR_NUM - flash_num)         //尽量用最后面的扇区，确保安全

extern float LCD_pagenum;
extern float  k, b ,kk;
extern float steer_mid;
extern float k2;
extern float b2;
/**************速度参数**********/
extern float setRunTime;        //定时时间
extern float add_speed;          //存在flash中，由按键调节 //加速
extern float min_speed;        //存在flash中，由按键调节 //弯道最小速度
extern float max_speed;        //存在flash中，由按键调节 //直道最大速度
extern float ra_speed;         //存在flash中，由按键调节 //坡道的速度
extern float dzhi_top;          //存在flash中，由按键调节 //直道刹车的判断
extern float dzhi_speed;       //存在flash中，由按键调节 //出弯加速的最大速度，短直道速度
extern float wan_speed;        //存在flash中，由按键调节 //二次公式的常数项
extern float outwan_speed;       //存在flash中，由按键调节 //出弯基础速度
extern float divition;
extern float wan_min_speed;
extern float brakeTime;          //刹车延时时间
extern float ruwan_speed;       //入弯减速目标速度
extern float squareTime;        //二次方公式再加速时间
extern float ob_speed;        //圆环速度
extern float speed_k;           //二次公式一系数
extern float stop_delay_time;
extern float extra_ringStop_speed; //额外速度余量
extern float extra_ruWanStop_speed;
/**************赛道类型判断阈值参数**********/
extern float lpp;               //障碍用
extern float rpp;
extern float flpp;
extern float frpp;
extern float ring_min_speed;
extern float direction_save;
extern float wan_ade;           //存在flash中，由按键调节 //出弯加速的判断
extern float zhidao_ade;        //存在flash中，由按键调节 //入直道的判断
extern float wanru_ade;         //存在flash中，由按键调节 //弯道加速的判断
extern float set_othertop;      //存在flash中，由按键调节 //另一顶端的设置，短直道的
extern float set_cothertop;     //存在flash中，由按键调节 //另一顶端的设置，长直道的
/**************PID参数**********/
extern float kp;
extern float ki;
extern float kd;  // 
extern uint16 change_ki;        //存在flash中，由按键调节 //变速积分的基础ki
extern uint16 change_kib;         //存在flash中，由按键调节 //变速积分的基础ki的常数项
extern float k_de_speed;
//变量地址数组
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
    //val,oldval,会在调用key_event_init的时候从其对应变量里赋值过来，所以这里的值可以随意写
    //需要设置minval,maxval,{x,y}
    //务必注意最小值不要大于最大值*/
    {1, 1, 1, 10, {75, 80}, 1, 10, 1, " ", {20,100}},           //页码
                      
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
//{0，0，最小值，最大值，{变量显示位置}，1次按键改变单位值，长按改变值，显示值*的倍数，"标签"(冒号前一共五个字符最佳（统一）)，{标签位置} }
//    {0, 0, 0, 1000, {75, 0}, 1, 10, 1,         "rd:",       {25,0}}, 
//    {0, 0, 0, 1000, {75, 16}, 1, 10, 1,       "pwm:",      {25,16}},
//    {0, 0, 0, 2000, {75, 32}, 1 , 10, 1,      "mid:",      {25,32}}, 
//    {0, 0, 0, 100 ,{75, 48}, 1, 10, 1,        "cn:",       {25,48}}, 
//    {0, 0, 0, 100, {75, 64}, 1, 10, 1,        "no:",       {25,64}}, 

};

uint8   new_tab = 0;        //当前选择的变量编号
uint32  last_tab;           //最后接收到的变量编号



//同步指定的值。tab 为 VAR_NUM 时表示全部同步，小于则同步对应的
//必须先同步再显示全部，因为有可能同步失败。
//static uint8 var_syn(uint8 tab);         //同步数据，1表示成功，0表示失败

void save_var2buff(var_tab_e var_num, uint8 *sendbuf);              //把需要发送的变量的数据写入到缓冲区
void var_init()
{
    uint8   var_num;
    float  vartemp;

    //FlashSave();  //只存一次

    
    flash_init();         //初始化flash
    //delay_ms(100);

    FlashRead();       //读取flash

    for(var_num = 0; var_num < VAR_MAX; var_num++)
    {
        get_var((var_tab_e)var_num, &vartemp);
        num_info[var_num].val       = vartemp;
        num_info[var_num].oldval    = vartemp;

        //检测最小值与最大值
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

//对变量的加减进行处理
void var_value(ui_var_event_e ctrl)
{
    ASSERT((new_tab <= VAR_MAX1 && new_tab >= VAR_MAX1 - 5) || new_tab == VAR_NUM);


    //修改当前变量的值
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
            VAR_VALUE(new_tab) = VAR_MAXVALUE(new_tab) ;//最小值减一为最大值
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
            VAR_VALUE(new_tab) = VAR_MAXVALUE(new_tab) ;//最小值减一为最大值
        }
        break;

    default:                        //无效选择，不需要切换
        break;
    }

    var_display(new_tab);
}

//对变量进行选择
void var_select(ui_var_event_e  ctrl)
{
    ASSERT((new_tab <= VAR_MAX1 && new_tab >= VAR_MAX1 - 5) || new_tab == VAR_NUM);

    uint8 old_tab = new_tab;       //先备份当前变量标号

    //切换到下一个变量
    switch(ctrl)
    {
    case VAR_NEXT:                      //下一个
        if(new_tab == 0) new_tab += (LCD_pagenum - 1) * 5;
        new_tab++;
        if(new_tab >= (VAR_MAX1) + 1 )
        {
            new_tab = 0;               //从头开始
        }
        break;

    case VAR_PREV:                      //上一个
        new_tab--;
        if(new_tab == (LCD_pagenum - 1) * 5) new_tab -= (LCD_pagenum - 1) * 5;
        if(new_tab >= (VAR_MAX1) + 1 )
        {
            new_tab = VAR_MAX1;     //从尾开始
        }
        break;

    default:                        //无效选择，不需要切换
        return;
    }

    var_display(old_tab);               //处理上一个变量

    var_display(new_tab);              //处理当前变量：

}


//确认当前选择的
void var_ok()
{
  Site_t site={2,-29};            //自行添加的，刷新屏幕用
  Size_t size={128,128};              
    ASSERT((new_tab <= VAR_MAX1 && new_tab >= VAR_MAX1 - 5) || new_tab == VAR_NUM);

    //比较是否有改变值
    if(VAR_VALUE(new_tab) != VAR_OLDVALUE(new_tab))   //值改变了，则需要处理
    {
        var_syn(new_tab);          //同步新的值
        /*              //暂不用flash
        flash_erase_sector(SECTOR_NUM);                     //擦除扇区
        //写入flash数据前，需要先擦除对应的扇区(不然数据会乱)

        for(uint8 var_num = 1; var_num < VAR_MAX; var_num++)
        {
            flash_write(SECTOR_NUM, 64 * (var_num - 1), (uint32)(*((float *)(var_addr[var_num])) * VAR_SHOWEN(var_num)));
        }
        */
    }
    LCD_rectangle(site,size,BLACK); //自行添加的，刷新屏幕用
    var_display(new_tab);
}

//取消当前选择的值  OK
void val_cancel()
{
    ASSERT((new_tab <= VAR_MAX1 && new_tab >= VAR_MAX1 - 5) || new_tab == VAR_NUM);

    //直接还原当前值
    VAR_VALUE(new_tab) = VAR_OLDVALUE(new_tab);

    var_display(new_tab);
}

//显示指定的值。tab 为 VAR_MAX 时表示全部显示，小于则显示对应的

void var_display(uint8 tab)
{
#if UI_VAR_USE_LCD

    //定义背景延时
#define SELECT_NO_CHANGE_BG         RED   //当前选中，而且没有改变
#define SELECT_CHANGE_BG            RED   //当前选中，而且改变了
#define NO_SELECT_NO_CHANGE_BG      BLACK     //没有选中，而且没有改变（普通的就是这样）
#define NO_SELECT_CHANGE_BG         BLACK     //没有选中，而且改变了

    //定义文字颜色
#define SELECT_NO_CHANGE            BLACK   //当前选中，而且没有改变
#define SELECT_CHANGE               GREEN   //当前选中，而且改变了
#define NO_SELECT_NO_CHANGE         WHITE    //没有选中，而且没有改变（普通的就是这样）
#define NO_SELECT_CHANGE            GREEN   //没有选中，而且改变了

    uint8  i = 0;
    uint16 bkColor;
    uint16 Color;

    ASSERT(((new_tab <= VAR_MAX1) && (tab <= VAR_MAX1) && new_tab >= VAR_MAX1 - 5) || new_tab == VAR_NUM);

    if(tab == VAR_MAX1)      //显示全部
    {
        i = 5;    //循环的次数
        tab = 0;
    }

    do
    {
        if(tab == new_tab)
        {
            //显示当前的值：判断值是否改变
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //值没改变，不需要处理
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
            //显示非当前的值
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //值没改变，不需要处理
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
    while(i--);         //tab != VAR_MAX 的时候，执行一次就跳出
#else
    tab = tab;          //消除编译警告
#endif
}

//同步指定的值。tab 为 VAR_MAX 时表示全部同步，小于则同步对应的
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
        //把值复制到对应的变量
        save_var((var_tab_e)tab, VAR_VALUE(tab));
        if(tab == 0)
            tab += (LCD_pagenum - 1) * 5;
        tab++;
    }
    while(i--);

    return 1;
}