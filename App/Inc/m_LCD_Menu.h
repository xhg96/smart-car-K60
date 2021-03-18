/*！
*     液晶显示+参数调试程序
*/

#ifndef __M_LCD_MENU_H__
#define __M_LCD_MENU_H__

/* 蜂鸣器开关 */
#define  BEEP_PORT  PTB17
#define  BEEP_ON   PTXn_T(BEEP_PORT,OUT) = 1
#define  BEEP_OFF  PTXn_T(BEEP_PORT,OUT) = 0
/******************************* 菜单结构体定义 *******************************/
typedef struct{

	uint8 ExitMark;     // 退出菜单(0-不退出，1-退出)

	uint8 Cursor;       // 光标值(当前光标位置)

	uint8 PageNo;       // 菜单页(显示开始项)

	uint8 Index;        // 菜单索引(当前选择的菜单项)

	uint8 DispNum;      // 显示项数(每页可以现在菜单项)

	uint8 MaxPage;      // 最大页数(最大有多少种显示页)

}MENU_PRMT;      // 菜单参数


typedef struct{

	uint8 *MenuName;		// 菜单项目名称

	void(*ItemHook)(void);  // 要运行的菜单函数

	uint16 *DebugParam;		// 要调试的参数
	
}MENU_TABLE;     // 菜单执行

/***************************** 菜单结构体定义 *********************************/


/*
 *  供外部调用的函数接口声明
 */
extern void PORTA_IRQHandler(void);
extern void Read_EEPROM();
extern void Write_EEPROM();
extern void MainMenu_Set();
//供外部调用变量
extern volatile KEY_MSG_t keymsg;        //按键消息

#endif