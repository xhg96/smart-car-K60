/*��
*     Һ����ʾ+�������Գ���
*/

#ifndef __M_LCD_MENU_H__
#define __M_LCD_MENU_H__

/* ���������� */
#define  BEEP_PORT  PTB17
#define  BEEP_ON   PTXn_T(BEEP_PORT,OUT) = 1
#define  BEEP_OFF  PTXn_T(BEEP_PORT,OUT) = 0
/******************************* �˵��ṹ�嶨�� *******************************/
typedef struct{

	uint8 ExitMark;     // �˳��˵�(0-���˳���1-�˳�)

	uint8 Cursor;       // ���ֵ(��ǰ���λ��)

	uint8 PageNo;       // �˵�ҳ(��ʾ��ʼ��)

	uint8 Index;        // �˵�����(��ǰѡ��Ĳ˵���)

	uint8 DispNum;      // ��ʾ����(ÿҳ�������ڲ˵���)

	uint8 MaxPage;      // ���ҳ��(����ж�������ʾҳ)

}MENU_PRMT;      // �˵�����


typedef struct{

	uint8 *MenuName;		// �˵���Ŀ����

	void(*ItemHook)(void);  // Ҫ���еĲ˵�����

	uint16 *DebugParam;		// Ҫ���ԵĲ���
	
}MENU_TABLE;     // �˵�ִ��

/***************************** �˵��ṹ�嶨�� *********************************/


/*
 *  ���ⲿ���õĺ����ӿ�����
 */
extern void PORTA_IRQHandler(void);
extern void Read_EEPROM();
extern void Write_EEPROM();
extern void MainMenu_Set();
//���ⲿ���ñ���
extern volatile KEY_MSG_t keymsg;        //������Ϣ

#endif