#ifndef _VCAN_IMG2SD_H_1
#define _VCAN_IMG2SD_H_1


/////////////////////////////////////////////////////////////////////////////////////
//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp) ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

typedef struct
{   
    FATFS filesystem;       //�ļ�ϵͳ
    FIL file;               //�ļ�
    uint32 sector;          //diskģʽ����� ����λ��
    uint8 flag_disk2file;  //��־λ  ��ʼ��disk�����ݱ��浽�ļ�ϵͳ�� 
                            // 0��disk��,1 disk����ת�Ƶ�file
    uint16 file_number;
    uint8 SD_config_OK;
    
    uint8 uart1_rec_busy;
    uint8 stopCar_to_send_data; //ͣ���źţ���ʼ����λ��������
    
} IMAGE2SDINFO;


extern IMAGE2SDINFO Image2sd;


extern void img_sd_init(void);
extern void img_sd_save(uint8 * imgaddr,uint32 size);
extern void img_sd_exit(void);

extern int send_data_to_SD_disk(void);

extern void img_sd_init_disk(void);
extern int SD_disk_to_FAT_file(void);
extern int SD_disk_2_RAK421(void);
#endif //_VCAN_IMG2SD_H_
