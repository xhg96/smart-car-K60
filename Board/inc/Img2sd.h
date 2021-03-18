#ifndef _VCAN_IMG2SD_H_1
#define _VCAN_IMG2SD_H_1


/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp) ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

typedef struct
{   
    FATFS filesystem;       //文件系统
    FIL file;               //文件
    uint32 sector;          //disk模式保存的 扇区位置
    uint8 flag_disk2file;  //标志位  开始从disk读数据保存到文件系统里 
                            // 0存disk中,1 disk数据转移到file
    uint16 file_number;
    uint8 SD_config_OK;
    
    uint8 uart1_rec_busy;
    uint8 stopCar_to_send_data; //停车信号，开始向上位机发数据
    
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
