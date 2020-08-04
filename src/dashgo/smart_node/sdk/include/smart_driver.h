#ifndef SMART_DRIVER_H
#define SMART_DRIVER_H
#include <stdlib.h>
#include <atomic>
#include <functional>
#include "locker.h"
#include "serial.h"
#include "thread.h"
#include "timer.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include "simplesignal.hpp"
#include "data_fifo.h"
#include "common.h"
#include <map>
#include "detect_task.h"
#include "ActiveSocket.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The SMART SDK requires a C++ compiler to be built"
#endif
#endif

#if !defined(__countof)
#define __countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define _DEBUG
#ifdef _DEBUG
#include <stdio.h>
#define DBG_OUT(...) printf(__VA_ARGS__)
#else
#define DBG_OUT(...)
#endif


typedef enum
{
    Platform_HexCmd_Get_Bdrate = 0x00,

    Platform_HexCmd_Get_Encoder = 0x01,

    Platform_HexCmd_Clear_Encoder = 0x02,

    Platform_HexCmd_Set_Speed = 0x03,
    Platform_HexCmd_Get_Gyro = 0x04,
    Platform_HexCmd_Reset_Gyro = 0x05,
    Platform_HexCmd_Get_AllSensor = 0x06,

    Platform_HexCmd_Set_Recharge = 0x07,
    Platform_HexCmd_Get_Recharge = 0x08,
    Platform_HexCmd_Get_Version = 0x09,

    Platform_HexCmd_Get_SpeedValue = 0x23,
    Platform_HexCmd_Get_PC_BaseConfig = 0x2A,
}RayTouch_Cmd_Enum;

#define SONAR_RECHARGE_AVOID   (1<<1)
#define DROP_FUNCTION_OPEN     (1<<2)

#define PRODUCT_NUM_LEN 64
#define VERSION_INFO_LENGTH 18
/* pc udp prameters */
#define PC_UDP_SOCKET_INDEX  0
#define PC_REMOTE_IP1 192
#define PC_REMOTE_IP2 168
#define PC_REMOTE_IP3 31
#define PC_REMOTE_IP4 48
#define PC_REMOTE_PORT 6000
#define PC_LOCAL_PORT 5000
/* lidar udp prameters */
#define LIDAR_UDP_SOCKET_INDEX  1
#define LIDAR_REMOTE_IP1 192
#define LIDAR_REMOTE_IP2 168
#define LIDAR_REMOTE_IP3 31
#define LIDAR_REMOTE_IP4 48
#define LIDAR_REMOTE_PORT 2000
#define LIDAR_LOCAL_PORT 1000
/* debug udp prameters */
#define DEBUG_UDP_SOCKET_INDEX  2
#define DEBUG_REMOTE_IP1 192
#define DEBUG_REMOTE_IP2 168
#define DEBUG_REMOTE_IP3 31
#define DEBUG_REMOTE_IP4 48
#define DEBUG_REMOTE_PORT 4000
#define DEBUG_LOCAL_PORT 3000
/* stm32 local eth prameters */
#define DEFAULT_LOCAL_IP1 192
#define DEFAULT_LOCAL_IP2 168
#define DEFAULT_LOCAL_IP3 31
#define DEFAULT_LOCAL_IP4 100

#define DEFAULT_LOCAL_SUB1 255
#define DEFAULT_LOCAL_SUB2 255
#define DEFAULT_LOCAL_SUB3 255
#define DEFAULT_LOCAL_SUB4 0

#define DEFAULT_LOCAL_GW1 192
#define DEFAULT_LOCAL_GW2 168
#define DEFAULT_LOCAL_GW3 31
#define DEFAULT_LOCAL_GW4 1

#define DEFAULT_LOCAL_DNS1 8
#define DEFAULT_LOCAL_DNS2 8
#define DEFAULT_LOCAL_DNS3 8
#define DEFAULT_LOCAL_DNS4 8

#define TCP_LOCAL_PORT 30000

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


typedef enum
{
    B1 = 0x00,
    E1 = 0x01,
    K4 = 0x02,
    E6 = 0x03,

}ModeType_e;

enum _base_type {
    CIRCLE = 0,
    SQUARE = 1,
};

enum _base_motor_type {
    TWO_WHEEL = 0,
};

//设置为0则无回充，设置为1则前置回充，设置为2则后置回充
enum _recharge_type {
    NO_RECHARGE = 0,
    FRONT = 1,
    BACK = 2,
};

typedef enum //回充状态
{
    Recharge_START = 0,     //启动回充标志
    Recharge_FAIL,              //未找到充电桩发出的红外信号且未收到握手信号
    Recharge_SUCCESS,       //收到充电桩发出的握手信号
    Recharge_GOING,          //充电正常进行中
    Recharge_ERROR,          //充电过程中出现电压不正常
    Recharge_END,           //电池电量充满，充电结束

}Recharge_ENUM;


#ifdef _WIN32
#pragma pack(1)
#endif

struct PIDParamStruct_t
{
        uint32_t kp_back;
        uint32_t ki_back;
        uint32_t kd_back;
} __attribute((packed));

struct ChargeParamStruct_t
{
        uint16_t MaxBattaryVol;
        uint16_t MinBattaryVol;
        uint16_t MaxRechargeVol;
        uint16_t MinRechargeVol;
} __attribute((packed));

//传感器安装的位置和摆放的方向
struct base_pos_t
{
    int16_t  x_to_center_mm;
    int16_t  y_to_center_mm;
    int16_t  z_to_center_mm;
    int16_t  anti_clockwise_angle_to_center_degree;
} __attribute((packed));

//底盘4个顶点的坐标
struct chassis_pos_t
{
    int16_t  x_to_center_mm;
    int16_t  y_to_center_mm;
} __attribute((packed));

struct version_t
{
//    uint8_t ProtocolVer;
//    uint8_t hardWareMainVer;
//    uint8_t hardWareMinVer;
//    uint8_t softWareMainVer;
//    uint8_t softWareMinVer;
    char version_info[VERSION_INFO_LENGTH];
} __attribute((packed));

struct infoStruct_t
{
    uint8_t ModeType;  //注意这里使用枚举类型ModeType_e会是4个字节。导致数据对齐不了
    uint8_t sonar_recharge_value;//回充避障阈值
    uint8_t drop_value;//跌落功能阈值
    uint8_t rechargPosition;//设置为0则无回充，设置为1则前置回充，设置为2则后置回充
    uint16_t wheel_diameter_mm;
    uint16_t encoder_resolution;
    uint16_t SettingBits;//用于配置开启了什么功能
} __attribute((packed));



struct base_conf_response_t
{
    char productNum[PRODUCT_NUM_LEN];
    uint8_t        base_type;
    uint16_t       base_radius_mm;
    uint16_t       distance_between_wheels_mm;
    chassis_pos_t chassis_pos[4];
    uint8_t        base_snoar_num;
    base_pos_t base_snoars[10];
    uint8_t        base_bumper_num;
    base_pos_t base_bumpers[4];
    uint8_t        base_drop_num;
    base_pos_t base_drops[4];
    uint8_t        base_lidar_num;
    base_pos_t base_lidar[2];
    base_pos_t base_camera;
} __attribute((packed));

struct imu_data_t
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t roll_rate;
    int16_t pitch_rate;
    int16_t yaw_rate;
    int16_t xacc;
    int16_t yacc;
    int16_t zacc;
} __attribute((packed));

struct CONFIG_MSG
{
  uint8_t lip[4];
  uint8_t sub[4];
  uint8_t gw[4];
  uint8_t dns[4];
  uint8_t dhcp;
  uint8_t state;
  uint8_t pc_udp_rip[4];
  uint8_t lidar_udp_rip[4];
  uint8_t debug_udp_rip[4];
  uint16_t tcp_local_port;
  uint16_t pc_udp_port;
  uint16_t lidar_udp_port;
  uint16_t debug_udp_port;
} __attribute((packed));

struct config_param_t
{
    infoStruct_t    chassisInfo;  //底盘产品信息
    ChargeParamStruct_t	  ChargeParam;	//充电参数
    base_conf_response_t  base_conf;//底盘的配置信息
    PIDParamStruct_t LSpeedPID;//速度pid
    PIDParamStruct_t RSpeedPID;//速度pid
    CONFIG_MSG eth_param;//网络参数
} __attribute((packed));

struct encoder_data_t
{
    int32_t lencoder;
    int32_t rencoder;

} __attribute((packed));

struct sensor_data_t
{
    uint8_t bumper;
    uint8_t drop;
    uint8_t sonar[10];
} __attribute((packed));


struct ekf_data_t
{
    float  ekf_x;
    float  ekf_y;
    float  ekf_dethe;
    float  ekf_vx;
    float  ekf_vy;
    float  ekf_vw;
} __attribute((packed));



struct charge_data_t
{
    uint8_t  charge_type;
    uint8_t  battary_percent;
    uint8_t  recharge_status;//枚举变量会被认为是4个字节
    uint8_t   stop_status;
    uint16_t  bat_vol;
    uint16_t  charge_vol;
} __attribute((packed));



/**
  * @brief  error information
  */
struct chassis_err_t
{
  bottom_err_e err_sta;                 /* bottom error state */
  bottom_err_e err[ERROR_LIST_LENGTH];  /* device error list */
} __attribute((packed));

struct ans_data_t
{
    uint8_t  infra[4];
    float  speed[2];
} __attribute((packed));


struct speed_data_t
{
    float vx;
    float vw;

} __attribute((packed));

struct spd_data_t
{
    int16_t vx;//uint:mm/s
    int16_t vy;//uint:mm/s
    int16_t vw;//uint:mrad/s
} __attribute((packed));
/********* variables **********/
/**
  * @brief  the data structure send to pc
  */
typedef struct
{
  /* data send */
  config_param_t    config_param;
  imu_data_t        imu_data;
  encoder_data_t    encoder_data;
  sensor_data_t     sensor_data;
  ekf_data_t        ekf_data;
  charge_data_t     charge_data;
  chassis_err_t    bottom_error_data;
  ans_data_t      ans_data;
} send_pc_t;

/**
  * @brief  frame header structure definition
  */
struct frame_header_t
{
  uint8_t  header1;
  uint8_t  header2;
  uint16_t data_length;
} __attribute((packed));

#ifdef _WIN32
#pragma pack()
#endif
typedef enum
{
  STEP_HEADER_HEADER1  = 0,
  STEP_HEADER_HEADER2  = 1,
  STEP_LENGTH_LOW  = 2,
  STEP_LENGTH_HIGH = 3,
  STEP_HEADER_CMD  = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

#define HEADER1_ID  0xff
#define HEADER2_ID  0xaa
#define HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      1    //cmd id bytes
#define CRC_LEN      2    //crc16 bytes
#define PROTOCAL_FRAME_MAX_SIZE 500

struct unpack_data_t
{
  fifo      *data_fifo;
  uint16_t       data_len;
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
  uint8_t  unpack_step;
  uint16_t       index;
} ;

enum {
    DEFAULT_TIMEOUT = 2000,
    DEFAULT_HEART_BEAT = 1000,
    MAX_SCAN_NODES = 2048,
};

struct pos_data_t
{
    float  pos_x;
    float  pos_y;
    float  pos_dethe;
    float  pos_vx;
    float  pos_vy;
    float  pos_vw;
};



using namespace std;
using namespace serial;




namespace smart{

    class SmartDriver
    {
    public:

        result_t connect(const char * port_path, uint32_t baudrate, int drivertype = DRIVER_TYPE_SERIALPORT, char parity = 0);
        void disconnect();
        bool isconnecting(){
            return isConnected;
        }

        bool setSaveParse(bool parse, const std::string& filename);

        pos_data_t * get_encoder_process_data(void);
        static std::map<std::string, std::string> dashPortList();
        static std::string getSDKVersion(){return SDKVerision;}
        send_pc_t* get_pc_send_data(void){return pc_send_mesg;}
        void set_speed(float vx,float vy,float vw);
        void cmd_loop();
        result_t smart_init(uint32_t timeout= DEFAULT_TIMEOUT);
        void setDTR();
        void clearDTR();

        SmartDriver();
        virtual ~SmartDriver();

        result_t sendCommand(uint8_t cmd,bool flag = 1, const void * payload = NULL, size_t payloadsize = 0);
        result_t getData(uint8_t * data, size_t size,uint32_t timeout = DEFAULT_TIMEOUT);
        result_t sendData(const uint8_t * data, size_t size);
        void set_data_rate(uint16_t rate);
        void set_automatic_recharge(uint8_t temp);
        result_t ResetEncoder(uint32_t timeout= DEFAULT_TIMEOUT);
        version_t * getVersion(void){return &version_data;}
    protected:

    private:
        volatile bool    isConnected;  //通讯连接是否建立  //std::atomic<bool>
        volatile bool    isRevTimeout; //是否接受数据超时
        volatile bool     save_parsing; //是否将日志保存为文件
        volatile bool     isGetVersion;  //通讯连接是否建立
        volatile bool    isGetBaseParam;  //是否获取到底盘尺寸配置信息
        volatile bool    isResetIMU; //是否接受数据超时
        volatile bool    isResetEncoder; //是否将日志保存为文件
        volatile bool     isGetBaudrate;//是否获取波特率

        Event          _sendEvent;
        Event          _revEvent;
        Locker         _speed_lock;//速度变量在两个线程里都有用到，防止数据竞争
        Locker         _lock;
        Locker         _serial_lock;
        Thread 	       _send_thread;

        err_dectect err_det;

        ChannelDevice *_serial;

        int device_type; //指示此时使用串口连接还是网口，
        uint32_t _baudrate;//存储从yaml文件中获取的波特率参数
        uint32_t _rev_baudrate; //接受到的波特率

        pos_data_t * _encoder_pos_data;//用于存储由编码器计算得到的位置和速度

        fifo *_send_fifo;//发送数据队列
        fifo *_rev_fifo;//接受数据队列
        unpack_data_t *p_obj;

        version_t version_data;//底盘版本数据
        /*数据存储结构体*/
        send_pc_t  * pc_send_mesg;
        spd_data_t * spd_data;
        FILE *fd;
        uint16_t _data_rate;
        speed_data_t _speed_data;
        bool running;//指示驱动是否在运行


        result_t getBaudrate(uint32_t timeout= DEFAULT_TIMEOUT);
        uint8_t *protocol_packet_pack(uint8_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t *tx_buf);
        uint32_t send_packed_fifo_data();
        void pc_data_handler(uint8_t *p_frame);
        void unpack_fifo_data(unpack_data_t *p_obj);
        int revDataThread();
        int sendCmdThread();
        result_t getPlatformVersion(uint32_t timeout= DEFAULT_TIMEOUT);
        result_t ResetIMU(uint32_t timeout= DEFAULT_TIMEOUT);

        result_t getPlatformParam(uint32_t timeout= DEFAULT_TIMEOUT);

		/**
		* @brief 等待激光数据包头 \n
    	* @param[in] header 	 包头
    	* @param[in] timeout      超时时间	
		* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
		* @retval RESULT_TIMEOUT  等待超时
    	* @retval RESULT_FAILE    获取失败	
		* @note 当timeout = -1 时, 将一直等待
    	*/
		result_t waitResponseHeader(frame_header_t * header, uint32_t timeout = DEFAULT_TIMEOUT);
		/**
		* @brief 等待固定数量串口数据 \n
    	* @param[in] data_count 	 等待数据大小
    	* @param[in] timeout    	 等待时间	
		* @param[in] returned_size   实际数据大小	
		* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
		* @retval RESULT_TIMEOUT  等待超时
    	* @retval RESULT_FAILE    获取失败	
		* @note 当timeout = -1 时, 将一直等待
    	*/
        result_t waitForData(size_t data_count,uint32_t timeout = DEFAULT_TIMEOUT, size_t * returned_size = NULL);

        result_t getPlatformData(uint8_t cmd_id,uint32_t timeout = DEFAULT_TIMEOUT);

        void encoder_process_handler(float l_right,float l_left,uint32_t now_millis,pos_data_t * pos_data);





    };
}

#endif // SMART_DRIVER_H
