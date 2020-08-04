
#ifndef __ERROR_TASK_H__
#define __ERROR_TASK_H__

#include <stdint.h>
#include <stdlib.h>
#include "locker.h"
#include "timer.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include "common.h"
#include "Console.h"

/*! detect task period time (ms) */
#define DETECT_TASK_PERIOD 50

typedef enum
{
  DEV_OFFLINE     = 0,
  DEV_RUNNING_ERR = 1,
  SYS_CONFIG_ERR  = 2,
} err_type_e;

typedef enum
{
  BOTTOM_DEVICE        = 0,
  ENCODER_OFFLINE      = 1,
  SPEED_OFFLINE        = 2,
  GYRO_OFFLINE = 3,
  ALLSENSORS_OFFLINE = 4,
  RECHARGE_OFFLINE   = 5,
  ERROR_LIST_LENGTH = 6,
} err_id_e;

typedef enum
{
  DEVICE_NORMAL = 0,
  ERROR_EXIST   = 1,
  UNKNOWN_STATE = 2,
} bottom_err_e;

typedef struct
{
  uint16_t set_timeout;
  /* uint32_t prevent delta_time overflow */
  uint32_t delta_time;
  uint32_t last_time;
} offline_dev_t;

typedef struct
{
  /*! enable the device error detect */
  uint8_t  enable;
  /*! device error exist flag */
  uint8_t  err_exist;
  /*! device error priority */
  uint8_t  pri;
  /*! device error type */
  uint8_t  type;
  /*! the pointer of device offline param */
  offline_dev_t *dev;
} err_dev_t;

typedef struct
{
  /*! the pointer of the highest priority error device */
  err_dev_t *err_now;
  err_id_e  err_now_id;
  /*! all device be detected list */
  err_dev_t list[ERROR_LIST_LENGTH];//每一个错误类型分配一个错误结构体

  /*! error alarm relevant */
  uint16_t err_count;

} global_err_t;


class err_dectect
{

    public:
        err_dectect();
        ~err_dectect();
        void err_detector_hook(int err_id);
        int detect_task(void);
    private:

        void global_err_detector_init(void);


        void detector_param_init(void);

        void module_offline_callback(void);
        void module_offline_detect(void);

        /*! detect task global parameter */
        global_err_t g_err;

        /*! detect task static parameter */
        offline_dev_t offline_dev[RECHARGE_OFFLINE + 1];

};







#endif
