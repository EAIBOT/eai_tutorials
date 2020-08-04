
#include "detect_task.h"


err_dectect::err_dectect()
{
    detector_param_init();
}

err_dectect::~err_dectect()
{

}
/**
  * @brief     initialize detector error_list
  * @usage     used before detect loop in detect_task() function
  */
void err_dectect::global_err_detector_init(void)
{
  g_err.err_now = nullptr;
  g_err.list[BOTTOM_DEVICE].dev    = nullptr;
  g_err.list[BOTTOM_DEVICE].enable = 0;

  /* initialize device error type and offline timeout value */
  for (uint8_t i = ENCODER_OFFLINE; i < ERROR_LIST_LENGTH; i++)
  {
    if (i <= RECHARGE_OFFLINE)
    {
      offline_dev[i].set_timeout = 1000; //ms
      offline_dev[i].last_time   = 0;
      offline_dev[i].delta_time  = 0;
      
      g_err.list[i].dev  = &offline_dev[i];
      g_err.list[i].type = DEV_OFFLINE;
      g_err.list[i].err_exist  = 0;
      g_err.list[i].pri        = i;//set priority of errors
      g_err.list[i].enable     = 1;
    }
    
  }
    
}

/**
  * @brief     record the detected module return time to judge offline
  * @param     err_id: module id
  * @retval    None
  * @usage     used in CAN/usart.. rx interrupt callback
  */
void err_dectect::err_detector_hook(int err_id)
{
  if (g_err.list[err_id].enable)
      g_err.list[err_id].dev->last_time = getms();
}

void err_dectect::detector_param_init(void)
{
  global_err_detector_init();
}

/**
  * @brief     according to the interval time
  * @param     err_id: module id
  * @retval    None
  * @usage     used in CAN/usart.. rx interrupt callback
  */

int err_dectect::detect_task(void)
{

//  while(1)
//  {

    /* module offline detect */
    module_offline_detect();

    if (g_err.err_now != nullptr)
    {
      
      module_offline_callback();
    }
    else
    {

    }

//    delay_ms(DETECT_TASK_PERIOD);//当考虑作为一个线程执行时，取消注释
//  }
//  return -1;
}

void err_dectect::module_offline_detect(void)
{
  int max_priority = 0;
  int err_cnt      = 0;
  for (uint8_t id = ENCODER_OFFLINE; id <= RECHARGE_OFFLINE; id++)
  {
    g_err.list[id].dev->delta_time = getms() - g_err.list[id].dev->last_time;
    
    if (g_err.list[id].enable 
        && (g_err.list[id].dev->delta_time > g_err.list[id].dev->set_timeout))
    {
      g_err.list[id].err_exist = 1; //this module is offline
      err_cnt++;
      if (g_err.list[id].pri > max_priority)//get which error is max priority
      {
        max_priority     = g_err.list[id].pri;
        g_err.err_now    = &(g_err.list[id]);
        g_err.err_now_id = (err_id_e)id;
      }
    }
    else
    {
      g_err.list[id].err_exist = 0;
    }
  }

  if (!err_cnt)
  {
    g_err.err_now    = nullptr;
    g_err.err_now_id = BOTTOM_DEVICE;
  }
}



void err_dectect::module_offline_callback(void)
{
  g_err.err_count++;
  if (g_err.err_count > 50)
    g_err.err_count = 0;

  switch (g_err.err_now_id)
  {
    case ENCODER_OFFLINE:
    {
        smart::console.error("detect encoder offline timeout");
    }break; 

    case SPEED_OFFLINE:
    {
        smart::console.error("detect speed offline timeout");
    }break;

    case GYRO_OFFLINE:
    {
        smart::console.error("detect gyro offline timeout");
    }break;
    
    case ALLSENSORS_OFFLINE:
    {
        smart::console.error("detect all sensors offline timeout");
    }break;

    case RECHARGE_OFFLINE:
    {
        smart::console.error("detect recharge info offline timeout");
    }break;


    default:
    {

    }break;
  }
}




