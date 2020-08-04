/*
*  SMART SYSTEM
*  SMART DRIVER
*
*  Copyright 2019 - 2020 EAI TEAM
*  http://www.eaibot.com
* 
*/
#include "common.h"
#include "smart_driver.h"
#include "ActiveSocket.h"
#include <math.h>
#include "crc16.h"
#include "data_fifo.h"
#include "Console.h"
#include "detect_task.h"
#include "SimpleSocket.h"


using namespace impl;


namespace smart{

    SmartDriver::SmartDriver():
	_serial(nullptr) {
        isConnected = false;

        _baudrate = 115200;
        _rev_baudrate = 0;

        save_parsing = false; //是否生成log文件
        isConnected = false;  //通讯连接是否建立
        isRevTimeout = false; //是否接受数据超时
        isGetVersion = false;  //通讯连接是否建立
        isResetIMU = false; //是否IMU复位成功
        isResetEncoder = false; //是否码盘复位成功
        isGetBaudrate  = false; //是否获取波特率，主要用来检测通讯是否建立了

        _encoder_pos_data = new pos_data_t;
        memset(_encoder_pos_data,0,sizeof(pos_data_t));
        /*fifo declare*/
        _send_fifo = new fifo;
        _rev_fifo = new fifo;
        /*object for unpacking fifo data*/
        p_obj = new unpack_data_t;
        memset(p_obj,0,sizeof(unpack_data_t));
        p_obj->data_fifo = _send_fifo;
        p_obj->unpack_step = STEP_HEADER_HEADER1;
        /*data struct declare*/
        pc_send_mesg = new send_pc_t;
        memset(pc_send_mesg,0,sizeof(send_pc_t));
        spd_data = new spd_data_t;
        memset(spd_data,0,sizeof(spd_data_t));
        _data_rate = 6;
        fd = nullptr;//log file
	}

    SmartDriver::~SmartDriver(){

		_send_thread.join();

        ScopedLocker lock(_serial_lock);
		if(_serial){
			if(_serial->isOpen()){
                _serial->flush();
                _serial->closefd();
			}
		}
		if(_serial){
			delete _serial;
                        _serial = nullptr;
		}
        if (nullptr != fd)
            fclose(fd);
        delete p_obj;
        delete _send_fifo;
        delete _rev_fifo;
        delete pc_send_mesg;
        delete _encoder_pos_data;
	}

    result_t SmartDriver::connect(const char * port_path, uint32_t baudrate, int drivertype, char parity) {
		_baudrate = baudrate;
        device_type = (uint8_t)drivertype;
        {
            ScopedLocker lock(_serial_lock);
            switch (device_type) {
            case DRIVER_TYPE_SERIALPORT:
                if(!_serial){
                    _serial = new serial::Serial(port_path,baudrate,serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
                    // _serial->bindport(port_path, baudrate);
                }
                break;
            // case DRIVER_TYPE_TCP://测试发现下位机会主动发连接复位的数据包
            //     if(!_serial) {
            //         _serial = new smart::CActiveSocket(CSimpleSocket::CSocketType::SocketTypeTcp,port_path, (uint16_t)baudrate);
            //     }
            //     break;
            case DRIVER_TYPE_UDP:
                if(!_serial) {
                    _serial = new smart::CActiveSocket(CSimpleSocket::CSocketType::SocketTypeUdp,port_path, (uint16_t)baudrate);

                }
                break;
            default:
                smart::console.error( "Device type is not correct.\n");
                return RESULT_FAIL;
                break;
            }
            

        }

        if(!_serial->open()){
            return RESULT_FAIL;
        }

        isConnected = true;

        if(parity){
            _serial->setParity(serial::parity_even);
        }
		return RESULT_OK;
	}

    void SmartDriver::setDTR() {
		if (!isConnected){
			return ;
		}
        ScopedLocker lock(_serial_lock);

		if(_serial){
            _serial->flush();
			_serial->setDTR(1);
		}

	}
    //TODO：是否需要日志文件
    //是否建立日志文件
    bool SmartDriver::setSaveParse(bool parse, const std::string& filename) {
        bool ret = false;
        save_parsing = parse;
        if(save_parsing) {
            if(fd == nullptr){
                 fd=fopen("smartdriverlog.txt","w");
                 ret = true;
                 if (nullptr == fd){
                     fd =fopen(filename.c_str(), "w");
                     ret = false;
                 }
            }
        } else {
            if(fd != nullptr) {
                fclose(fd);
            }
        }

        return ret;
    }

    void SmartDriver::clearDTR() {
		if (!isConnected){
			return ;
		}
        ScopedLocker lock(_serial_lock);
		if(_serial){
            _serial->flush();
			_serial->setDTR(0);
		}
	}

    void SmartDriver::disconnect() {
                running = false;
		if (!isConnected){
			return ;
		}
        ScopedLocker lock(_serial_lock);
		if(_serial){
			if(_serial->isOpen()){
                _serial->flush();
                _serial->closefd();
			}
		}
		isConnected = false;
	}

    //组包
    uint8_t* SmartDriver::protocol_packet_pack(uint8_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t *tx_buf)
    {

      uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
      frame_header_t *p_header = (frame_header_t*)tx_buf;

      p_header->header1 = HEADER1_ID;
      p_header->header2 = HEADER2_ID;
      p_header->data_length  = len+1;//长度需加上命令码

      memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
      if(p_data != nullptr)
        memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);
      append_crc16_check_sum(tx_buf, frame_length);

      return tx_buf;
    }

    //sendCommand(Platform_HexCmd_Set_MotorSpeed, (uint8_t*)&SpeedValue, sizeof(speed_data_t));使用示例 TODO:去掉flag
    result_t SmartDriver::sendCommand(uint8_t cmd_id,bool flag, const void *p_data, size_t len)
    {
      uint8_t tx_buf[PROTOCAL_FRAME_MAX_SIZE];

      uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
      if (!isConnected) {
          return RESULT_FAIL;
      }
      protocol_packet_pack(cmd_id, (uint8_t *)p_data, len, tx_buf);

      if(flag)
          sendData(tx_buf, frame_length);
      else
      {
          _send_fifo->puts(tx_buf, frame_length);
          _sendEvent.set();

      }

      return RESULT_OK;
    }

    result_t SmartDriver::sendData(const uint8_t * data, size_t size) {
        {
		if (!isConnected) {
			return RESULT_FAIL;
		}
        }

        if (data == nullptr || size ==0) {
                return RESULT_FAIL;
        }
        size_t r;
        // while (size) {
            {
                ScopedLocker lock(_serial_lock);
                if(_serial)
                    r = _serial->writedata(data, size);
                else
                    return RESULT_FAIL;
            }

            if (r < 1)
                return RESULT_FAIL;
        //     size -= r;
        //     data += r;
        // }
        return RESULT_OK;
	}

    result_t SmartDriver::getData(uint8_t * data, size_t size,uint32_t timeout) {
        {
            if (!isConnected) {
                    return RESULT_FAIL;
            }
        }
        uint32_t startTs = getms();

        int r;
        while (size) {
            if(getms() - startTs > timeout)
            {
                return RESULT_FAIL;
            }
            {
                ScopedLocker lock(_serial_lock);
                if(_serial)
                {
                    r = _serial->readdata(data, size);
                    // printf("r=%d\r\n",(int)r);
                }
                else
                    return RESULT_FAIL;
            }
            if (r < 1)
            {
                
            }
            else
            {
                // for(uint8_t i = 0;i<r;++i)
                //     printf("data=0x%X\r\n",data[i]);
                size -= r;
                data += r;            
            }            
        }  
        return RESULT_OK;
	}

    //fifo数据发送
    uint32_t SmartDriver::send_packed_fifo_data(void)
    {

      uint8_t  tx_buf[PROTOCAL_FRAME_MAX_SIZE];

      uint32_t fifo_count = _send_fifo->used_count();

      if (fifo_count)
      {
        _send_fifo->gets(tx_buf, fifo_count);
        sendData(tx_buf, fifo_count);

      }

      return fifo_count;
    }

    //数据解析后复制给相应的数据结构体
    void SmartDriver::pc_data_handler(uint8_t *p_frame)
    {
//      uint32_t temp_data;
//      int16_t temp16_data[2];
//      uint8_t inc = 0;
      frame_header_t *p_header = (frame_header_t*)p_frame;
      memcpy(p_header, p_frame, HEADER_LEN);

      uint16_t data_length = p_header->data_length - CMD_LEN;
      uint8_t cmd_id      = *(uint8_t *)(p_frame + HEADER_LEN);
      uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;

      switch (cmd_id)
      {

        case Platform_HexCmd_Get_Bdrate: //命令码:FFAA01000042EF
          memcpy(&_rev_baudrate,data_addr, sizeof(uint32_t));
          isGetBaudrate = true;
        break;

        case Platform_HexCmd_Get_PC_BaseConfig: //此命令是PC用来读取底盘配置信息的  命令码:FFAA01002A6A6A
          memcpy(&pc_send_mesg->config_param.base_conf,data_addr, sizeof(base_conf_response_t));
          isGetBaseParam = true;
        break;

        case Platform_HexCmd_Get_Gyro://命令码:FFAA010005E7BF
        memcpy(&pc_send_mesg->imu_data, data_addr, data_length);

        err_det.err_detector_hook(GYRO_OFFLINE);
        break;

        case Platform_HexCmd_Clear_Encoder:
            isResetEncoder = true;
        break;

        case Platform_HexCmd_Reset_Gyro:
            isResetIMU = true;
        break;

        case Platform_HexCmd_Get_Encoder:    //命令码:FFAA 0100 02 00CF
        memcpy(&pc_send_mesg->encoder_data, data_addr, data_length);
        encoder_process_handler(pc_send_mesg->encoder_data.rencoder,pc_send_mesg->encoder_data.lencoder,
                                                          getms(),_encoder_pos_data);
        err_det.err_detector_hook(ENCODER_OFFLINE);
        break;

        case Platform_HexCmd_Get_SpeedValue:    //命令码:FFAA 0100 02 00CF
        memcpy(&_speed_data, data_addr, data_length);
        _encoder_pos_data->pos_vx = _speed_data.vx*0.001f;
        _encoder_pos_data->pos_vw = _speed_data.vw*0.001f;

        err_det.err_detector_hook(ENCODER_OFFLINE);
        break;

        case Platform_HexCmd_Get_AllSensor://命令码:FFAA 0100 0D EF3E
        memcpy(&pc_send_mesg->sensor_data, data_addr, data_length);
        err_det.err_detector_hook(ALLSENSORS_OFFLINE);
        break;

        case Platform_HexCmd_Get_Recharge://命令码:FFAA 0100 11 52ED
        memcpy(&pc_send_mesg->charge_data, data_addr, data_length);
        err_det.err_detector_hook(RECHARGE_OFFLINE);
        break;

        case Platform_HexCmd_Set_Speed://命令码:
        err_det.err_detector_hook(SPEED_OFFLINE);
        break;

        case Platform_HexCmd_Get_Version://命令码:FFAA 0100 2F CF3A
//        memcpy(&version_data.version_info[0], data_addr, data_length);TODO:测试是否OK
        for(uint8_t i=0;i<data_length;i++)
        {
            version_data.version_info[i] = static_cast<char>(data_addr[i]);
        }
        isGetVersion = true;
        break;

        default:
            break;

      }

    }


    //解析fifo数据包
    void SmartDriver::unpack_fifo_data(unpack_data_t *p_obj)
    {
      uint8_t byte = 0;
      uint8_t cmd_id = 0;
      //DBG_OUT("[unpack fifo]fifo_count = %d \r\n",fifo_used_count(p_obj->data_fifo));
      while ( p_obj->data_fifo->used_count())
      {

        byte = p_obj->data_fifo->get();
        switch(p_obj->unpack_step)
        {
          case STEP_HEADER_HEADER1:
          {
            if(byte == HEADER1_ID)
            {
              p_obj->unpack_step = STEP_HEADER_HEADER2;
              p_obj->protocol_packet[p_obj->index++] = byte;
            }
            else
            {
              p_obj->index = 0;
            }
          }break;
          case STEP_HEADER_HEADER2:
          {
            if(byte == HEADER2_ID)
            {
              p_obj->unpack_step = STEP_LENGTH_LOW;
              p_obj->protocol_packet[p_obj->index++] = byte;
            }
            else
            {
              p_obj->index = 0;
            }
          }break;
          case STEP_LENGTH_LOW:
          {
            p_obj->data_len = byte;
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_LENGTH_HIGH;
          }break;

          case STEP_LENGTH_HIGH:
          {
            p_obj->data_len |= (byte << 8);
            p_obj->protocol_packet[p_obj->index++] = byte;

            if(p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
            {
              p_obj->unpack_step = STEP_HEADER_CMD;
            }
            else
            {
              p_obj->unpack_step = STEP_HEADER_HEADER1;
              p_obj->index = 0;
            }
          }break;

          case STEP_HEADER_CMD:
          {
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_DATA_CRC16;
            cmd_id = byte;
          }break;
          case STEP_DATA_CRC16:
          {
            if (p_obj->index < (HEADER_LEN + p_obj->data_len + CRC_LEN))
            {
               p_obj->protocol_packet[p_obj->index++] = byte;
            }
            if (p_obj->index >= (HEADER_LEN + p_obj->data_len + CRC_LEN))
            {
              p_obj->unpack_step = STEP_HEADER_HEADER1;
              p_obj->index = 0;

              if ( verify_crc16_check_sum(p_obj->protocol_packet, HEADER_LEN + p_obj->data_len + CRC_LEN) )
              {
                pc_data_handler(p_obj->protocol_packet);
              }
              else
              {
                smart::console.error("[protocol]error:crc16 dismatch[%2x].",cmd_id);
              }
            }
          }break;

          default:
          {
            p_obj->unpack_step = STEP_HEADER_HEADER1;
            p_obj->index = 0;
          }break;
        }
      }
    }



    //数据发送线程
    int SmartDriver::sendCmdThread(void) //线程函数的返回值必须要是int类型
    {
        static uint8_t excute_count;
        smart::console.show( "sendCmdThread is running successfully.\n");
    //    static uint32_t last_ms,now_ms,error_ms;
        while(running){
            //用于查看循环周期
        //    now_ms = getms();
        //    error_ms = now_ms - last_ms;
        //    last_ms = now_ms;
        //    smart::console.show("error_ms=%d ms",error_ms);

            cmd_loop();
            excute_count++;
            if(excute_count >= 10){
                excute_count = 0;
                err_det.detect_task();
            }

            delay_ms(_data_rate);
            // std::this_thread::sleep_for(std::chrono::microseconds(30)); //该方法延时不准确
//            switch (_sendEvent.wait()) {
//            case Event::EVENT_TIMEOUT:

//                break;
//            case Event::EVENT_OK:
//                {
//                    send_packed_fifo_data();
//                }
//                break;
//            default:

//                break;
//            }
        }
        int socket_error = _serial->get_serial_error();
        smart::console.show( "sendCmdThread is shutting down.[%d]\n",socket_error);
        return -1;
    }


    //等待包头数据
	result_t SmartDriver::waitResponseHeader(frame_header_t * header, uint32_t timeout) {
		int  recvPos = 0;
		uint32_t startTs = getms();
		uint8_t  recvBuffer[sizeof(frame_header_t)];
		uint8_t  *headerBuffer = reinterpret_cast<uint8_t *>(header);
		uint32_t waitTime;

		while ((waitTime=getms() - startTs) <= timeout) {
			size_t remainSize = sizeof(frame_header_t) - recvPos;
			size_t recvSize;

			result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);
			if (ans != RESULT_OK){
				return ans;
			}

			if(recvSize > remainSize) recvSize = remainSize;

			ans = getData(recvBuffer, recvSize);
			if (ans == RESULT_FAIL){
				return RESULT_FAIL;
			}

			for (size_t pos = 0; pos < recvSize; ++pos) {
				uint8_t currentByte = recvBuffer[pos];
				switch (recvPos) {
				case 0:
					if (currentByte != HEADER1_ID) {
						continue;
					}
					break;
				case 1:
					if (currentByte != HEADER2_ID) {
						recvPos = 0;
						continue;
					}
					break;
				}
				headerBuffer[recvPos++] = currentByte;

				if (recvPos == sizeof(frame_header_t)) {
					return RESULT_OK;
				}
			}
		}
		return RESULT_FAIL;
	}

	result_t SmartDriver::waitForData(size_t data_count, uint32_t timeout, size_t * returned_size) {
		size_t length = 0;
		if (returned_size==NULL) {
			returned_size=(size_t *)&length;
		}
        ScopedLocker lock(_serial_lock);
		return (result_t)_serial->waitfordata(data_count, timeout, returned_size);
	}

#if 0
    //针对TCP，经测试通讯并不稳定.通讯会被下位机复位而中断
    result_t SmartDriver::getPlatformData(uint8_t cmd_id,uint32_t timeout){
        result_t ans = RESULT_OK;
        if (!isConnected) {
                return RESULT_FAIL;
        }

        {
            ScopedLocker lock(_lock);
            if(cmd_id == Platform_HexCmd_Set_Speed){
                {
                    ScopedLocker lock(_speed_lock);
                    if ((ans = sendCommand(Platform_HexCmd_Set_Speed,1,spd_data,sizeof(spd_data_t))) != RESULT_OK)
                    {
                        return ans;
                    }
                }
            }
            else{
                if ((ans = sendCommand(cmd_id)) != RESULT_OK)
                {
                    return ans;
                }
            }
            //用TCP时一个数据包可以分次获取
            // frame_header_t response_header;
            // if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
            //     printf("header error[%d][%d]\r\n",cmd_id,ans);
            //     memset(&response_header,0,sizeof(frame_header_t));
            //     return ans;
            // }
            // p_obj->data_fifo->puts(reinterpret_cast<uint8_t*>(&response_header),sizeof(frame_header_t));
            // size_t count = response_header.data_length+CRC_LEN;
            // // printf("come through0[%d]\r\n",cmd_id);
            // if (waitForData(count, timeout) != RESULT_OK) {
            //     // printf("context error[%d]\r\n",cmd_id);
            //     return RESULT_FAIL;
            // }
            // uint8_t buffer[count];
            // getData(buffer, count);
            // p_obj->data_fifo->puts(buffer,count);

            // unpack_fifo_data(p_obj);
            // printf("come through[%d]\r\n",cmd_id);
        }
        return RESULT_OK;
    }

#else
    //针对UDP的收发
    result_t SmartDriver::getPlatformData(uint8_t cmd_id,uint32_t timeout){
        result_t ans = RESULT_OK;
        if (!isConnected) {
                return RESULT_FAIL;
        }

        {
            ScopedLocker lock(_lock);
            if(cmd_id == Platform_HexCmd_Set_Speed){
                {
                    ScopedLocker lock(_speed_lock);
                    if ((ans = sendCommand(Platform_HexCmd_Set_Speed,1,spd_data,sizeof(spd_data_t))) != RESULT_OK)
                    {
                        return ans;
                    }
                }
            }
            else{
                if ((ans = sendCommand(cmd_id)) != RESULT_OK)
                {
                    return ans;
                }
            }
            //使用UDP必须一次性获取所有数据
            size_t count = 0;
            switch (cmd_id)
            {
            case Platform_HexCmd_Get_Bdrate:
                count = 11;
                break;
            case Platform_HexCmd_Get_Encoder:
                count = 15;
                break;
            case Platform_HexCmd_Clear_Encoder:
                count = 7;
                break;
            case Platform_HexCmd_Set_Speed:
                count = 7;
                break;
            case Platform_HexCmd_Get_Gyro:
                count = 25;
                break;
            case Platform_HexCmd_Reset_Gyro:
                count = 7;
                break;
            case Platform_HexCmd_Get_AllSensor:
                count = 19;
                break;
            case Platform_HexCmd_Set_Recharge:
                count = 7;
                break;
            case Platform_HexCmd_Get_Recharge:
                count = 15;
                break;
                
            case Platform_HexCmd_Get_Version:
                count = 25;
                break;
            case Platform_HexCmd_Get_SpeedValue:
                count = 15;
                break;
                
            case Platform_HexCmd_Get_PC_BaseConfig:
                count = 264;
                break;


            default:
                break;
            }


            uint8_t buffer[count];
            getData(buffer, count,timeout);
            p_obj->data_fifo->puts(buffer,count);

            unpack_fifo_data(p_obj);
            
        }
        return RESULT_OK;
    }
#endif

    result_t SmartDriver::getBaudrate(uint32_t timeout) {
        result_t ans;

        if ((ans = getPlatformData(Platform_HexCmd_Get_Bdrate,timeout)) != RESULT_OK)
        {
            return ans;
        }

        if (isGetBaudrate == false)
        {
            smart::console.error("timeout when getting baudrate.\n");
            return RESULT_FAIL;
        }

        smart::console.show( "connect smart platform successful.\n");

        return RESULT_OK;
	}

    result_t SmartDriver::getPlatformVersion(uint32_t timeout) {
        result_t ans;

        if ((ans = getPlatformData(Platform_HexCmd_Get_Version)) != RESULT_OK)
        {
            return ans;
        }

        if (isGetVersion == false)
        {
            smart::console.error("timeout when getting PlatformVersion.\n");
            return RESULT_FAIL;
        }

        smart::console.show("get Platform Version successful.Version :%s\n",version_data);

        return RESULT_OK;
    }

    result_t SmartDriver::ResetIMU(uint32_t timeout) {
        result_t ans;

        if ((ans = getPlatformData(Platform_HexCmd_Reset_Gyro)) != RESULT_OK)
        {
            return ans;
        }

        if (isResetIMU == false)
        {
            smart::console.error("timeout when Resetting IMU.\n");
            return RESULT_FAIL;
        }
        smart::console.show("Reset IMU successful.\n");

        return RESULT_OK;
    }

    result_t SmartDriver::ResetEncoder(uint32_t timeout) {
        result_t ans;

        if ((ans = getPlatformData(Platform_HexCmd_Clear_Encoder)) != RESULT_OK)
        {
            return ans;
        }

        if (isResetEncoder == false)
        {
            smart::console.error( "timeout when Reset Encoder.\n");
            return RESULT_FAIL;
        }
        memset(_encoder_pos_data,0,sizeof(pos_data_t));
        smart::console.show("Reset Encoder successful.\n");

        return RESULT_OK;
    }

    result_t SmartDriver::getPlatformParam(uint32_t timeout) {
        result_t ans;

        if ((ans = getPlatformData(Platform_HexCmd_Get_PC_BaseConfig)) != RESULT_OK)
        {
            return ans;
        }

        if (isGetBaseParam == false)
        {
            smart::console.error("timeout when getting BaseParam.\n");
            return RESULT_FAIL;
        }

        smart::console.show("Get BaseParam successful.\n");

        return RESULT_OK;
    }

    result_t SmartDriver::smart_init(uint32_t timeout)
    {

        running = true; //用于退出线程

        if(getBaudrate(timeout) != RESULT_OK)
        {
            return RESULT_FAIL;
        }

        if(getPlatformVersion(timeout) != RESULT_OK)
        {
            return RESULT_FAIL;
        }

        if(ResetEncoder(timeout) != RESULT_OK)
        {
            return RESULT_FAIL;
        }

        if(ResetIMU(timeout*2) != RESULT_OK)//复位陀螺仪需要等待多点时间
        {
            return RESULT_FAIL;
        }

        if(getPlatformParam(timeout) != RESULT_OK)
        {
            return RESULT_FAIL;
        }
        _send_thread = CLASS_THREAD(SmartDriver, sendCmdThread);//开启发送线程
        if (_send_thread.getHandle() == 0) {
            smart::console.error("error when creating send-cmd threads.\n");
            return RESULT_FAIL;
        }

        
        return RESULT_OK;
    }


    void SmartDriver::cmd_loop(void)
    {
        static uint8_t cmd_step;

        switch (cmd_step) {
            case 0:
                getPlatformData(Platform_HexCmd_Get_Gyro);
                cmd_step = 1;
                break;

            case 1:
                getPlatformData(Platform_HexCmd_Set_Speed);
                cmd_step = 2;
                break;

            case 2:
                getPlatformData(Platform_HexCmd_Get_Encoder);
                cmd_step = 3;
                break;

            case 3:
                getPlatformData(Platform_HexCmd_Get_AllSensor);
                cmd_step = 4;
                break;

            case 4:
                getPlatformData(Platform_HexCmd_Get_Recharge);
                cmd_step = 0;
                break;

            case 5:
                getPlatformData(Platform_HexCmd_Get_SpeedValue);
                cmd_step = 0;
                break;
            default:
                break;
        }
        // printf("step=%d\r\n",cmd_step);
    }

    void SmartDriver::encoder_process_handler(float l_right,float l_left,uint32_t now_millis,pos_data_t * pos_data)
    {
        float dxy_ave,dth,dright,dleft,dx,dy;
        float dt;
        static uint32_t pre_millis;//previous time of computation
        static float pre_l_right;//previous right encoder value in meter
        static float pre_l_left;//previous left encoder value in meter
        uint32_t dt_int;
        dright = l_right - pre_l_right;
        dleft = l_left - pre_l_left;

        /*save data for next computation*/
        pre_l_right = l_right;
        pre_l_left = l_left;
        
        dxy_ave = (dright + dleft) * 0.0005f;//0.0005 = 1/1000/2 转换单位为m
        dth = 1.0f*(dright - dleft) / pc_send_mesg->config_param.base_conf.distance_between_wheels_mm;//需要知道轮间距wheel_diameter_mm

        /*get time duration in sec*/
        dt = (now_millis - pre_millis) / 1000.0f;
        dt_int = now_millis - pre_millis;
        pre_millis = now_millis;
//        printf("dt=%f dt_int=%d \n\r",dt,dt_int);
        pos_data->pos_vx = dxy_ave / dt;
        pos_data->pos_vw = dth / dt;

        if (dxy_ave != 0)
        {
            dx = cos(dth/2.0) * dxy_ave;     
            dy = sin(dth/2.0) * dxy_ave;    
            pos_data->pos_x += (cos(pos_data->pos_dethe) * dx - sin(pos_data->pos_dethe) * dy);        
            pos_data->pos_y += (sin(pos_data->pos_dethe) * dx + cos(pos_data->pos_dethe) * dy);

        }
                
        if (dth != 0) 
        {
            pos_data->pos_dethe += dth;
        }
    }

    pos_data_t * SmartDriver::get_encoder_process_data(void)
    {
        return _encoder_pos_data;
    }


    std::map<std::string, std::string>  SmartDriver::dashPortList() {
        std::vector<PortInfo> lst = list_ports();
        std::map<std::string, std::string> ports;

        for (std::vector<PortInfo>::iterator it = lst.begin(); it != lst.end(); it++) {
            std::string port = "dash" + (*it).device_id;
            ports[port] = (*it).port;
        }

        return ports;
    }

    //set speed value in m/s
    void SmartDriver::set_speed(float vx,float vy,float vw)
    {
        ScopedLocker l(_speed_lock);
        spd_data->vx = (int16_t)(vx*1000);
        spd_data->vy = (int16_t)(vy*1000);
        spd_data->vw = (int16_t)(vw*1000);      
    }
    //set data update rate in hz
    void SmartDriver::set_data_rate(uint16_t rate)
    {
        _data_rate = 200/rate;
    }

    void SmartDriver::set_automatic_recharge(uint8_t temp)
    {
        uint8_t data = temp;
        ScopedLocker lock(_lock);
        sendCommand(Platform_HexCmd_Set_Recharge,1,&data,1);

    }

}



