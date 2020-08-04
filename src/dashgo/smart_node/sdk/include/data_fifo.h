 
#ifndef __DATA_FIFO_H__
#define __DATA_FIFO_H__

#include <stdlib.h>
#include <functional>
#include "locker.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>


#define ASSERT(x) do {while(!(x));} while(0)
 



//! FIFO Memory Model (Single Byte Mode)
struct fifo_s_t
{
  uint8_t   *start_addr;                   //Start Address
  uint8_t   *end_addr;                     //End Address
  uint32_t  free;                         //The capacity of FIFO
  uint32_t  buf_size;                     //Buffer size
  uint32_t  used;                         //The number of elements in FIFO
  uint16_t   read_index;                   //Read Index Pointer
  uint16_t   write_index;                  //Write Index Pointer

} ;


class fifo
{
public:
    fifo_s_t* create(uint32_t unit_cnt);
    fifo(uint32_t count = 1024);
    virtual ~fifo();
    int32_t put(uint8_t element);
    int32_t puts(uint8_t *psource, uint32_t number);

    uint8_t  get(void);
    uint16_t gets(uint8_t* source, uint32_t len);

    uint8_t  pre_read(uint8_t offset);
    uint8_t  is_empty(void);
    uint8_t  is_full(void);
    uint32_t used_count(void);
    uint32_t free_count(void);
    uint8_t  flush(void);
protected:

private:
    void init(fifo_s_t* pfifo, void* base_addr, uint32_t unit_cnt);

    void     destory(void);


    // MUTEX_WAIT(void) {_lock.lock();}
    // MUTEX_RELEASE(void) {_lock.unlock();}
    Locker   _lock;
    fifo_s_t *pfifo;
};

#define    MUTEX_WAIT()    _lock.lock()
#define     MUTEX_RELEASE() _lock.unlock()





#endif
