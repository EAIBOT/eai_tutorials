
#include "data_fifo.h"
#include "v8stdint.h"

fifo::fifo(uint32_t count)
{
  create(count);

}

fifo::~fifo()
{
  destory();
}

fifo_s_t* fifo::create(uint32_t unit_cnt)
{
  pfifo     = NULL;
  uint8_t  *base_addr = NULL;
  
  //! Check input parameters.
  ASSERT(0 != unit_cnt);

  //! Allocate Memory for pointer of new FIFO Control Block.
  pfifo = (fifo_s_t*) malloc(sizeof(fifo_s_t));
  if(NULL == pfifo)
  {
    //! Allocate Failure, exit now.
    return (NULL);
  }

  //! Allocate memory for FIFO.
  base_addr = (uint8_t*) malloc(unit_cnt);
  if(NULL == base_addr)
  {
    //! Allocate Failure, exit now.
    return (NULL);
  }

  this->init(pfifo, base_addr, unit_cnt);

  return (pfifo);
}

void fifo::destory(void)
{
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  ASSERT(NULL != pfifo->start_addr);

  //! free FIFO memory
  free(pfifo->start_addr);
  
  //! delete mutex

  
  //! free FIFO Control Block memory.
  free(pfifo);

  return;
}



void fifo::init(fifo_s_t* pfifo, void* base_addr, uint32_t unit_cnt)
{
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  ASSERT(NULL != base_addr);
  ASSERT(0    != unit_cnt);


  

    //! Initialize FIFO Control Block.
    pfifo->start_addr  = (uint8_t*) base_addr;
    pfifo->end_addr    = (uint8_t*) base_addr + unit_cnt - 1;
    pfifo->buf_size    = unit_cnt;
    pfifo->free        = unit_cnt;
    pfifo->used        = 0;
    pfifo->read_index  = 0;
    pfifo->write_index = 0;
    
}



/******************************************************************************************
//
//! \brief  Put an element into FIFO
//!
//! \param  [in]  pfifo is the pointer of valid FIFO.
//! \param  [in]  element is the data element you want to put
//!
//! \retval 0 if operate successfully, otherwise return -1.
//
*******************************************************************************************/
int32_t fifo::put(uint8_t element)
{
  //! Check input parameters.
  ASSERT(NULL != pfifo);

  if(0 >= pfifo->free)
  {
    //! Error, FIFO is full!
    return -1;
  }
  
  MUTEX_WAIT();
  pfifo->start_addr[pfifo->write_index++] = element;
  pfifo->write_index %= pfifo->buf_size;
  pfifo->free--;
  pfifo->used++;
  MUTEX_RELEASE();
  
  return 0;
}

/******************************************************************************************
//
//! \brief  Put some elements into FIFO(in single mode).
//!
//! \param  [in]  pfifo is the pointer of valid FIFO.
//! \param  [in]  element is the data element you want to put
//! \param  [in]  the number of elements
//! \retval 0 if operate successfully, otherwise return -1.
//
******************************************************************************************/
int32_t fifo::puts(uint8_t *psource, uint32_t number)
{
  int puts_num = 0;
  
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  
  if(psource == NULL)
      return -1;
  
  MUTEX_WAIT();
  for(uint32_t i = 0; (i < number) && (pfifo->free > 0); i++)
  {
    pfifo->start_addr[pfifo->write_index++] = psource[i];
    pfifo->write_index %= pfifo->buf_size;
    pfifo->free--;
    pfifo->used++;
    puts_num++;
  }
  MUTEX_RELEASE();
  return puts_num;
}

/******************************************************************************************
//
//! \brief  Get an element from FIFO(in single mode).
//!
//! \param  [in]  pfifo is the pointer of valid FIFO.
//!
//! \retval the data element of FIFO.
//
******************************************************************************************/
uint8_t fifo::get(void)
{
  uint8_t   retval = 0;
  
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  
  MUTEX_WAIT();
  retval = pfifo->start_addr[pfifo->read_index++];
  pfifo->read_index %= pfifo->buf_size;
  pfifo->free++;
  pfifo->used--;
  MUTEX_RELEASE();

  return retval;
}

uint16_t fifo::gets(uint8_t* source, uint32_t len)
{
  uint8_t   retval = 0;
  
  //! Check input parameters.
  ASSERT(NULL != pfifo);
  
  MUTEX_WAIT();
  for (int i = 0; (i < len) && (pfifo->used > 0); i++)
  {
    source[i] = pfifo->start_addr[pfifo->read_index++];
    pfifo->read_index %= pfifo->buf_size;
    pfifo->free++;
    pfifo->used--;
    retval++;
  }
  MUTEX_RELEASE();

  return retval;
}

/******************************************************************************************
//
//! \brief  Pre-Read an element from FIFO(in single mode).
//!
//! \param  [in]  pfifo is the pointer of valid FIFO.
//! \param  [in]  offset is the offset from current pointer.
//!
//! \retval the data element of FIFO.
//
******************************************************************************************/
uint8_t fifo::pre_read(uint8_t offset)
{
  uint32_t index;

  //! Check input parameters.
  ASSERT(NULL != pfifo);

  if(offset > pfifo->used)
  {        
    return 0x00;
  }
  else
  {
    index = ((pfifo->read_index + offset) % pfifo->buf_size);
    // Move Read Pointer to right position   
    return pfifo->start_addr[index];
  }
}


/******************************************************************************************
//!
//! \retval - None-zero(true) if empty.
//!         - Zero(false) if not empty.
//
******************************************************************************************/
uint8_t fifo::is_empty(void)
{
  //! Check input parameter.
  ASSERT(NULL != pfifo);

  return (0 == pfifo->used);
}

/*****************************************************************************************
//!
//! \retval - None-zero(true) if full.
//!         - Zero(false) if not full.
//
*****************************************************************************************/
uint8_t fifo::is_full(void)
{
  //! Check input parameter.
  ASSERT(NULL != pfifo);

  return (0 == pfifo->free);
}

/******************************************************************************************
//!
//! \retval The number of elements in FIFO.
//
******************************************************************************************/
uint32_t fifo::used_count(void)
{
  //! Check input parameter.
  ASSERT(NULL != pfifo);

  return (pfifo->used);
}

/******************************************************************************************
//!
//! \retval The number of elements in FIFO.
//
******************************************************************************************/
uint32_t fifo::free_count(void)
{
  //! Check input parameter.
  ASSERT(NULL != pfifo);

  return (pfifo->free);
}


/******************************************************************************************
//
//! \brief  Flush the content of FIFO.
//!
//! \param  [in] pfifo is the pointer of valid FIFO.
//!
//! \retval 0 if success, -1 if failure.
//
******************************************************************************************/
uint8_t fifo::flush(void)
{
  //! Check input parameters.
  ASSERT(NULL != pfifo);

  //! Initialize FIFO Control Block.
  MUTEX_WAIT();
  pfifo->free        = pfifo->buf_size;
  pfifo->used        = 0;
  pfifo->read_index  = 0;
  pfifo->write_index = 0;
  MUTEX_RELEASE();

  return 0;
}

