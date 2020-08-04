
/************************************************************************************************
 * libc/misc/lib_crc16.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
 *
 * The logic in this file was developed by Gary S. Brown:
 *
 *   COPYRIGHT (C) 1986 Gary S. Brown.  You may use this program, or code or tables
 *   extracted from it, as desired without restriction.
 *
 * First, the polynomial itself and its table of feedback terms.  The polynomial is:
 *
 *    X^32+X^26+X^23+X^22+X^16+X^12+X^11+X^10+X^8+X^7+X^5+X^4+X^2+X^1+X^0
 *
 * Note that we take it "backwards" and put the highest-order term in the lowest-order bit.
 * The X^32 term is "implied"; the LSB is the X^31 term, etc.  The X^0 term (usually shown
 * as "+1") results in the MSB being 1
 *
 * Note that the usual hardware shift register implementation, which is what we're using
 * (we're merely optimizing it by doing eight-bit chunks at a time) shifts bits into the
 * lowest-order term.  In our implementation, that means shifting towards the right.  Why
 * do we do it this way?  Because the calculated CRC must be transmitted in order from
 * highest-order term to lowest-order term.  UARTs transmit characters in order from LSB
 * to MSB.  By storing the CRC this way we hand it to the UART in the order low-byte to
 * high-byte; the UART sends each low-bit to hight-bit; and the result is transmission bit
 * by bit from highest- to lowest-order term without requiring any bit shuffling on our
 * part.  Reception works similarly
 *
 * The feedback terms table consists of 256, 32-bit entries.  Notes
 *
 * - The table can be generated at runtime if desired; code to do so is shown later.  It
 *   might not be obvious, but the feedback terms simply represent the results of eight
 *   shift/xor operations for all combinations of data and CRC register values
 *
 * - The values must be right-shifted by eight bits by the updcrc logic; the shift must
 *   be u_(bring in zeroes).  On some hardware you could probably optimize the shift in
 *   assembler by using byte-swap instructions polynomial $edb88320
  ************************************************************************************************/

/************************************************************************************************
 * Included Files
 ************************************************************************************************/


#include "crc16.h"



/************************************************************************************************
 * Private Data
 ************************************************************************************************/

/* crc16_tab calculated by Mark G. Mendel, Network Systems Corporation */

static uint16_t crc16_tab[256] =
{
  0x0000,  0x1021,  0x2042,  0x3063,  0x4084,  0x50a5,  0x60c6,  0x70e7,
  0x8108,  0x9129,  0xa14a,  0xb16b,  0xc18c,  0xd1ad,  0xe1ce,  0xf1ef,
  0x1231,  0x0210,  0x3273,  0x2252,  0x52b5,  0x4294,  0x72f7,  0x62d6,
  0x9339,  0x8318,  0xb37b,  0xa35a,  0xd3bd,  0xc39c,  0xf3ff,  0xe3de,
  0x2462,  0x3443,  0x0420,  0x1401,  0x64e6,  0x74c7,  0x44a4,  0x5485,
  0xa56a,  0xb54b,  0x8528,  0x9509,  0xe5ee,  0xf5cf,  0xc5ac,  0xd58d,
  0x3653,  0x2672,  0x1611,  0x0630,  0x76d7,  0x66f6,  0x5695,  0x46b4,
  0xb75b,  0xa77a,  0x9719,  0x8738,  0xf7df,  0xe7fe,  0xd79d,  0xc7bc,
  0x48c4,  0x58e5,  0x6886,  0x78a7,  0x0840,  0x1861,  0x2802,  0x3823,
  0xc9cc,  0xd9ed,  0xe98e,  0xf9af,  0x8948,  0x9969,  0xa90a,  0xb92b,
  0x5af5,  0x4ad4,  0x7ab7,  0x6a96,  0x1a71,  0x0a50,  0x3a33,  0x2a12,
  0xdbfd,  0xcbdc,  0xfbbf,  0xeb9e,  0x9b79,  0x8b58,  0xbb3b,  0xab1a,
  0x6ca6,  0x7c87,  0x4ce4,  0x5cc5,  0x2c22,  0x3c03,  0x0c60,  0x1c41,
  0xedae,  0xfd8f,  0xcdec,  0xddcd,  0xad2a,  0xbd0b,  0x8d68,  0x9d49,
  0x7e97,  0x6eb6,  0x5ed5,  0x4ef4,  0x3e13,  0x2e32,  0x1e51,  0x0e70,
  0xff9f,  0xefbe,  0xdfdd,  0xcffc,  0xbf1b,  0xaf3a,  0x9f59,  0x8f78,
  0x9188,  0x81a9,  0xb1ca,  0xa1eb,  0xd10c,  0xc12d,  0xf14e,  0xe16f,
  0x1080,  0x00a1,  0x30c2,  0x20e3,  0x5004,  0x4025,  0x7046,  0x6067,
  0x83b9,  0x9398,  0xa3fb,  0xb3da,  0xc33d,  0xd31c,  0xe37f,  0xf35e,
  0x02b1,  0x1290,  0x22f3,  0x32d2,  0x4235,  0x5214,  0x6277,  0x7256,
  0xb5ea,  0xa5cb,  0x95a8,  0x8589,  0xf56e,  0xe54f,  0xd52c,  0xc50d,
  0x34e2,  0x24c3,  0x14a0,  0x0481,  0x7466,  0x6447,  0x5424,  0x4405,
  0xa7db,  0xb7fa,  0x8799,  0x97b8,  0xe75f,  0xf77e,  0xc71d,  0xd73c,
  0x26d3,  0x36f2,  0x0691,  0x16b0,  0x6657,  0x7676,  0x4615,  0x5634,
  0xd94c,  0xc96d,  0xf90e,  0xe92f,  0x99c8,  0x89e9,  0xb98a,  0xa9ab,
  0x5844,  0x4865,  0x7806,  0x6827,  0x18c0,  0x08e1,  0x3882,  0x28a3,
  0xcb7d,  0xdb5c,  0xeb3f,  0xfb1e,  0x8bf9,  0x9bd8,  0xabbb,  0xbb9a,
  0x4a75,  0x5a54,  0x6a37,  0x7a16,  0x0af1,  0x1ad0,  0x2ab3,  0x3a92,
  0xfd2e,  0xed0f,  0xdd6c,  0xcd4d,  0xbdaa,  0xad8b,  0x9de8,  0x8dc9,
  0x7c26,  0x6c07,  0x5c64,  0x4c45,  0x3ca2,  0x2c83,  0x1ce0,  0x0cc1,
  0xef1f,  0xff3e,  0xcf5d,  0xdf7c,  0xaf9b,  0xbfba,  0x8fd9,  0x9ff8,
  0x6e17,  0x7e36,  0x4e55,  0x5e74,  0x2e93,  0x3eb2,  0x0ed1,  0x1ef0
};
/************************************************************************************************
 * Public Functions
 ************************************************************************************************/
/************************************************************************************************
 * Name: crc16part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer.
 *
 ************************************************************************************************/

uint16_t crc16part(const uint8_t *src, size_t len, uint16_t crc16val)
{
  size_t i;

  for (i = 0; i < len; i++)
  {
    crc16val = crc16_tab[((crc16val >> 8) & 0xff) ^ src[i]] ^ (crc16val << 8);
  }

  return crc16val;
}

/************************************************************************************************
 * Name: crc16
 *
 * Description:
 *   Return a 16-bit CRC of the contents of the 'src' buffer, length 'len'
 *
 ************************************************************************************************/

uint16_t crc16(const uint8_t *src, size_t len)
{
  return crc16part(src, len, 0);
}



/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength)
{
  uint16_t wExpected = 0;
  if ((pchMessage == NULL) || (dwLength <= 2))
  {
    return 0;
  }
  wExpected = crc16(pchMessage, dwLength - 2);
  //DBG_OUT("[crc16]crc16 = 0x%x\r\n",wExpected);
  return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength)
{
  uint16_t wCRC = 0;
  if ((pchMessage == NULL) || (dwLength <= 2))
  {
    return;
  }
  wCRC = crc16((uint8_t*)pchMessage, dwLength - 2);
  pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}




