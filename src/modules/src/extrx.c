/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2013 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * extrx.c - Module to handle external receiver inputs
 */

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32fxxx.h"
#include "config.h"
#include "system.h"
#include "nvicconf.h"
#include "commander.h"
#include "uart1.h"
#include "uart3.h"
#include "cppm.h"

#define DEBUG_MODULE  "EXTRX"
#include "debug.h"
#include "log.h"

// #define ENABLE_CPPM
#define ENABLE_SBUS
#define SBUS_Endbyte	0x00
#define SBUS_Startbyte	0x0F
#define ENABLE_EXTRX_LOG

#define SBUS_Baudrate 100000

#define EXTRX_NR_CHANNELS  8

#define EXTRX_CH_TRUST     2
#define EXTRX_CH_ROLL      0
#define EXTRX_CH_PITCH     1
#define EXTRX_CH_YAW       3

#define EXTRX_SIGN_ROLL    (-1)
#define EXTRX_SIGN_PITCH   (-1)
#define EXTRX_SIGN_YAW     (-1)

#define EXTRX_SCALE_ROLL   (40.0f)
#define EXTRX_SCALE_PITCH  (40.0f)
#define EXTRX_SCALE_YAW    (400.0f)

static struct CommanderCrtpValues commanderPacket;
static uint16_t ch[EXTRX_NR_CHANNELS];
static uint8_t	SBUS_Byte[24];		// 25 Datenbytes
static uint8_t	SBUS_Byteindex;
static uint8_t	SBUS_lost_Frames;
static uint16_t	SBUS_Channel[16];

extern  xQueueHandle uart3queue;  // Introduce queueHandler from uart3.c

struct {
	unsigned Endbyte_received:1;
	unsigned SBUS_synced:1;
	unsigned SBUS_Channel_17:1;
	unsigned SBUS_Channel_16:1;
	unsigned Frame_valid:1;
	unsigned Failsave_activated:1;
	unsigned New_Frame_received:1;
} SBUS_Flags;

static void extRxTask(void *param);
static void extRxDecodeCppm(void);
static void extRxDecodeChannels(void);
static void extRxReceiveSBusFrame(void);
static void extRxDecodeSBusChannels(void);

void extRxInit(void)
{
#ifdef ENABLE_CPPM
  cppmInit();
#endif

#ifdef ENABLE_SPEKTRUM
  uart1Init();
#endif

#ifdef ENABLE_SBUS
  uart3Init(SBUS_Baudrate);
#endif


  xTaskCreate(extRxTask, EXTRX_TASK_NAME, EXTRX_TASK_STACKSIZE, NULL, EXTRX_TASK_PRI, NULL);
}

static void extRxTask(void *param)
{

  //Wait for the system to be fully started
  //systemWaitStart();

  while (true)
  {
		#ifdef ENABLE_SBUS
		extRxReceiveSBusFrame();
		if(SBUS_Flags.New_Frame_received==1){
				extRxDecodeSBusChannels();
		}
		#endif
		// extRxDecodeCppm();
  }
}

static void extRxReceiveSBusFrame(void){
	char byte;
	bool test=xQueueReceive(uart3queue, &byte, portMAX_DELAY);
	while(xQueueReceive(uart3queue, &byte, portMAX_DELAY)){ // read until queue empty, terminate immediately afterwards (no timeout)
		// Prüfe Synchronisierung
		if(SBUS_Flags.SBUS_synced){
			//Lese Frame
			if (SBUS_Byteindex<24){
				SBUS_Byte[SBUS_Byteindex++]=byte;
			}
			else if (byte == SBUS_Endbyte || SBUS_Byteindex==24)   {
				SBUS_Flags.New_Frame_received=1;
				SBUS_Byteindex=0;
				return;
			}
			else {
				SBUS_Flags.SBUS_synced=0;// Synchronisierung verloren
				SBUS_lost_Frames++;
			}
		}
		else {
			// Synchronisiere
			if(byte==SBUS_Endbyte){
				SBUS_Flags.Endbyte_received=1;
			}
			else if (byte==SBUS_Startbyte && SBUS_Flags.Endbyte_received){
				SBUS_Flags.SBUS_synced=1;
				SBUS_Flags.Endbyte_received=0;
				SBUS_Byteindex=1;
				SBUS_Byte[0]=SBUS_Startbyte;
			}
		}
	}

}

static void extRxDecodeSBusChannels(void){
	if (SBUS_Byte[0]!=SBUS_Startbyte ){
		SBUS_Flags.Frame_valid=0;
		SBUS_Flags.SBUS_synced=0;
		return;
	}
	else{
		SBUS_Channel[0]	 = ((SBUS_Byte[1]	 | (SBUS_Byte[2]<<8)) & 0x7FF ) ;
		commanderPacket.thrust = SBUS_Channel[0]* (0xFFFF/2047);
		SBUS_Channel[1]  = ((SBUS_Byte[2]>>3 |SBUS_Byte[3]<<5)                 & 0x07FF);
		commanderPacket.roll = SBUS_Channel[1]* (0xFFFF/2047);
		SBUS_Channel[2]  = ((SBUS_Byte[3]>>6 |SBUS_Byte[4]<<2 |SBUS_Byte[5]<<10)  & 0x07FF)	;
		commanderPacket.pitch = SBUS_Channel[2]* (0xFFFF/2047);
		SBUS_Channel[3]  = ((SBUS_Byte[5]>>1 |SBUS_Byte[6]<<7)                 & 0x07FF);
		commanderPacket.yaw = SBUS_Channel[3]* (0xFFFF/2047);
		/* SBUS_Channel[4]  = ((SBUS_Byte[6]>>4 |SBUS_Byte[7]<<4)                 & 0x07FF);
		SBUS_Channel[5]  = ((SBUS_Byte[7]>>7 |SBUS_Byte[8]<<1 |SBUS_Byte[9]<<9)   & 0x07FF);
		SBUS_Channel[6]  = ((SBUS_Byte[9]>>2 |SBUS_Byte[10]<<6)                & 0x07FF);
		SBUS_Channel[7]  = ((SBUS_Byte[10]>>5|SBUS_Byte[11]<<3)                & 0x07FF);
		SBUS_Channel[8]  = ((SBUS_Byte[12]   |SBUS_Byte[13]<<8)                & 0x07FF);
		SBUS_Channel[9]  = ((SBUS_Byte[13]>>3|SBUS_Byte[14]<<5)                & 0x07FF);
		SBUS_Channel[10] = ((SBUS_Byte[14]>>6|SBUS_Byte[15]<<2|SBUS_Byte[16]<<10) & 0x07FF);
		SBUS_Channel[11] = ((SBUS_Byte[16]>>1|SBUS_Byte[17]<<7)                & 0x07FF);
		SBUS_Channel[12] = ((SBUS_Byte[17]>>4|SBUS_Byte[18]<<4)                & 0x07FF);
		SBUS_Channel[13] = ((SBUS_Byte[18]>>7|SBUS_Byte[19]<<1|SBUS_Byte[20]<<9)  & 0x07FF);
		SBUS_Channel[14] = ((SBUS_Byte[20]>>2|SBUS_Byte[21]<<6)                & 0x07FF);
		SBUS_Channel[15] = ((SBUS_Byte[21]>>5|SBUS_Byte[22]<<3)                & 0x07FF);
		SBUS_Flags.SBUS_Channel_16 = (SBUS_Byte[23] & 0b1000000 );		// Bit 7
		SBUS_Flags.SBUS_Channel_17 = (SBUS_Byte[23] & 0b01000000 );		// Bit 6	*/
		SBUS_lost_Frames = SBUS_lost_Frames + (SBUS_Byte[23] & 0b00100000 );		// Funktioniert so nicht!!! oder vll doch?
		SBUS_Flags.Failsave_activated = (SBUS_Byte[23] & 0b00010000 );
		SBUS_Flags.Frame_valid = 1;
		commanderExtrxSet(&commanderPacket);
	}
}


static void extRxDecodeChannels(void)
{
  commanderPacket.thrust = cppmConvert2uint16(ch[EXTRX_CH_TRUST]);
  commanderPacket.roll = EXTRX_SIGN_ROLL * cppmConvert2Float(ch[EXTRX_CH_ROLL], -EXTRX_SCALE_ROLL, EXTRX_SCALE_ROLL);
  commanderPacket.pitch = EXTRX_SIGN_PITCH * cppmConvert2Float(ch[EXTRX_CH_PITCH], -EXTRX_SCALE_PITCH, EXTRX_SCALE_PITCH);
  commanderPacket.yaw = EXTRX_SIGN_YAW * cppmConvert2Float(ch[EXTRX_CH_YAW], -EXTRX_SCALE_YAW, EXTRX_SCALE_YAW);
  commanderExtrxSet(&commanderPacket);
}

static void extRxDecodeCppm(void)
{
  uint16_t ppm;
  static uint8_t currChannel = 0;

  if (cppmGetTimestamp(&ppm) == pdTRUE)
  {
    if (cppmIsAvailible() && ppm < 2100)
    {
      if (currChannel < EXTRX_NR_CHANNELS)
      {
        ch[currChannel] = ppm;
      }
      currChannel++;
    }
    else
    {
      extRxDecodeChannels();
      currChannel = 0;
    }
  }
}

#if 0
static void extRxDecodeSpektrum(void)
{
  while (SerialAvailable(SPEK_SERIAL_PORT) > SPEK_FRAME_SIZE)
  { // More than a frame?  More bytes implies we weren't called for multiple frame times.  We do not want to process 'old' frames in the buffer.
    for (uint8_t i = 0; i < SPEK_FRAME_SIZE; i++)
    {
      SerialRead(SPEK_SERIAL_PORT);
    }  //Toss one full frame of bytes.
  }
  if (spekFrameFlags == 0x01)
  { //The interrupt handler saw at least one valid frame start since we were last here.
    if (SerialAvailable(SPEK_SERIAL_PORT) == SPEK_FRAME_SIZE)
    {  //A complete frame? If not, we'll catch it next time we are called.
      SerialRead(SPEK_SERIAL_PORT);
      SerialRead(SPEK_SERIAL_PORT);        //Eat the header bytes
      for (uint8_t b = 2; b < SPEK_FRAME_SIZE; b += 2)
      {
        uint8_t bh = SerialRead(SPEK_SERIAL_PORT);
        uint8_t bl = SerialRead(SPEK_SERIAL_PORT);
        uint8_t spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);
      if (spekChannel < RC_CHANS) rcValue[spekChannel] = 988 + ((((uint16_t)(bh & SPEK_CHAN_MASK) << 8) + bl) SPEK_DATA_SHIFT);
    }
    spekFrameFlags = 0x00;
    spekFrameData = 0x01;
#if defined(FAILSAFE)
    if(failsafeCnt > 20) failsafeCnt -= 20;
    else failsafeCnt = 0;   // Valid frame, clear FailSafe counter
#endif
  }
  else
  { //Start flag is on, but not enough bytes means there is an incomplete frame in buffer.  This could be OK, if we happened to be called in the middle of a frame.  Or not, if it has been a while since the start flag was set.
    uint32_t spekInterval = (timer0_overflow_count << 8)
        * (64 / clockCyclesPerMicrosecond()) - spekTimeLast;
    if (spekInterval > 2500)
    {
      spekFrameFlags = 0;
    }  //If it has been a while, make the interrupt handler start over.
  }
}
#endif

/* Loggable variables */
#ifdef ENABLE_EXTRX_LOG
LOG_GROUP_START(extrx)
/*
LOG_ADD(LOG_UINT16, ch0, &ch[0])
LOG_ADD(LOG_UINT16, ch1, &ch[1])
LOG_ADD(LOG_UINT16, ch2, &ch[2])
LOG_ADD(LOG_UINT16, ch3, &ch[3])
*/
LOG_ADD(LOG_UINT16, ch0, &SBUS_Channel[0])
LOG_ADD(LOG_UINT16, ch1, &SBUS_Channel[1])
LOG_ADD(LOG_UINT16, ch3, &SBUS_Channel[2])
LOG_ADD(LOG_UINT16, ch4, &SBUS_Channel[3])
LOG_ADD(LOG_UINT16, thrust, &commanderPacket.thrust)
LOG_ADD(LOG_FLOAT, roll, &commanderPacket.roll)
LOG_ADD(LOG_FLOAT, pitch, &commanderPacket.pitch)
LOG_ADD(LOG_FLOAT, yaw, &commanderPacket.yaw)
LOG_GROUP_STOP(extrx)
#endif


