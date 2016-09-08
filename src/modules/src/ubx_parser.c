/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32fxxx.h"
#include "config.h"
#include "system.h"
#include "nvicconf.h"
#include "uart4.h"
#include "log.h"

#include "ubx_parser.h"

#define	UBX_SYNC1_CHAR	0xB5
#define	UBX_SYNC2_CHAR	0x62

#define UBX_Baudrate 9600

// ubx protocol parser state machine
#define	UBXSTATE_IDLE	0
#define	UBXSTATE_SYNC1	1
#define	UBXSTATE_SYNC2	2
#define	UBXSTATE_CLASS	3
#define	UBXSTATE_LEN1	4
#define	UBXSTATE_LEN2	5
#define UBXSTATE_DATA	6
#define	UBXSTATE_CKA	7
#define	UBXSTATE_CKB	8

// ublox protocoll id
#define	UBX_CLASS_NAV	0x01

// ublox message id
#define	UBX_ID_RELPOSNED	0x3C

typedef struct {

	uint8_t			NA;		//
	int8_t			NA2;		//
	int16_t			NA3;		//
	uint32_t		NA4;		//
	int32_t			relPosN;		// North component of relative position vector cm
	int32_t			relPosE;		// East component of relative position vector cm
	int32_t			relPosD;		// Down component of relative position vector cm
	int8_t			NA5;
	int8_t			NA6;
	int8_t			NA7;
	uint8_t			NA8;
	uint32_t		NA9;
	uint32_t		NA10;
	uint32_t		NA11;
	uint32_t		Flags;
	Status_t		Status;
} UBX_RELPOSNED_t;

extern  xQueueHandle uart4queue;
static void ubx_parserTask(void *param);

UBX_RELPOSNED_t		UbxRELPOSNED	  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,INVALID};



void ubxpaserInit(void)
{

  uart4Init(UBX_Baudrate);
  xTaskCreate(ubx_parserTask, UBX_PARSER_TASK_NAME, UBX_PARSER_TASK_STACKSIZE, NULL, UBX_PARSER_TASK_PRI, NULL);
}

static void ubx_parserTask(void *param)
{
	//Wait for the system to be fully started
	systemWaitStart();

	while (1) {
		char c;
		while (xQueueReceive(uart4queue, &c, portMAX_DELAY)) {
			static uint8_t ubxstate = UBXSTATE_IDLE;
			static uint8_t cka, ckb;
			static uint16_t msglen;
			static int8_t *ubxP, *ubxEp, *ubxSp; // pointers to data currently transfered

			switch (ubxstate) {
			case UBXSTATE_IDLE: // check 1st sync byte
				if (c == UBX_SYNC1_CHAR)
					ubxstate = UBXSTATE_SYNC1;
				else
					ubxstate = UBXSTATE_IDLE; // out of synchronization
				break;

			case UBXSTATE_SYNC1: // check 2nd sync byte
				if (c == UBX_SYNC2_CHAR)
					ubxstate = UBXSTATE_SYNC2;
				else
					ubxstate = UBXSTATE_IDLE; // out of synchronization
				break;

			case UBXSTATE_SYNC2: // check msg class to be NAV
				if (c == UBX_CLASS_NAV)
					ubxstate = UBXSTATE_CLASS;
				else
					ubxstate = UBXSTATE_IDLE; // unsupported message class
				break;

			case UBXSTATE_CLASS: // check message identifier to be RELPOSNED
				if (c == UBX_ID_RELPOSNED) {
					ubxP = (int8_t *) &UbxRELPOSNED; // data start pointer
					ubxEp = (int8_t *) (&UbxRELPOSNED + 1); // data end pointer
					ubxSp = (int8_t *) &UbxRELPOSNED.Status; // status pointer
					ubxstate = UBXSTATE_LEN1;
					cka = UBX_CLASS_NAV + c;	// begin calculating checksum
					ckb = UBX_CLASS_NAV + cka;
				} else
					ubxstate = UBXSTATE_IDLE;
				break;

			case UBXSTATE_LEN1: // 1st message length byte
				msglen = c;
				cka += c;
				ckb += cka;
				ubxstate = UBXSTATE_LEN2;
				break;

			case UBXSTATE_LEN2: // 2nd message length byte
				msglen += ((uint16_t) c) << 8;
				cka += c;
				ckb += cka;
				// if the old data are not processed so far then break parsing now
				// to avoid writing new data in ISR during reading by another function
				if (*ubxSp == NEWDATA) {
					//UpdateGPSInfo(); //update GPS info respectively debug
					ubxstate = UBXSTATE_IDLE;
				} else // data invalid or allready processd
				{
					*ubxSp = INVALID;
					ubxstate = UBXSTATE_DATA;
				}
				break;

			case UBXSTATE_DATA:
				if (ubxP < ubxEp)
					*ubxP++ = c; // copy curent data byte if any space is left
				cka += c;
				ckb += cka;
				if (--msglen == 0)
					ubxstate = UBXSTATE_CKA; // switch to next state if all data was read
				break;

			case UBXSTATE_CKA:
				if (c == cka)
					ubxstate = UBXSTATE_CKB;
				else {
					*ubxSp = INVALID;
					ubxstate = UBXSTATE_IDLE;
				}
				break;

			case UBXSTATE_CKB:
				if (c == ckb) {
					*ubxSp = NEWDATA; // new data are valid
					//UpdateGPSInfo(); //update GPS info respectively debug
					//GPSTimeout = 255;	debug
				} else {	// if checksum not fit then set data invalid
					*ubxSp = INVALID;
				}
				ubxstate = UBXSTATE_IDLE; // ready to parse new data
				break;

			default:			// unsupported identifier
				ubxstate = UBXSTATE_IDLE;
				break;
			}
		}
	}
}


LOG_GROUP_START(UBX_GPS)
LOG_ADD(LOG_INT32, RelPosN, &UbxRELPOSNED.relPosN)
LOG_ADD(LOG_INT32, RelPosE, &UbxRELPOSNED.relPosE)
LOG_ADD(LOG_INT32, RelPosD, &UbxRELPOSNED.relPosD)
LOG_ADD(LOG_UINT8, Status_INV_NEW_PAR, &UbxRELPOSNED.Status)
LOG_GROUP_STOP(UBX_GPS)

