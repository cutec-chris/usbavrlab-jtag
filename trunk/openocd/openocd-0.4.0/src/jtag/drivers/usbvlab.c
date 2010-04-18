/***************************************************************************
 *                                                                         *
 *   Copyright (C) 2009 by Cahya Wirawan <cahya@gmx.at>                    * 
 *   Based on opendous driver by Vladimir Fonov                            *
 *                                                                         *
 *   Copyright (C) 2009 by Vladimir Fonov <vladimir.fonov@gmai.com>        * 
 *   Based on J-link driver by  Juergen Stuber                             *
 *                                                                         *
 *   Copyright (C) 2007 by Juergen Stuber <juergen@jstuber.net>            *
 *   based on Dominic Rath's and Benedikt Sauter's usbprog.c               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/commands.h>
#include "usb_common.h"
#include <string.h>
#include <sys/timeb.h>
#include <time.h>

#define VID 0x16C0
#define PID 0x05DC

#define USBVLAB_WRITE_ENDPOINT   0x02
#define USBVLAB_READ_ENDPOINT    0x81

static unsigned int usbvlab_hw_jtag_version = 1;

#define USBVLAB_USB_TIMEOUT      1000

#define USBVLAB_USB_BUFFER_SIZE  360
#define USBVLAB_IN_BUFFER_SIZE   (USBVLAB_USB_BUFFER_SIZE)
#define USBVLAB_OUT_BUFFER_SIZE  (USBVLAB_USB_BUFFER_SIZE)

/* Global USB buffers */
static uint8_t usb_in_buffer[USBVLAB_IN_BUFFER_SIZE];
static uint8_t usb_out_buffer[USBVLAB_OUT_BUFFER_SIZE];

/* Constants for USBVLAB command */

#define USBVLAB_MAX_SPEED 				66
#define USBVLAB_MAX_TAP_TRANSMIT 350  //even number is easier to handle
#define USBVLAB_MAX_INPUT_DATA   (USBVLAB_MAX_TAP_TRANSMIT*4)

#define USBVLAB_TAP_BUFFER_SIZE 65536

#define FUNC_START_BOOTLOADER 30
#define FUNC_WRITE_DATA       0x50
#define FUNC_READ_DATA        0x51

#define MAX_PENDING_SCAN_RESULTS (USBVLAB_MAX_INPUT_DATA)

//JTAG usb commans
#define JTAG_CMD_TAP_OUTPUT     0x0
#define JTAG_CMD_SET_TRST       0x1
#define JTAG_CMD_SET_SRST       0x2
#define JTAG_CMD_READ_INPUT     0x3
#define JTAG_CMD_TAP_OUTPUT_EMU 0x4
#define JTAG_CMD_SET_DELAY      0x5
#define JTAG_CMD_SET_SRST_TRST  0x6

/* External interface functions */
static int usbvlab_execute_queue(void);
static int usbvlab_speed(int speed);
static int usbvlab_speed_div(int speed, int *khz);
static int usbvlab_khz(int khz, int *jtag_speed);
static int usbvlab_init(void);
static int usbvlab_quit(void);

/* Queue command functions */
static void usbvlab_end_state(tap_state_t state);
static void usbvlab_state_move(void);
static void usbvlab_path_move(int num_states, tap_state_t *path);
static void usbvlab_runtest(int num_cycles);
static void usbvlab_scan(int ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command);
static void usbvlab_reset(int trst, int srst);
static void usbvlab_simple_command(uint8_t command,uint8_t _data);
static int usbvlab_get_status(void);

/* USBVLab tap buffer functions */
static void usbvlab_tap_init(void);
static int  usbvlab_tap_execute(void);
static void usbvlab_tap_ensure_space(int scans, int bits);
static void usbvlab_tap_append_step(int tms, int tdi);
static void usbvlab_tap_append_scan(int length, uint8_t *buffer, struct scan_command *command);

/* usbvlab lowlevel functions */
struct usbvlab_jtag
{
	struct usb_dev_handle* usb_handle;
};

static struct usbvlab_jtag *usbvlab_usb_open(void);
static void usbvlab_usb_close(struct usbvlab_jtag *usbvlab_jtag);
static int usbvlab_usb_message(struct usbvlab_jtag *usbvlab_jtag, int out_length, int in_length);
static int usbvlab_usb_write(struct usbvlab_jtag *usbvlab_jtag, int out_length);
static int usbvlab_usb_read(struct usbvlab_jtag *usbvlab_jtag);
//static int usbvlab_usb_read_emu_result(struct usbvlab_jtag *usbvlab_jtag);

/* helper functions */
int usbvlab_get_version_info(void);

#ifdef _DEBUG_USB_COMMS_
char time_str[50];
static void usbvlab_debug_buffer(uint8_t *buffer, int length);
char *usbvlab_get_time(char *);
#endif

static struct usbvlab_jtag* usbvlab_jtag_handle;

/***************************************************************************/
/* External interface implementation */

COMMAND_HANDLER(usbvlab_handle_usbvlab_info_command)
{
	if (usbvlab_get_version_info() == ERROR_OK)
	{
		/* attempt to get status */
		usbvlab_get_status();
	}

	return ERROR_OK;
}

COMMAND_HANDLER(usbvlab_handle_usbvlab_hw_jtag_command)
{
	switch (CMD_ARGC) {
	case 0:
		command_print(CMD_CTX, "usbvlab hw jtag  %i", usbvlab_hw_jtag_version);
		break;
	case 1: {
		int request_version = atoi(CMD_ARGV[0]);
		switch (request_version) {
		case 2: case 3:
			usbvlab_hw_jtag_version = request_version;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		break;
	}
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

static const struct command_registration usbvlab_command_handlers[] = {
	{
		.name = "usbvlab_info",
		.handler = &usbvlab_handle_usbvlab_info_command,
		.mode = COMMAND_EXEC,
		.help = "show usbvlab info",
	},
	{
		.name = "usbvlab_hw_jtag",
		.handler = &usbvlab_handle_usbvlab_hw_jtag_command,
		.mode = COMMAND_EXEC,
		.help = "access USBVLab HW JTAG command version",
		.usage = "[2|3]",
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface usbvlab_interface = 
{
	.name = "usbvlab",
	.commands = usbvlab_command_handlers,
	.execute_queue = usbvlab_execute_queue,
	.speed = usbvlab_speed,
	.speed_div = usbvlab_speed_div,
	.khz = usbvlab_khz,
	.init = usbvlab_init,
	.quit = usbvlab_quit,
};

static int usbvlab_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;

	while (cmd != NULL)
	{
		switch (cmd->type)
		{
			case JTAG_RUNTEST:
				DEBUG_JTAG_IO( "runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles, \
					cmd->cmd.runtest->end_state);

				if (cmd->cmd.runtest->end_state != -1)
				{
					usbvlab_end_state(cmd->cmd.runtest->end_state);
				}
				usbvlab_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_STATEMOVE:
				DEBUG_JTAG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

				if (cmd->cmd.statemove->end_state != -1)
				{
					usbvlab_end_state(cmd->cmd.statemove->end_state);
				}
				usbvlab_state_move();
				break;

			case JTAG_PATHMOVE:
				DEBUG_JTAG_IO("pathmove: %i states, end in %i", \
					cmd->cmd.pathmove->num_states, \
					cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

				usbvlab_path_move(cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path);
				break;

			case JTAG_SCAN:
				DEBUG_JTAG_IO("scan end in %i", cmd->cmd.scan->end_state);

				if (cmd->cmd.scan->end_state != -1)
				{
					usbvlab_end_state(cmd->cmd.scan->end_state);
				}

				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				DEBUG_JTAG_IO("scan input, length = %d", scan_size);

#ifdef _DEBUG_USB_COMMS_
				usbvlab_debug_buffer(buffer, (scan_size + 7) / 8);
#endif
				type = jtag_scan_type(cmd->cmd.scan);
				usbvlab_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size, cmd->cmd.scan);
				break;

			case JTAG_RESET:
				DEBUG_JTAG_IO("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);

				usbvlab_tap_execute();

				if (cmd->cmd.reset->trst == 1)
				{
					tap_set_state(TAP_RESET);
				}
				usbvlab_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;

			case JTAG_SLEEP:
				DEBUG_JTAG_IO("sleep %i", cmd->cmd.sleep->us);
				usbvlab_tap_execute();
				jtag_sleep(cmd->cmd.sleep->us);
				break;

			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}
	return usbvlab_tap_execute();
}

/* Sets speed in kHz. */
static int usbvlab_speed(int speed)
{
  
	if (speed <= USBVLAB_MAX_SPEED)
	{
    //one day...
    return ERROR_OK;
	}
	else
	{
		LOG_INFO("Requested speed %dkHz exceeds maximum of %dkHz, ignored", speed, USBVLAB_MAX_SPEED);
	}

	return ERROR_OK;
}

static int usbvlab_speed_div(int speed, int* khz)
{
	*khz = speed;

	return ERROR_OK;
}

static int usbvlab_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
  //TODO: convert this into delay value for usbvlab

	return ERROR_OK;
}

static int usbvlab_init(void)
{
	int check_cnt;

	usbvlab_jtag_handle = usbvlab_usb_open();

	if (usbvlab_jtag_handle == 0)
	{
		LOG_ERROR("Cannot find USBVLab Interface! Please check connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}
  
  //reset the board
  
  //usb_control_msg(usbvlab_jtag_handle, (REQDIR_DEVICETOHOST | REQTYPE_VENDOR | REQTYPE_STANDARD), 1, 0, 0, &dummy, 1, USBVLAB_USB_TIMEOUT);
	
	check_cnt = 0;
	while (check_cnt < 3)
	{
		if (usbvlab_get_version_info() == ERROR_OK)
		{
			/* attempt to get status */
			usbvlab_get_status();
			break;
		}

		check_cnt++;
	}

	LOG_INFO("USBVLab JTAG Interface ready");

	usbvlab_reset(0, 0);
	usbvlab_tap_init();
	//usbvlab_simple_command ( JTAG_CMD_SET_DELAY, 255);


	return ERROR_OK;
}

static int usbvlab_quit(void)
{
	usbvlab_usb_close(usbvlab_jtag_handle);
	return ERROR_OK;
}

/***************************************************************************/
/* Queue command implementations */

void usbvlab_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
	{
		tap_set_end_state(state);
	}
	else
	{
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

/* Goes to the end state. */
void usbvlab_state_move(void)
{
	int i;
	int tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	uint8_t tms_scan_bits = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = 0; i < tms_scan_bits; i++)
	{
		tms = (tms_scan >> i) & 1;
		usbvlab_tap_append_step(tms, 0);
	}

	tap_set_state(tap_get_end_state());
}

void usbvlab_path_move(int num_states, tap_state_t *path)
{
	int i;

	for (i = 0; i < num_states; i++)
	{
		if (path[i] == tap_state_transition(tap_get_state(), false))
		{
			usbvlab_tap_append_step(0, 0);
		}
		else if (path[i] == tap_state_transition(tap_get_state(), true))
		{
			usbvlab_tap_append_step(1, 0);
		}
		else
		{
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_name(tap_get_state()), tap_state_name(path[i]));
			exit(-1);
		}

		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());
}

void usbvlab_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE)
	{
		usbvlab_end_state(TAP_IDLE);
		usbvlab_state_move();
	}

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++)
	{
		usbvlab_tap_append_step(0, 0);
	}

	/* finish in end_state */
	usbvlab_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
	{
		usbvlab_state_move();
	}
}

void usbvlab_scan(int ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command)
{
	tap_state_t saved_end_state;

	usbvlab_tap_ensure_space(1, scan_size + 8);

	saved_end_state = tap_get_end_state();

	/* Move to appropriate scan state */
	usbvlab_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	usbvlab_state_move();
	usbvlab_end_state(saved_end_state);

	/* Scan */
	usbvlab_tap_append_scan(scan_size, buffer, command);

	/* We are in Exit1, go to Pause */
	usbvlab_tap_append_step(0, 0);

	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
	{
		usbvlab_state_move();
	}
}

void usbvlab_reset(int trst, int srst)
{
    LOG_DEBUG("trst: %i, srst: %i", trst, srst);
  
    /* Signals are active low */
/*
    if (srst == 0)
    {
        usbvlab_simple_command ( JTAG_CMD_SET_SRST,1);
    }
    else if (srst == 1)
    {
        usbvlab_simple_command ( JTAG_CMD_SET_SRST,0);
    }

    if (trst == 0)
    {
        usbvlab_simple_command ( JTAG_CMD_SET_TRST,1);
    }
    else if (trst == 1)
    {
        usbvlab_simple_command ( JTAG_CMD_SET_TRST,0);
    }*/

		srst=srst?0:1;
		trst=trst?0:2;
		usbvlab_simple_command ( JTAG_CMD_SET_SRST_TRST,srst|trst );
}

void usbvlab_simple_command(uint8_t command,uint8_t _data)
{
	int result;

	DEBUG_JTAG_IO("0x%02x 0x%02x", command,_data);

	usb_out_buffer[0] = (uint16_t) 2;
	usb_out_buffer[2] = command;
	usb_out_buffer[3] = _data;

	result = usbvlab_usb_message(usbvlab_jtag_handle, 4, 1);
	if (result != 1)
	{
		LOG_ERROR("USBVLab command 0x%02x failed (%d)", command, result);
	}
}

int usbvlab_get_status(void)
{
/*
	usbvlab_simple_command(EMU_CMD_GET_STATE);
	usbvlab_simple_command(EMU_CMD_GET_STATE);
	result = usbvlab_usb_read(usbvlab_jtag_handle);

	if (result == 8)
	{
		int vref = usb_in_buffer[0] + (usb_in_buffer[1] << 8);
		LOG_INFO("Vref = %d.%d TCK = %d TDI = %d TDO = %d TMS = %d SRST = %d TRST = %d\n", \
			vref / 1000, vref % 1000, \
			usb_in_buffer[2], usb_in_buffer[3], usb_in_buffer[4], \
			usb_in_buffer[5], usb_in_buffer[6], usb_in_buffer[7]);

		if (vref < 1500)
		{
			LOG_ERROR("Vref too low. Check Target Power\n");
		}
	}
	else
	{
		LOG_ERROR("J-Link command EMU_CMD_GET_STATE failed (%d)\n", result);
	}
*/
	return ERROR_OK;
}

int usbvlab_get_version_info(void)
{
  return ERROR_OK;
}

/***************************************************************************/
/* USBVLab tap functions */


static int tap_length;
static uint8_t tms_buffer[USBVLAB_TAP_BUFFER_SIZE];
static uint8_t tdo_buffer[USBVLAB_TAP_BUFFER_SIZE];

struct pending_scan_result {
	int first;	/* First bit position in tdo_buffer to read */
	int length; /* Number of bits to read */
	struct scan_command *command; /* Corresponding scan command */
	uint8_t *buffer;
};

static int pending_scan_results_length;
static struct pending_scan_result pending_scan_results_buffer[MAX_PENDING_SCAN_RESULTS];

static int last_tms;

void usbvlab_tap_init(void)
{
	tap_length = 0;
	pending_scan_results_length = 0;
}

void usbvlab_tap_ensure_space(int scans, int bits)
{
	int available_scans = MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;

	if (scans > available_scans )
	{
		usbvlab_tap_execute();
	}
}

void usbvlab_tap_append_step(int tms, int tdi)
{
	last_tms = tms;
	unsigned char _tms=tms?1:0;
	unsigned char _tdi=tdi?1:0;
  
	int index =  tap_length/4;
	int bits  = (tap_length%4)*2;

	if (tap_length < USBVLAB_TAP_BUFFER_SIZE)
	{
	    if(!bits)
					tms_buffer[index]=0;

      tms_buffer[index]  |= (_tdi<<bits)|(_tms<<(bits+1)) ;
	    tap_length++;
	}
	else
	{
		LOG_ERROR("usbvlab_tap_append_step, overflow");
	}
}

void usbvlab_tap_append_scan(int length, uint8_t *buffer, struct scan_command *command)
{
  DEBUG_JTAG_IO("append scan, length = %d", length);

	struct pending_scan_result *pending_scan_result = &pending_scan_results_buffer[pending_scan_results_length];
	int i;

	pending_scan_result->first = tap_length;
	pending_scan_result->length = length;
	pending_scan_result->command = command;
	pending_scan_result->buffer = buffer;

	for (i = 0; i < length; i++)
	{
		usbvlab_tap_append_step((i < length-1 ? 0 : 1), (buffer[i/8] >> (i%8)) & 1);
	}
	pending_scan_results_length++;
}

/* Pad and send a tap sequence to the device, and receive the answer.
 * For the purpose of padding we assume that we are in idle or pause state. */
int usbvlab_tap_execute(void)
{
	int byte_length,byte_length_out;
	int i,j;
	int result;
	int output_counter;

  
	if (tap_length > 0)
	{

	    //memset(tdo_buffer,0,USBVLAB_TAP_BUFFER_SIZE);
	    //LOG_INFO("USBVLAB tap execute %d",tap_length);
	    byte_length =     (tap_length+3)/4;
	    byte_length_out = (tap_length+7)/8;
	    
#ifdef _DEBUG_USB_COMMS_
	    LOG_DEBUG("USBVLab is sending %d bytes", byte_length);
#endif
		
	    output_counter=0;
	    for (j = 0, i = 0; j <  byte_length;)
	    {
				 int recieve;
				 int transmit=byte_length-j;
				 if(transmit>USBVLAB_MAX_TAP_TRANSMIT)
				 {
						 transmit=USBVLAB_MAX_TAP_TRANSMIT;
						 recieve=(USBVLAB_MAX_TAP_TRANSMIT)/2;
						 usb_out_buffer[2]=JTAG_CMD_TAP_OUTPUT;

				 }  else {
						 usb_out_buffer[2]=JTAG_CMD_TAP_OUTPUT | ((tap_length%4)<<4);
						 recieve=(transmit+1)/2;
				 }
				 *(uint16_t *)&usb_out_buffer[0]=transmit+1;
				 memmove(usb_out_buffer+3,tms_buffer+j,transmit);
				 result = usbvlab_usb_message(usbvlab_jtag_handle, 3 + transmit, recieve);
				 if(result!=recieve)
				 {
						 LOG_ERROR("usbvlab_tap_execute, wrong result %d, expected %d", result, recieve);
						 return ERROR_JTAG_QUEUE_FAILED;
				 }
		 
				 memmove(tdo_buffer+i,usb_in_buffer,recieve);
         i+=recieve;
				 j+=transmit;
	    }
    
    result=byte_length_out;
#ifdef _DEBUG_USB_COMMS_
    LOG_DEBUG("USBVLab tap result %d",result);
    usbvlab_debug_buffer(tdo_buffer,result);
#endif
    //LOG_INFO("USBVLab tap execute %d",tap_length);
    for (i = 0; i < pending_scan_results_length; i++)
    {
      struct pending_scan_result *pending_scan_result = &pending_scan_results_buffer[i];
      uint8_t *buffer = pending_scan_result->buffer;
      int length = pending_scan_result->length;
      int first = pending_scan_result->first;
      struct scan_command *command = pending_scan_result->command;

      /* Copy to buffer */
      buf_set_buf(tdo_buffer, first, buffer, 0, length);

      DEBUG_JTAG_IO("pending scan result, length = %d", length);

#ifdef _DEBUG_USB_COMMS_
      //usbvlab_debug_buffer(buffer, byte_length_out);
#endif

      if (jtag_read_buffer(buffer, command) != ERROR_OK)
      {
        usbvlab_tap_init();
        return ERROR_JTAG_QUEUE_FAILED;
      }

      if (pending_scan_result->buffer != NULL)
      {
        free(pending_scan_result->buffer);
      }
    }

		usbvlab_tap_init();
	}

	return ERROR_OK;
}

/*****************************************************************************/
/* USBVLab USB low-level functions */

struct usbvlab_jtag* usbvlab_usb_open()
{
	struct usb_bus *busses;
	struct usb_bus *bus;
	struct usb_device *dev;

	struct usbvlab_jtag *result;

	result = (struct usbvlab_jtag*) malloc(sizeof(struct usbvlab_jtag));

	usb_init();
	usb_find_busses();
	usb_find_devices();

	busses = usb_get_busses();

	/* find usbvlab_jtag device in usb bus */

	for (bus = busses; bus; bus = bus->next)
	{
		for (dev = bus->devices; dev; dev = dev->next)
		{
			if ((dev->descriptor.idVendor == VID) && (dev->descriptor.idProduct == PID))
			{
				result->usb_handle = usb_open(dev);

				/* usb_set_configuration required under win32 */
				usb_set_configuration(result->usb_handle, dev->config[0].bConfigurationValue);
				usb_claim_interface(result->usb_handle, 0);

#if 0
				/*
				 * This makes problems under Mac OS X. And is not needed
				 * under Windows. Hopefully this will not break a linux build
				 */
				usb_set_altinterface(result->usb_handle, 0);
#endif
				return result;
			}
		}
	}

	free(result);
	return NULL;
}

void usbvlab_usb_close(struct usbvlab_jtag *usbvlab_jtag)
{
	usb_close(usbvlab_jtag->usb_handle);
	free(usbvlab_jtag);
}

/* Send a message and receive the reply. */
int usbvlab_usb_message(struct usbvlab_jtag *usbvlab_jtag, int out_length, int in_length)
{
	int result;

	result = usbvlab_usb_write(usbvlab_jtag, out_length);
	if (result == out_length)
	{
		result = usbvlab_usb_read(usbvlab_jtag);
    if (result == in_length)
    {
      return result;
    }
		else
		{
			LOG_ERROR("usb_bulk_read failed (requested=%d, result=%d)", in_length, result);
			return -1;
		}
	}
	else
	{
		LOG_ERROR("usb_bulk_write failed (requested=%d, result=%d)", out_length, result);
		return -1;
	}
}

/* Write data from out_buffer to USB. */
int usbvlab_usb_write(struct usbvlab_jtag *usbvlab_jtag, int out_length)
{
	int result;

	if (out_length > USBVLAB_OUT_BUFFER_SIZE)
	{
		LOG_ERROR("usbvlab_jtag_write illegal out_length=%d (max=%d)", out_length, USBVLAB_OUT_BUFFER_SIZE);
		return -1;
	}
	
#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("%s: USB write begin", usbvlab_get_time(time_str));
#endif
	result = usb_control_msg(usbvlab_jtag->usb_handle, 
		USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT,
		FUNC_WRITE_DATA, 0, 0, (char *) usb_out_buffer, out_length, USBVLAB_USB_TIMEOUT);
#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("%s: USB write end: %d bytes", usbvlab_get_time(time_str), result);
#endif

	DEBUG_JTAG_IO("usbvlab_usb_write, out_length = %d, result = %d", out_length, result);

#ifdef _DEBUG_USB_COMMS_
	//usbvlab_debug_buffer(usb_out_buffer, out_length);
#endif
	return result;
}

/* Read data from USB into in_buffer. */
int usbvlab_usb_read(struct usbvlab_jtag *usbvlab_jtag)
{
	int result;

#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("%s: USB read begin", usbvlab_get_time(time_str));
#endif
	result = usb_control_msg(usbvlab_jtag->usb_handle, 
		USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN,
		FUNC_READ_DATA, 0, 0, (char *) usb_in_buffer, USBVLAB_IN_BUFFER_SIZE, USBVLAB_USB_TIMEOUT);
#ifdef _DEBUG_USB_COMMS_  
	LOG_DEBUG("%s: USB read end: %d bytes", usbvlab_get_time(time_str), result);
#endif
	DEBUG_JTAG_IO("usbvlab_usb_read, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
	//usbvlab_debug_buffer(usb_in_buffer, result);
#endif
	return result;
}

//#ifdef _DEBUG_USB_COMMS_
#define BYTES_PER_LINE  16

void usbvlab_debug_buffer(uint8_t *buffer, int length)
{
	char line[81];
	char s[4];
	int i;
	int j;

	for (i = 0; i < length; i += BYTES_PER_LINE)
	{
		snprintf(line, 5, "%04x", i);
		for (j = i; j < i + BYTES_PER_LINE && j < length; j++)
		{
			snprintf(s, 4, " %02x", buffer[j]);
			strcat(line, s);
		}
		LOG_DEBUG("%s", line);
	}
}

char *usbvlab_get_time(char *str)
{
	struct timeb timebuffer;
    char *timeline;

	ftime( &timebuffer );
	timeline = ctime( & ( timebuffer.time ) );
	snprintf(str, 49, "%.8s.%hu", &timeline[11], timebuffer.millitm);
	return str;
}
//#endif
