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

#include "replacements.h"

#include "jtag.h"

#include <usb.h>
#include <string.h>
#include <sys/timeb.h>
#include <time.h>

#include "log.h"

/* enable this to debug communication
 */
#if 1
#define _DEBUG_USB_COMMS_
#endif

#ifdef _DEBUG_JTAG_IO_
#define DEBUG_JTAG_IO(expr ...)	LOG_DEBUG(expr)
#else
#define DEBUG_JTAG_IO(expr ...)
#endif

#define VID 0x16C0
#define PID 0x05DC

#define USBAVRLAB_WRITE_ENDPOINT	  0x02
#define USBAVRLAB_READ_ENDPOINT    0x81

#define USBAVRLAB_USB_TIMEOUT	   	1000

#define USBAVRLAB_USB_BUFFER_SIZE 360
#define USBAVRLAB_IN_BUFFER_SIZE	(USBAVRLAB_USB_BUFFER_SIZE)
#define USBAVRLAB_OUT_BUFFER_SIZE  (USBAVRLAB_USB_BUFFER_SIZE)

/* Global USB buffers */
static uint8_t usb_in_buffer[USBAVRLAB_IN_BUFFER_SIZE];
static uint8_t usb_out_buffer[USBAVRLAB_OUT_BUFFER_SIZE];

/* Constants for USBVLAB command */

#define USBAVRLAB_MAX_SPEED 				66
#define USBAVRLAB_MAX_TAP_TRANSMIT 350  //even number is easier to handle
#define USBAVRLAB_MAX_INPUT_DATA   (USBAVRLAB_MAX_TAP_TRANSMIT*4)

#define USBAVRLAB_TAP_BUFFER_SIZE 65536

#define MAX_PENDING_SCAN_RESULTS (USBAVRLAB_MAX_INPUT_DATA)

#define FUNC_START_BOOTLOADER 30
#define FUNC_WRITE_DATA       0x50
#define FUNC_READ_DATA        0x51

//JTAG usb commans
#define JTAG_CMD_TAP_OUTPUT     0x0
#define JTAG_CMD_SET_TRST       0x1
#define JTAG_CMD_SET_SRST       0x2
#define JTAG_CMD_READ_INPUT     0x3
#define JTAG_CMD_TAP_OUTPUT_EMU 0x4
#define JTAG_CMD_SET_DELAY      0x5
#define JTAG_CMD_SET_SRST_TRST  0x6


/* External interface functions */
int usbavrlab_execute_queue(void);
int usbavrlab_speed(int speed);
int usbavrlab_khz(int khz, int *jtag_speed);
int usbavrlab_register_commands(struct command_context_s *cmd_ctx);
int usbavrlab_init(void);
int usbavrlab_quit(void);

/* CLI command handler functions */
int usbavrlab_handle_usbavrlab_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* Queue command functions */
void usbavrlab_end_state(tap_state_t state);
void usbavrlab_state_move(void);
void usbavrlab_path_move(int num_states, tap_state_t *path);
void usbavrlab_runtest(int num_cycles);
void usbavrlab_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size, scan_command_t *command);
void usbavrlab_reset(int trst, int srst);
void usbavrlab_simple_command(u8 command,u8 _data);
int usbavrlab_get_status(void);

/* usbvlab tap buffer functions */
void usbavrlab_tap_init(void);
int  usbavrlab_tap_execute(void);
void usbavrlab_tap_ensure_space(int scans, int bits);
void usbavrlab_tap_append_step(int tms, int tdi);
void usbavrlab_tap_append_scan(int length, u8 *buffer, scan_command_t *command);

/* usbvlab lowlevel functions */
typedef struct usbavrlab_jtag
{
	struct usb_dev_handle* usb_handle;
} usbavrlab_jtag_t;

usbavrlab_jtag_t *usbavrlab_usb_open(void);
void usbavrlab_usb_close(usbavrlab_jtag_t *usbavrlab_jtag);
int usbavrlab_usb_message(usbavrlab_jtag_t *usbavrlab_jtag, int out_length, int in_length);
int usbavrlab_usb_write(usbavrlab_jtag_t *usbavrlab_jtag, int out_length);
int usbavrlab_usb_read(usbavrlab_jtag_t *usbavrlab_jtag);
int usbavrlab_usb_read_emu_result(usbavrlab_jtag_t *usbavrlab_jtag);

/* helper functions */
int usbavrlab_get_version_info(void);

#ifdef _DEBUG_USB_COMMS_
char time_str[50];
void usbavrlab_debug_buffer(u8 *buffer, int length);
char *usbavrlab_get_time(char *);
#endif

usbavrlab_jtag_t* usbavrlab_jtag_handle;

/***************************************************************************/
/* External interface implementation */

jtag_interface_t usbavrlab_interface =
{
	.name = "usbvlab",
	.execute_queue = usbavrlab_execute_queue,
	.speed = usbavrlab_speed,
	.khz = usbavrlab_khz,
	.register_commands = usbavrlab_register_commands,
	.init = usbavrlab_init,
	.quit = usbavrlab_quit
};

int usbavrlab_execute_queue(void)
{
	jtag_command_t *cmd = jtag_command_queue;
	int scan_size;
	enum scan_type type;
	u8 *buffer;

	while (cmd != NULL)
	{
		switch (cmd->type)
		{
			case JTAG_END_STATE:
				DEBUG_JTAG_IO("end_state: %i", cmd->cmd.end_state->end_state);

				if (cmd->cmd.end_state->end_state != -1)
				{
					usbavrlab_end_state(cmd->cmd.end_state->end_state);
				}
				break;

			case JTAG_RUNTEST:
				DEBUG_JTAG_IO( "runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles, \
					cmd->cmd.runtest->end_state);

				if (cmd->cmd.runtest->end_state != -1)
				{
					usbavrlab_end_state(cmd->cmd.runtest->end_state);
				}
				usbavrlab_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_STATEMOVE:
				DEBUG_JTAG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

				if (cmd->cmd.statemove->end_state != -1)
				{
					usbavrlab_end_state(cmd->cmd.statemove->end_state);
				}
				usbavrlab_state_move();
				break;

			case JTAG_PATHMOVE:
				DEBUG_JTAG_IO("pathmove: %i states, end in %i", \
					cmd->cmd.pathmove->num_states, \
					cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

				usbavrlab_path_move(cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path);
				break;

			case JTAG_SCAN:
				DEBUG_JTAG_IO("scan end in %i", cmd->cmd.scan->end_state);

				if (cmd->cmd.scan->end_state != -1)
				{
					usbavrlab_end_state(cmd->cmd.scan->end_state);
				}

				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				DEBUG_JTAG_IO("scan input, length = %d", scan_size);

#ifdef _DEBUG_USB_COMMS_
				usbavrlab_debug_buffer(buffer, (scan_size + 7) / 8);
#endif
				type = jtag_scan_type(cmd->cmd.scan);
				usbavrlab_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size, cmd->cmd.scan);
				break;

			case JTAG_RESET:
				DEBUG_JTAG_IO("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);

				usbavrlab_tap_execute();

				if (cmd->cmd.reset->trst == 1)
				{
					tap_set_state(TAP_RESET);
				}
				usbavrlab_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;

			case JTAG_SLEEP:
				DEBUG_JTAG_IO("sleep %i", cmd->cmd.sleep->us);
				usbavrlab_tap_execute();
				jtag_sleep(cmd->cmd.sleep->us);
				break;

			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}
	return usbavrlab_tap_execute();
}

/* Sets speed in kHz. */
int usbavrlab_speed(int speed)
{
  
	if (speed <= USBAVRLAB_MAX_SPEED)
	{
    //one day...
    return ERROR_OK;
	}
	else
	{
		LOG_INFO("Requested speed %dkHz exceeds maximum of %dkHz, ignored", speed, USBAVRLAB_MAX_SPEED);
	}

	return ERROR_OK;
}

int usbavrlab_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
  //TODO: convert this into delay value for usbvlab

	return ERROR_OK;
}

int usbavrlab_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "usbavrlab_info", usbavrlab_handle_usbavrlab_info_command, COMMAND_EXEC,
		"query jlink info");
	return ERROR_OK;
}

int usbavrlab_init(void)
{
	int check_cnt;
  unsigned char dummy=0;

	usbavrlab_jtag_handle = usbavrlab_usb_open();

	if (usbavrlab_jtag_handle == 0)
	{
		LOG_ERROR("Cannot find USBVlab Interface! Please check connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}
  
  //reset the board
  
  //usb_control_msg(usbavrlab_jtag_handle, (REQDIR_DEVICETOHOST | REQTYPE_VENDOR | REQTYPE_STANDARD), 1, 0, 0, &dummy, 1, USBAVRLAB_USB_TIMEOUT);
	
	check_cnt = 0;
	while (check_cnt < 3)
	{
		if (usbavrlab_get_version_info() == ERROR_OK)
		{
			/* attempt to get status */
			usbavrlab_get_status();
			break;
		}

		check_cnt++;
	}

	LOG_INFO("USBVlab JTAG Interface ready");

	usbavrlab_reset(0, 0);
	usbavrlab_tap_init();
	//usbavrlab_simple_command ( JTAG_CMD_SET_DELAY, 255);


	return ERROR_OK;
}

int usbavrlab_quit(void)
{
	usbavrlab_usb_close(usbavrlab_jtag_handle);
	return ERROR_OK;
}

/***************************************************************************/
/* Queue command implementations */

void usbavrlab_end_state(tap_state_t state)
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
void usbavrlab_state_move(void)
{
	int i;
	int tms = 0;
	u8 tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());

	for (i = 0; i < 7; i++)
	{
		tms = (tms_scan >> i) & 1;
		usbavrlab_tap_append_step(tms, 0);
	}

	tap_set_state(tap_get_end_state());
}

void usbavrlab_path_move(int num_states, tap_state_t *path)
{
	int i;

	for (i = 0; i < num_states; i++)
	{
		if (path[i] == tap_state_transition(tap_get_state(), false))
		{
			usbavrlab_tap_append_step(0, 0);
		}
		else if (path[i] == tap_state_transition(tap_get_state(), true))
		{
			usbavrlab_tap_append_step(1, 0);
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

void usbavrlab_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE)
	{
		usbavrlab_end_state(TAP_IDLE);
		usbavrlab_state_move();
	}

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++)
	{
		usbavrlab_tap_append_step(0, 0);
	}

	/* finish in end_state */
	usbavrlab_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
	{
		usbavrlab_state_move();
	}
}

void usbavrlab_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size, scan_command_t *command)
{
	tap_state_t saved_end_state;

	usbavrlab_tap_ensure_space(1, scan_size + 8);

	saved_end_state = tap_get_end_state();

	/* Move to appropriate scan state */
	usbavrlab_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	usbavrlab_state_move();
	usbavrlab_end_state(saved_end_state);

	/* Scan */
	usbavrlab_tap_append_scan(scan_size, buffer, command);

	/* We are in Exit1, go to Pause */
	usbavrlab_tap_append_step(0, 0);

	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
	{
		usbavrlab_state_move();
	}
}

void usbavrlab_reset(int trst, int srst)
{
    LOG_DEBUG("trst: %i, srst: %i", trst, srst);
  
    /* Signals are active low */
/*
    if (srst == 0)
    {
        usbavrlab_simple_command ( JTAG_CMD_SET_SRST,1);
    }
    else if (srst == 1)
    {
        usbavrlab_simple_command ( JTAG_CMD_SET_SRST,0);
    }

    if (trst == 0)
    {
        usbavrlab_simple_command ( JTAG_CMD_SET_TRST,1);
    }
    else if (trst == 1)
    {
        usbavrlab_simple_command ( JTAG_CMD_SET_TRST,0);
    }*/

		srst=srst?0:1;
		trst=trst?0:2;
		usbavrlab_simple_command ( JTAG_CMD_SET_SRST_TRST,srst|trst );
}

void usbavrlab_simple_command(u8 command,uint8_t _data)
{
	int result;

	DEBUG_JTAG_IO("0x%02x 0x%02x", command,_data);

	usb_out_buffer[0] = (uint16_t) 2;
	usb_out_buffer[2] = command;
	usb_out_buffer[3] = _data;

	result = usbavrlab_usb_message(usbavrlab_jtag_handle, 4, 1);
	if (result != 1)
	{
		LOG_ERROR("USBVlab command 0x%02x failed (%d)", command, result);
	}
}

int usbavrlab_get_status(void)
{
	int result;
/*
	usbavrlab_simple_command(EMU_CMD_GET_STATE);
	usbavrlab_simple_command(EMU_CMD_GET_STATE);
	result = usbavrlab_usb_read(usbavrlab_jtag_handle);

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

int usbavrlab_get_version_info(void)
{
  return ERROR_OK;
}

int usbavrlab_handle_usbavrlab_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (usbavrlab_get_version_info() == ERROR_OK)
	{
		/* attempt to get status */
		usbavrlab_get_status();
	}

	return ERROR_OK;
}

/***************************************************************************/
/* USBVlab tap functions */


static int tap_length;
static u8 tms_buffer[USBAVRLAB_TAP_BUFFER_SIZE];
static u8 tdo_buffer[USBAVRLAB_TAP_BUFFER_SIZE];

typedef struct
{
	int first;	/* First bit position in tdo_buffer to read */
	int length; /* Number of bits to read */
	scan_command_t *command; /* Corresponding scan command */
	u8 *buffer;
} pending_scan_result_t;


static int pending_scan_results_length;
static pending_scan_result_t pending_scan_results_buffer[MAX_PENDING_SCAN_RESULTS];

static int last_tms;

void usbavrlab_tap_init(void)
{
	tap_length = 0;
	pending_scan_results_length = 0;
}

void usbavrlab_tap_ensure_space(int scans, int bits)
{
	int available_scans = MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;

	if (scans > available_scans )
	{
		usbavrlab_tap_execute();
	}
}

void usbavrlab_tap_append_step(int tms, int tdi)
{
	last_tms = tms;
	unsigned char _tms=tms?1:0;
	unsigned char _tdi=tdi?1:0;
  
	int index =  tap_length/4;
	int bits  = (tap_length%4)*2;

	if (tap_length < USBAVRLAB_TAP_BUFFER_SIZE)
	{
	    if(!bits)
					tms_buffer[index]=0;

      tms_buffer[index]  |= (_tdi<<bits)|(_tms<<(bits+1)) ;
	    tap_length++;
	}
	else
	{
		LOG_ERROR("usbavrlab_tap_append_step, overflow");
	}
}

void usbavrlab_tap_append_scan(int length, u8 *buffer, scan_command_t *command)
{
  DEBUG_JTAG_IO("append scan, length = %d", length);

	pending_scan_result_t *pending_scan_result = &pending_scan_results_buffer[pending_scan_results_length];
	int i;

	pending_scan_result->first = tap_length;
	pending_scan_result->length = length;
	pending_scan_result->command = command;
	pending_scan_result->buffer = buffer;

	for (i = 0; i < length; i++)
	{
		usbavrlab_tap_append_step((i < length-1 ? 0 : 1), (buffer[i/8] >> (i%8)) & 1);
	}
	pending_scan_results_length++;
}

/* Pad and send a tap sequence to the device, and receive the answer.
 * For the purpose of padding we assume that we are in idle or pause state. */
int usbavrlab_tap_execute(void)
{
	int byte_length,byte_length_out;
	int i,j;
	int result;
	int output_counter;

  
	if (tap_length > 0)
	{

	    //memset(tdo_buffer,0,USBAVRLAB_TAP_BUFFER_SIZE);
	    //LOG_INFO("USBVLAB tap execute %d",tap_length);
	    byte_length =     (tap_length+3)/4;
	    byte_length_out = (tap_length+7)/8;
	    
#ifdef _DEBUG_USB_COMMS_
	    LOG_DEBUG("USBVlab is sending %d bytes", byte_length);
#endif
		
	    output_counter=0;
	    for (j = 0, i = 0; j <  byte_length;)
	    {
				 int recieve;
				 int transmit=byte_length-j;
				 if(transmit>USBAVRLAB_MAX_TAP_TRANSMIT)
				 {
						 transmit=USBAVRLAB_MAX_TAP_TRANSMIT;
						 recieve=(USBAVRLAB_MAX_TAP_TRANSMIT)/2;
						 usb_out_buffer[2]=JTAG_CMD_TAP_OUTPUT;

				 }  else {
						 usb_out_buffer[2]=JTAG_CMD_TAP_OUTPUT | ((tap_length%4)<<4);
						 recieve=(transmit+1)/2;
				 }
				 *(uint16_t *)&usb_out_buffer[0]=transmit+1;
				 memmove(usb_out_buffer+3,tms_buffer+j,transmit);
				 result = usbavrlab_usb_message(usbavrlab_jtag_handle, 3 + transmit, recieve);
				 if(result!=recieve)
				 {
						 LOG_ERROR("usbavrlab_tap_execute, wrong result %d, expected %d", result, recieve);
						 return ERROR_JTAG_QUEUE_FAILED;
				 }
		 
				 memmove(tdo_buffer+i,usb_in_buffer,recieve);
         i+=recieve;
				 j+=transmit;
	    }
    
    result=byte_length_out;
#ifdef _DEBUG_USB_COMMS_
    LOG_DEBUG("USBVlab tap result %d",result);
    usbavrlab_debug_buffer(tdo_buffer,result);
#endif
    //LOG_INFO("USBVlab tap execute %d",tap_length);
    for (i = 0; i < pending_scan_results_length; i++)
    {
      pending_scan_result_t *pending_scan_result = &pending_scan_results_buffer[i];
      u8 *buffer = pending_scan_result->buffer;
      int length = pending_scan_result->length;
      int first = pending_scan_result->first;
      scan_command_t *command = pending_scan_result->command;

      /* Copy to buffer */
      buf_set_buf(tdo_buffer, first, buffer, 0, length);

      DEBUG_JTAG_IO("pending scan result, length = %d", length);

#ifdef _DEBUG_USB_COMMS_
      //usbavrlab_debug_buffer(buffer, byte_length_out);
#endif

      if (jtag_read_buffer(buffer, command) != ERROR_OK)
      {
        usbavrlab_tap_init();
        return ERROR_JTAG_QUEUE_FAILED;
      }

      if (pending_scan_result->buffer != NULL)
      {
        free(pending_scan_result->buffer);
      }
    }

		usbavrlab_tap_init();
	}

	return ERROR_OK;
}

/*****************************************************************************/
/* USBVlab USB low-level functions */

usbavrlab_jtag_t* usbavrlab_usb_open()
{
	struct usb_bus *busses;
	struct usb_bus *bus;
	struct usb_device *dev;

	usbavrlab_jtag_t *result;

	result = (usbavrlab_jtag_t*) malloc(sizeof(usbavrlab_jtag_t));

	usb_init();
	usb_find_busses();
	usb_find_devices();

	busses = usb_get_busses();

	/* find usbavrlab_jtag device in usb bus */

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

void usbavrlab_usb_close(usbavrlab_jtag_t *usbavrlab_jtag)
{
	usb_close(usbavrlab_jtag->usb_handle);
	free(usbavrlab_jtag);
}

/* Send a message and receive the reply. */
int usbavrlab_usb_message(usbavrlab_jtag_t *usbavrlab_jtag, int out_length, int in_length)
{
	int result;
	int result2;

	result = usbavrlab_usb_write(usbavrlab_jtag, out_length);
	if (result == out_length)
	{
		result = usbavrlab_usb_read(usbavrlab_jtag);
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
int usbavrlab_usb_write(usbavrlab_jtag_t *usbavrlab_jtag, int out_length)
{
	int result;

	if (out_length > USBAVRLAB_OUT_BUFFER_SIZE)
	{
		LOG_ERROR("usbavrlab_jtag_write illegal out_length=%d (max=%d)", out_length, USBAVRLAB_OUT_BUFFER_SIZE);
		return -1;
	}
	
#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("%s: USB write begin", usbavrlab_get_time(time_str));
#endif
	/*
	result = usb_bulk_write(usbavrlab_jtag->usb_handle, USBAVRLAB_WRITE_ENDPOINT, \
		usb_out_buffer, out_length, USBAVRLAB_USB_TIMEOUT);
	*/
	result = usb_control_msg(usbavrlab_jtag->usb_handle, 
		USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT,
		FUNC_WRITE_DATA, 0, 0, usb_out_buffer, out_length, USBAVRLAB_USB_TIMEOUT);
#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("%s: USB write end: %d bytes", usbavrlab_get_time(time_str), result);
#endif

	DEBUG_JTAG_IO("usbavrlab_usb_write, out_length = %d, result = %d", out_length, result);

#ifdef _DEBUG_USB_COMMS_
	//usbavrlab_debug_buffer(usb_out_buffer, out_length);
#endif
	return result;
}

/* Read data from USB into in_buffer. */
int usbavrlab_usb_read(usbavrlab_jtag_t *usbavrlab_jtag)
{
	int result;

#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("%s: USB read begin", usbavrlab_get_time(time_str));
#endif
	/*
	int result = usb_bulk_read(usbavrlab_jtag->usb_handle, USBAVRLAB_READ_ENDPOINT,
		usb_in_buffer, USBAVRLAB_IN_BUFFER_SIZE, USBAVRLAB_USB_TIMEOUT);
	*/
	result = usb_control_msg(usbavrlab_jtag->usb_handle, 
		USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN,
		FUNC_READ_DATA, 0, 0, usb_in_buffer, USBAVRLAB_IN_BUFFER_SIZE, USBAVRLAB_USB_TIMEOUT);
#ifdef _DEBUG_USB_COMMS_  
	LOG_DEBUG("%s: USB read end: %d bytes", usbavrlab_get_time(time_str), result);
#endif
	DEBUG_JTAG_IO("usbavrlab_usb_read, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
	//usbavrlab_debug_buffer(usb_in_buffer, result);
#endif
	return result;
}

//#ifdef _DEBUG_USB_COMMS_
#define BYTES_PER_LINE  16

void usbavrlab_debug_buffer(u8 *buffer, int length)
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
		LOG_DEBUG(line);
	}
}

char *usbavrlab_get_time(char *str)
{
	struct timeb timebuffer;
    char *timeline;

	ftime( &timebuffer );
	timeline = ctime( & ( timebuffer.time ) );
	snprintf(str, 49, "%.8s.%hu", &timeline[11], timebuffer.millitm);
	return str;
}
//#endif
