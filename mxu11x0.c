/*
 *
 * MOXA UPort 11x0 USB to Serial Hub Driver
 *
 * Copyright (C) 2007 MOXA Technologies Co., Ltd.
 * Copyright (C) 2015 Mathieu Othacehe <m.othacehe@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 *
 * Supports the following Moxa USB to serial converters:
 *  UPort 1110,  1 port RS-232 USB to Serial Hub.
 *  UPort 1130,  1 port RS-422/485 USB to Serial Hub.
 *  UPort 1130I, 1 port RS-422/485 USB to Serial Hub with isolation
 *    protection.
 *  UPort 1150,  1 port RS-232/422/485 USB to Serial Hub.
 *  UPort 1150I, 1 port RS-232/422/485 USB to Serial Hub with isolation
 *  protection.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#include "mxu11x0.h"

struct mxu1_port {
	int mxp_is_open;
	__u8 mxp_msr;
	__u8 mxp_lsr;
	__u8 mxp_shadow_mcr;
	__u8 mxp_uart_mode;     /* 232 or 485 modes */
	__u8 mxp_user_get_uart_mode;
	unsigned int mxp_uart_base_addr;
	int mxp_flags;
	int mxp_msr_wait_flags;
	struct async_icount	mxp_icount;
	wait_queue_head_t mxp_msr_wait;	/* wait for msr change */
	struct mxu1_device *mxp_mxdev;
	struct usb_serial_port *mxp_port;
	spinlock_t mxp_lock;
	int mxp_read_urb_state;
	int mxp_write_urb_in_use;
	int mxp_send_break;
	int mxp_set_B0;
};

struct mxu1_device {
	struct mutex mxd_lock;
	int mxd_open_port_count;
	struct usb_serial	*mxd_serial;
	int mxd_model_name;
};

/* supported setserial flags */
#define MXU1_SET_SERIAL_FLAGS	    (ASYNC_LOW_LATENCY)

#define MXU1_DEFAULT_LOW_LATENCY    1

#define MXU1_TRANSFER_TIMEOUT	    2
#define MXU1_MSR_WAIT_TIMEOUT	    (5 * HZ)

/* Configuration ids */
#define MXU1_BOOT_CONFIG	    1
#define MXU1_ACTIVE_CONFIG	    2

/* read urb states */
#define MXU1_READ_URB_RUNNING	    0
#define MXU1_READ_URB_STOPPING	    1
#define MXU1_READ_URB_STOPPED	    2

#define MXU1_DEFAULT_CLOSING_WAIT   4000		/* in .01 secs */

static const struct usb_device_id mxuport11_idtable[] = {
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1110_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1130_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1150_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1151_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1131_PRODUCT_ID) },
	{ }
};

MODULE_DEVICE_TABLE(usb, mxuport11_idtable);

static int closing_wait = MXU1_DEFAULT_CLOSING_WAIT;

static void mxu1_send(struct mxu1_port *mxport);
static void mxu1_recv(struct mxu1_port *mxport,
		      unsigned char *data, int length);

/* Write the given buffer out to the control pipe.  */
static int mxu1_send_ctrl_data_urb(struct usb_serial *serial,
				   u8 request,
				   u16 value, u16 index,
				   u8 *data, size_t size)
{
	int status;

	status = usb_control_msg(serial->dev,
				 usb_sndctrlpipe(serial->dev, 0),
				 request,
				 (USB_DIR_OUT | USB_TYPE_VENDOR |
				  USB_RECIP_DEVICE), value, index,
				 data, size,
				 1000);
	if (status < 0) {
		dev_err(&serial->interface->dev,
			"%s - usb_control_msg failed (%d)\n",
			__func__, status);
		return status;
	}

	if (status != size) {
		dev_err(&serial->interface->dev,
			"%s - short write (%d / %zd)\n",
			__func__, status, size);
		return -EIO;
	}

	return 0;
}

/* Send a vendor request without any data */
static int mxu1_send_ctrl_urb(struct usb_serial *serial,
			      u8 request, u16 value, u16 index)
{
	return mxu1_send_ctrl_data_urb(serial, request, value, index,
				       NULL, 0);
}

static void mxu1_stop_read(struct mxu1_port *mxport, struct tty_struct *tty)
{
	unsigned long flags;

	spin_lock_irqsave(&mxport->mxp_lock, flags);

	if (mxport->mxp_read_urb_state == MXU1_READ_URB_RUNNING)
		mxport->mxp_read_urb_state = MXU1_READ_URB_STOPPING;

	spin_unlock_irqrestore(&mxport->mxp_lock, flags);
}


static int mxu1_restart_read(struct mxu1_port *mxport, struct tty_struct *tty)
{
	struct urb *urb;
	int status = 0;
	unsigned long flags;

	spin_lock_irqsave(&mxport->mxp_lock, flags);

	if (mxport->mxp_read_urb_state == MXU1_READ_URB_STOPPED) {
		urb = mxport->mxp_port->read_urb;
		status = usb_submit_urb(urb, GFP_KERNEL);
	}
	mxport->mxp_read_urb_state = MXU1_READ_URB_RUNNING;

	spin_unlock_irqrestore(&mxport->mxp_lock, flags);

	return status;
}

static int mxu1_download_firmware(struct usb_serial *serial,
				  const struct firmware *fw_p)
{
	int status = 0;
	int buffer_size;
	int pos;
	int len;
	int done;
	__u8 cs = 0;
	__u8 *buffer;
	struct usb_device *dev = serial->dev;
	struct mxu1_firmware_header *header;
	unsigned int pipe = usb_sndbulkpipe(dev, serial->port[0]->
					    bulk_out_endpointAddress);

	buffer_size = fw_p->size +
		sizeof(struct mxu1_firmware_header);
	buffer = kmalloc(buffer_size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	memcpy(buffer, fw_p->data, fw_p->size);
	memset(buffer + fw_p->size, 0xff, buffer_size - fw_p->size);

	for (pos = sizeof(struct mxu1_firmware_header);
	     pos < buffer_size; pos++)
		cs = (__u8)(cs + buffer[pos]);

	header = (struct mxu1_firmware_header *)buffer;
	header->wLength = cpu_to_le16(
		(__u16)(buffer_size - sizeof(struct mxu1_firmware_header)));
	header->bCheckSum = cs;

	pr_debug("%s - downloading firmware", __func__);

	for (pos = 0; pos < buffer_size; pos += done) {
		len = min(buffer_size - pos, MXU1_DOWNLOAD_MAX_PACKET_SIZE);

		status = usb_bulk_msg(dev, pipe, buffer+pos, len, &done, 1000);

		if (status)
			break;
	}

	kfree(buffer);

	if (status) {
		dev_err(&dev->dev, "%s - error downloading firmware, %d\n",
			__func__, status);
		return status;
	}

	msleep_interruptible(100);

	status = mxu1_send_ctrl_urb(serial, MXU1_RESET_EXT_DEVICE, 0, 0);

	pr_debug("%s - download successful (%d)", __func__, status);

	return 0;
}

static int mxu1_port_probe(struct usb_serial_port *port)
{
	struct mxu1_port *mxport;

	mxport = kzalloc(sizeof(struct mxu1_port), GFP_KERNEL);

	if (!mxport)
		return -ENOMEM;

	spin_lock_init(&mxport->mxp_lock);
	mxport->mxp_port = port;
	mxport->mxp_mxdev = usb_get_serial_data(port->serial);
	mxport->mxp_uart_base_addr = MXU1_UART1_BASE_ADDR;
	mxport->mxp_flags = ASYNC_LOW_LATENCY;

	init_waitqueue_head(&mxport->mxp_msr_wait);

	if (mxport->mxp_mxdev->mxd_model_name != MXU1_MODEL_1130 &&
	    mxport->mxp_mxdev->mxd_model_name != MXU1_MODEL_1131) {
		/* UPort 1110, 1150, 1150I */
		mxport->mxp_uart_mode = MXU1_UART_232;
	} else {
		/* UPort 1130 */
		mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_DISABLED;
		mxport->mxp_user_get_uart_mode = MXU1_RS4852W;
	}

	usb_set_serial_port_data(port, mxport);

	port->port.closing_wait = msecs_to_jiffies(closing_wait * 10);
	port->port.drain_delay = 1;	

	return 0;
}

static int mxu1_startup(struct usb_serial *serial)
{
	struct mxu1_device *mxdev;
	struct usb_device *dev = serial->dev;
	char fw_name[32];
	const struct firmware *fw_p = NULL;
	u16 product_id;
	int err;

	pr_debug("%s - product 0x%4X, num configurations %d, configuration value %d",
		 __func__, le16_to_cpu(dev->descriptor.idProduct),
		 dev->descriptor.bNumConfigurations,
		 dev->actconfig->desc.bConfigurationValue);

	/* create device structure */
	mxdev = kzalloc(sizeof(struct mxu1_device), GFP_KERNEL);
	if (mxdev == NULL)
		return -ENOMEM;

	mutex_init(&mxdev->mxd_lock);
	mxdev->mxd_serial = serial;
	usb_set_serial_data(serial, mxdev);

	/* determine device type */
	if (!usb_match_id(serial->interface, mxuport11_idtable))
		return 0;

	product_id = dev->descriptor.idProduct;
	switch (product_id) {
	case MXU1_1110_PRODUCT_ID:
		mxdev->mxd_model_name = MXU1_MODEL_1110;
		break;
	case MXU1_1130_PRODUCT_ID:
		mxdev->mxd_model_name = MXU1_MODEL_1130;
		break;
	case MXU1_1131_PRODUCT_ID:
		mxdev->mxd_model_name = MXU1_MODEL_1131;
		break;
	case MXU1_1150_PRODUCT_ID:
		mxdev->mxd_model_name = MXU1_MODEL_1150;
		break;
	case MXU1_1151_PRODUCT_ID:
		mxdev->mxd_model_name = MXU1_MODEL_1151;
		break;
	default:
		return -ENODEV;
	}

	/* if we have only 1 configuration, download firmware */
	if (dev->config->interface[0]->cur_altsetting->
	    desc.bNumEndpoints == 1) {

		snprintf(fw_name,
			 sizeof(fw_name) - 1,
			 "moxa/moxa-%04x.fw",
			 le16_to_cpu(product_id));

		err = request_firmware(&fw_p, fw_name, &serial->interface->dev);
		if (err) {
			dev_err(&serial->interface->dev, "Firmware %s not found\n",
				fw_name);
			kfree(mxdev);
			return err;
		}

		err = mxu1_download_firmware(serial, fw_p);
		if (err) {
			kfree(mxdev);
			return err;
		}

	}

	if (fw_p)
		release_firmware(fw_p);

	return 0;
}

static int mxu1_write_byte(struct mxu1_device *mxdev, unsigned long addr,
			   __u8 mask, __u8 byte)
{
	int status = 0;
	unsigned int size;
	struct mxu1_write_data_bytes *data;
	struct device *dev = &mxdev->mxd_serial->dev->dev;

	pr_debug("%s - addr 0x%08lX, mask 0x%02X, byte 0x%02X", __func__,
		 addr, mask, byte);

	size = sizeof(struct mxu1_write_data_bytes) + 2;
	data = kmalloc(size, GFP_KERNEL);
	if (!data) {
		dev_err(dev, "%s - out of memory\n", __func__);
		return -ENOMEM;
	}

	data->bAddrType = MXU1_RW_DATA_ADDR_XDATA;
	data->bDataType = MXU1_RW_DATA_BYTE;
	data->bDataCounter = 1;
	data->wBaseAddrHi = cpu_to_be16(addr>>16);
	data->wBaseAddrLo = cpu_to_be16(addr);
	data->bData[0] = mask;
	data->bData[1] = byte;

	status = mxu1_send_ctrl_data_urb(mxdev->mxd_serial, MXU1_WRITE_DATA, 0,
					 MXU1_RAM_PORT,
					 (__u8 *)data,
					 size);

	if (status < 0)
		dev_err(dev, "%s - failed, %d\n", __func__, status);

	kfree(data);

	return status;
}

static int mxu1_set_mcr(struct mxu1_port *mxport, unsigned int mcr)
{
	int status = 0;

	status = mxu1_write_byte(mxport->mxp_mxdev,
				 mxport->mxp_uart_base_addr +
				 MXU1_UART_OFFSET_MCR,
				 MXU1_MCR_RTS | MXU1_MCR_DTR | MXU1_MCR_LOOP,
				 mcr);

	if (!status)
		mxport->mxp_shadow_mcr = mcr;

	return status;
}

static void mxu1_set_termios(struct tty_struct *tty1,
			     struct usb_serial_port *port,
			     struct ktermios *old_termios)
{
	struct mxu1_port *mxport = usb_get_serial_port_data(port);

	struct tty_struct *tty = port->port.tty;

	struct mxu1_uart_config *config;
	tcflag_t cflag, iflag;
	int baud;
	int status = 0;
	int port_number = port->port_number - port->minor;
	unsigned int mcr;

	pr_debug("%s - port %d", __func__, port->port_number);

	if (!tty) {
		pr_debug("%s - no tty or termios", __func__);
		return;
	}

	cflag = tty->termios.c_cflag;
	iflag = tty->termios.c_iflag;

	if (old_termios && cflag == old_termios->c_cflag
	    && iflag == old_termios->c_iflag) {
		pr_debug("%s - nothing to change", __func__);
		return;
	}

	pr_debug("%s - clfag %08x, iflag %08x", __func__, cflag, iflag);

	if (old_termios)
		pr_debug("%s - old clfag %08x, old iflag %08x",
			 __func__,
			 old_termios->c_cflag,
			 old_termios->c_iflag);

	if (mxport == NULL)
		return;

	config = kmalloc(sizeof(*config), GFP_KERNEL);
	if (!config)
		return;

	config->wFlags = 0;

	/* these flags must be set */
	config->wFlags |= MXU1_UART_ENABLE_MS_INTS;
	config->wFlags |= MXU1_UART_ENABLE_AUTO_START_DMA;
	if (mxport->mxp_send_break == MXU1_LCR_BREAK)
		config->wFlags |= MXU1_UART_SEND_BREAK_SIGNAL;
	config->bUartMode = (__u8)(mxport->mxp_uart_mode);

	switch (cflag & CSIZE) {
	case CS5:
		config->bDataBits = MXU1_UART_5_DATA_BITS;
		break;
	case CS6:
		config->bDataBits = MXU1_UART_6_DATA_BITS;
		break;
	case CS7:
		config->bDataBits = MXU1_UART_7_DATA_BITS;
		break;
	default:
	case CS8:
		config->bDataBits = MXU1_UART_8_DATA_BITS;
		break;
	}

	if (cflag & PARENB) {
		if (cflag & PARODD) {
			config->wFlags |= MXU1_UART_ENABLE_PARITY_CHECKING;
			config->bParity = MXU1_UART_ODD_PARITY;
		} else {
			config->wFlags |= MXU1_UART_ENABLE_PARITY_CHECKING;
			config->bParity = MXU1_UART_EVEN_PARITY;
		}
	} else {
		config->wFlags &= ~MXU1_UART_ENABLE_PARITY_CHECKING;
		config->bParity = MXU1_UART_NO_PARITY;
	}

	if (cflag & CSTOPB)
		config->bStopBits = MXU1_UART_2_STOP_BITS;
	else
		config->bStopBits = MXU1_UART_1_STOP_BITS;

	if (cflag & CRTSCTS) {
		/* RTS flow control must be off to drop RTS for baud rate B0 */
		if ((cflag & CBAUD) != B0)
			config->wFlags |= MXU1_UART_ENABLE_RTS_IN;
		config->wFlags |= MXU1_UART_ENABLE_CTS_OUT;
	} else {
		tty->hw_stopped = 0;
		mxu1_restart_read(mxport, tty);
	}

	if (I_IXOFF(tty) || I_IXON(tty)) {
		config->cXon  = START_CHAR(tty);
		config->cXoff = STOP_CHAR(tty);

		if (I_IXOFF(tty))
			config->wFlags |= MXU1_UART_ENABLE_X_IN;
		else
			mxu1_restart_read(mxport, tty);

		if (I_IXON(tty))
			config->wFlags |= MXU1_UART_ENABLE_X_OUT;
	}

	baud = tty_get_baud_rate(tty);
	if (!baud)
		baud = 9600;
	config->wBaudRate = (__u16)(923077 / baud);

	pr_debug("%s - BaudRate=%d, wBaudRate=%d, wFlags=0x%04X, bDataBits=%d, bParity=%d, bStopBits=%d, cXon=%d, cXoff=%d, bUartMode=%d",
		 __func__, baud, config->wBaudRate, config->wFlags,
		 config->bDataBits, config->bParity, config->bStopBits,
		 config->cXon, config->cXoff, config->bUartMode);

	cpu_to_be16s(&config->wBaudRate);
	cpu_to_be16s(&config->wFlags);

	status = mxu1_send_ctrl_data_urb(port->serial, MXU1_SET_CONFIG, 0,
					 (__u8)(MXU1_UART1_PORT + port_number),
					 (__u8 *)config,
					 sizeof(*config));
	if (status)
		dev_err(&port->dev, "%s - cannot set config on port %d, %d\n",
			__func__,
			port_number,
			status);

	/* SET_CONFIG asserts RTS and DTR, reset them correctly */
	mcr = mxport->mxp_shadow_mcr;
	/* if baud rate is B0, clear RTS and DTR */
	if ((cflag & CBAUD) == B0) {

		mcr &= ~(MXU1_MCR_DTR | MXU1_MCR_RTS);
		mxport->mxp_set_B0 = true;
	} else {
		if (mxport->mxp_set_B0)
			mcr |= MXU1_MCR_DTR;

		mxport->mxp_set_B0 = false;
	}

	status = mxu1_set_mcr(mxport, mcr);

	if (status)
		dev_err(&port->dev,
			"%s - cannot set modem control on port %d, %d\n",
			__func__,
			port_number,
			status);

	kfree(config);
}

static int mxu1_get_serial_info(struct mxu1_port *mxport,
				struct serial_struct __user *ret_arg)
{
	struct usb_serial_port *port = mxport->mxp_port;
	struct serial_struct ret_serial;
	unsigned cwait;

	if (!ret_arg)
		return -EFAULT;

	cwait = port->port.closing_wait;
	if (cwait != ASYNC_CLOSING_WAIT_NONE)
		cwait = jiffies_to_msecs(cwait) / 10;

	memset(&ret_serial, 0, sizeof(ret_serial));

	ret_serial.type = PORT_16550A;
	ret_serial.line = port->minor;
	ret_serial.port = mxport->mxp_user_get_uart_mode;
	ret_serial.flags = mxport->mxp_flags;
	ret_serial.xmit_fifo_size = port->bulk_out_size;
	ret_serial.baud_base = tty_get_baud_rate(mxport->mxp_port->port.tty);
	ret_serial.close_delay = 5*HZ;
	ret_serial.closing_wait = cwait;

	if (copy_to_user(ret_arg, &ret_serial, sizeof(*ret_arg)))
		return -EFAULT;

	return 0;
}


static int mxu1_set_serial_info(struct mxu1_port *mxport,
				struct serial_struct __user *new_arg)
{
	struct serial_struct new_serial;

	if (copy_from_user(&new_serial, new_arg, sizeof(new_serial)))
		return -EFAULT;

	mxport->mxp_flags = new_serial.flags & MXU1_SET_SERIAL_FLAGS;

	if (mxport->mxp_mxdev->mxd_model_name != MXU1_MODEL_1110) {
		/* UPort 1130, 1150, 1150I */
		switch (new_serial.port) {
		case MXU1_RS232: /* UPort 1150, 1150I */
			if (mxport->mxp_mxdev->mxd_model_name ==
			    MXU1_MODEL_1130 ||
			    mxport->mxp_mxdev->mxd_model_name ==
			    MXU1_MODEL_1131) {
				return -EINVAL;
			}

			mxport->mxp_uart_mode = MXU1_UART_232;
			mxport->mxp_user_get_uart_mode = MXU1_RS232;
			break;
		case MXU1_RS422: /* UPort 1130, 1150, 1150I */
			mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_ENABLED;
			mxport->mxp_user_get_uart_mode = MXU1_RS422;
			break;
		case MXU1_RS4854W: /* UPort 1130, 1150, 1150I */
			mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_ENABLED;
			mxport->mxp_user_get_uart_mode = MXU1_RS4854W;
			break;
		case MXU1_RS4852W: /* UPort 1130, 1150, 1150I */
			mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_DISABLED;
			mxport->mxp_user_get_uart_mode = MXU1_RS4852W;
			break;
		default:
			return -EINVAL;
		}
	} else { /* UPort 1110 */
		if (new_serial.port != MXU1_RS232)
			return -EINVAL;
	}

	mxu1_set_termios(NULL, mxport->mxp_port, NULL);

	return 0;
}

static int mxu1_ioctl(struct tty_struct *tty,
		      unsigned int cmd, unsigned long arg)
{
	struct usb_serial_port *port = tty->driver_data;

	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	struct async_icount cnow;
	struct async_icount cprev;

	pr_debug("%s - port %d, cmd = 0x%04X",
		 __func__, port->port_number, cmd);

	if (mxport == NULL)
		return -ENODEV;

	switch (cmd) {
	case TIOCGSERIAL:
		pr_debug("%s - (%d) TIOCGSERIAL", __func__, port->port_number);
		return mxu1_get_serial_info(mxport,
					    (struct serial_struct __user *)arg);

	case TIOCSSERIAL:
		pr_debug("%s - (%d) TIOCSSERIAL", __func__, port->port_number);
		return mxu1_set_serial_info(mxport,
					    (struct serial_struct __user *)arg);

	case TIOCMIWAIT:
		pr_debug("%s - (%d) TIOCMIWAIT", __func__, port->port_number);
		cprev = mxport->mxp_icount;
		mxport->mxp_msr_wait_flags = 1;
		while (1) {
			wait_event_interruptible_timeout
				(mxport->mxp_msr_wait,
				 (mxport->mxp_msr_wait_flags == 0),
				 MXU1_MSR_WAIT_TIMEOUT);

			if (signal_pending(current))
				return -ERESTARTSYS;
			cnow = mxport->mxp_icount;
			if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr &&
			    cnow.dcd == cprev.dcd && cnow.cts == cprev.cts)
				return -EIO; /* no change => error */
			if (((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
			    ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
			    ((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
			    ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts))) {
				return 0;
			}
			cprev = cnow;
		}
		break;

	case TIOCGICOUNT:
		pr_debug("%s - (%d) TIOCGICOUNT RX=%d, TX=%d",
			 __func__,
			 port->port_number, mxport->mxp_icount.rx,
			 mxport->mxp_icount.tx);

		if (copy_to_user((void __user *)arg,
				 &mxport->mxp_icount,
				 sizeof(mxport->mxp_icount)))
			return -EFAULT;
		return 0;

	case MOXA_SET_INTERFACE:
		pr_debug("%s - port%d MOXA_SET_INTERFACE=%d", __func__,
			 port->port_number, (int)arg);

		if (mxport->mxp_mxdev->mxd_model_name != MXU1_MODEL_1110) {
			/* UPort 1130, 1150, 1150I */
			switch (arg) {
			case MXU1_RS232: /* UPort 1150, 1150I */
				if (mxport->mxp_mxdev->mxd_model_name
				    == MXU1_MODEL_1130 ||
				    mxport->mxp_mxdev->mxd_model_name
				    == MXU1_MODEL_1131) {
					return -EINVAL;
				}
				mxport->mxp_uart_mode = MXU1_UART_232;
				break;
			case MXU1_RS422: /* UPort 1130, 1150, 1150I */
			case MXU1_RS4854W: /* UPort 1130, 1150, 1150I */
				mxport->mxp_uart_mode =
					MXU1_UART_485_RECEIVER_ENABLED;
				break;

			case MXU1_RS4852W: /* UPort 1130, 1150, 1150I */
				mxport->mxp_uart_mode =
					MXU1_UART_485_RECEIVER_DISABLED;
				break;
			default:
				return -EINVAL;
			}
		} else { /* UPort 1110 */
			if (arg != MXU1_RS232)
				return -EINVAL;
		}

		mxu1_set_termios(NULL, port, NULL);
	}

	return -ENOIOCTLCMD;
}

static int mxu1_tiocmget(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;

	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	unsigned int result;
	unsigned int msr;
	unsigned int mcr;

	pr_debug("%s - port %d", __func__, port->port_number);

	if (mxport == NULL)
		return -ENODEV;

	msr = mxport->mxp_msr;
	mcr = mxport->mxp_shadow_mcr;

	result = ((mcr & MXU1_MCR_DTR) ? TIOCM_DTR : 0)
		| ((mcr & MXU1_MCR_RTS) ? TIOCM_RTS : 0)
		| ((mcr & MXU1_MCR_LOOP) ? TIOCM_LOOP : 0)
		| ((msr & MXU1_MSR_CTS) ? TIOCM_CTS : 0)
		| ((msr & MXU1_MSR_CD) ? TIOCM_CAR : 0)
		| ((msr & MXU1_MSR_RI) ? TIOCM_RI : 0)
		| ((msr & MXU1_MSR_DSR) ? TIOCM_DSR : 0);

	pr_debug("%s - 0x%04X", __func__, result);

	return result;
}

static int mxu1_tiocmset(struct tty_struct *tty,
			 unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;

	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	unsigned int mcr;

	pr_debug("%s - port %d", __func__, port->port_number);

	if (mxport == NULL)
		return -ENODEV;

	mcr = mxport->mxp_shadow_mcr;

	if (set & TIOCM_RTS)
		mcr |= MXU1_MCR_RTS;
	if (set & TIOCM_DTR)
		mcr |= MXU1_MCR_DTR;
	if (set & TIOCM_LOOP)
		mcr |= MXU1_MCR_LOOP;

	if (clear & TIOCM_RTS)
		mcr &= ~MXU1_MCR_RTS;
	if (clear & TIOCM_DTR)
		mcr &= ~MXU1_MCR_DTR;
	if (clear & TIOCM_LOOP)
		mcr &= ~MXU1_MCR_LOOP;

	return mxu1_set_mcr(mxport, mcr);
}

static int mxu1_get_icount(struct tty_struct *tty,
			   struct serial_icounter_struct *icount)
{
	struct usb_serial_port *port = tty->driver_data;
	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	struct async_icount cnow = mxport->mxp_icount;

	pr_debug("%s - (%d) TIOCGICOUNT RX=%d, TX=%d",
		 __func__, port->port_number,
		 cnow.rx, cnow.tx);

	icount->cts = cnow.cts;
	icount->dsr = cnow.dsr;
	icount->rng = cnow.rng;
	icount->dcd = cnow.dcd;
	icount->rx = cnow.rx;
	icount->tx = cnow.tx;
	icount->frame = cnow.frame;
	icount->overrun = cnow.overrun;
	icount->parity = cnow.parity;
	icount->brk = cnow.brk;
	icount->buf_overrun = cnow.buf_overrun;

	return 0;
}

static void mxu1_throttle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;

	struct mxu1_port *mxport = usb_get_serial_port_data(port);

	pr_debug("%s - port %d", __func__, port->port_number);

	if (mxport == NULL)
		return;

	tty = port->port.tty;

	if (!tty) {
		pr_debug("%s - no tty", __func__);
		return;
	}

	if (I_IXOFF(tty) || C_CRTSCTS(tty))
		mxu1_stop_read(mxport, tty);

}

static void mxu1_unthrottle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;

	struct mxu1_port *mxport = usb_get_serial_port_data(port);

	int status = 0;

	pr_debug("%s - port %d", __func__, port->port_number);

	if (mxport == NULL)
		return;

	tty = port->port.tty;

	if (!tty) {
		pr_debug("%s - no tty", __func__);
		return;
	}

	if (I_IXOFF(tty) || C_CRTSCTS(tty)) {
		status = mxu1_restart_read(mxport, tty);
		if (status)
			dev_err(&port->dev, "%s - cannot restart read, %d\n",
				__func__, status);
	}
}

static void mxu1_break(struct tty_struct *tty, int break_state)
{
	struct usb_serial_port *port = tty->driver_data;

	struct mxu1_port *mxport = usb_get_serial_port_data(port);

	pr_debug("%s - state = %d", __func__, break_state);

	if (mxport == NULL)
		return;

	if (break_state == -1)
		mxport->mxp_send_break = MXU1_LCR_BREAK;
	else
		mxport->mxp_send_break = 0;

	mxu1_set_termios(NULL, mxport->mxp_port, NULL);
}

static int mxu1_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	struct mxu1_device *mxdev;
	struct usb_device *dev;
	struct urb *urb;
	int port_number;
	int status;
	__u16 open_settings = (__u8)(MXU1_PIPE_MODE_CONTINOUS |
				     MXU1_PIPE_TIMEOUT_ENABLE |
				     (MXU1_TRANSFER_TIMEOUT << 2));
	if (!mxport)
		return -ENODEV;

	dev = port->serial->dev;
	mxdev = mxport->mxp_mxdev;

	if (port->port.tty)
		port->port.low_latency = MXU1_DEFAULT_LOW_LATENCY;

	port_number = port->port_number - port->minor;

	mxport->mxp_msr = 0;
	mxport->mxp_shadow_mcr |= (MXU1_MCR_RTS | MXU1_MCR_DTR);

	if (mutex_lock_interruptible(&mxdev->mxd_lock))
		return -ERESTARTSYS;

	/* start interrupt urb the first time a port is opened on this device */
	if (mxdev->mxd_open_port_count == 0) {
		pr_debug("%s - start interrupt in urb", __func__);
		urb = mxdev->mxd_serial->port[0]->interrupt_in_urb;
		if (!urb) {
			dev_err(&port->dev,
				"%s - no interrupt urb\n",
				__func__);
			status = -EINVAL;
			goto release_mxd_lock;
		}
		urb->context = mxdev;
		status = usb_submit_urb(urb, GFP_KERNEL);
		if (status) {
			dev_err(&port->dev,
				"%s - submit interrupt urb failed, %d\n",
				__func__,
				status);
			goto release_mxd_lock;
		}
	}

	mxu1_set_termios(NULL, port, NULL);

	pr_debug("%s - sending MXU1_OPEN_PORT", __func__);
	status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_OPEN_PORT,
				    open_settings,
				    (__u8)(MXU1_UART1_PORT + port_number));
	if (status) {
		dev_err(&port->dev, "%s - cannot send open command, %d\n",
			__func__,
			status);
		goto unlink_int_urb;
	}

	pr_debug("%s - sending MXU1_START_PORT", __func__);
	status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_START_PORT,
				    0, (__u8)(MXU1_UART1_PORT + port_number));
	if (status) {
		dev_err(&port->dev, "%s - cannot send start command, %d\n",
			__func__,
			status);
		goto unlink_int_urb;
	}

	pr_debug("%s - sending MXU1_PURGE_PORT", __func__);
	status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_PURGE_PORT,
				    MXU1_PURGE_INPUT,
				    (__u8)(MXU1_UART1_PORT + port_number));
	if (status) {
		dev_err(&port->dev,
			"%s - cannot clear input buffers, %d\n",
			__func__,
			status);

		goto unlink_int_urb;
	}
	status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_PURGE_PORT,
				    MXU1_PURGE_OUTPUT,
				    (__u8)(MXU1_UART1_PORT + port_number));
	if (status) {
		dev_err(&port->dev,
			"%s - cannot clear output buffers, %d\n",
			__func__,
			status);

		goto unlink_int_urb;
	}

	/* reset the data toggle on the bulk endpoints to work around bug in
	 * host controllers where things get out of sync some times */
	usb_clear_halt(dev, port->write_urb->pipe);
	usb_clear_halt(dev, port->read_urb->pipe);

	mxu1_set_termios(NULL, port, NULL);

	pr_debug("%s - sending MXU1_OPEN_PORT", __func__);
	status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_OPEN_PORT,
				    open_settings,
				    (__u8)(MXU1_UART1_PORT + port_number));
	if (status) {
		dev_err(&port->dev, "%s - cannot send open command, %d\n",
			__func__,
			status);
		goto unlink_int_urb;
	}

	pr_debug("%s - sending MXU1_START_PORT", __func__);
	status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_START_PORT,
				    0, (__u8)(MXU1_UART1_PORT + port_number));
	if (status) {
		dev_err(&port->dev, "%s - cannot send start command, %d\n",
			__func__,
			status);
		goto unlink_int_urb;
	}

	/* start read urb */
	pr_debug("%s - start read urb", __func__);
	urb = port->read_urb;

	if (!urb) {
		dev_err(&port->dev, "%s - no read urb\n", __func__);
		status = -EINVAL;
		goto unlink_int_urb;
	}

	mxport->mxp_read_urb_state = MXU1_READ_URB_RUNNING;
	urb->context = mxport;
	urb->dev = dev;
	status = usb_submit_urb(urb, GFP_KERNEL);

	if (status) {
		dev_err(&port->dev, "%s - submit read urb failed, %d\n",
			__func__, status);
		goto unlink_int_urb;
	}

	mxport->mxp_is_open = 1;
	++mxdev->mxd_open_port_count;

	goto release_mxd_lock;

unlink_int_urb:
	if (mxdev->mxd_open_port_count == 0)
		usb_kill_urb(port->serial->port[0]->interrupt_in_urb);

release_mxd_lock:
	mutex_unlock(&mxdev->mxd_lock);
	pr_debug("%s - exit %d", __func__, status);

	return status;
}

static void mxu1_close(struct usb_serial_port *port)
{
	struct mxu1_device *mxdev;
	struct mxu1_port *mxport;
	int port_number;
	unsigned long flags;
	int status = 0;
	
	pr_debug("%s - port %d", __func__, port->port_number);

	mxdev = usb_get_serial_data(port->serial);
	mxport = usb_get_serial_port_data(port);
	if (mxdev == NULL || mxport == NULL)
		return;

	mxport->mxp_is_open = 0;

	usb_kill_urb(port->read_urb);
	usb_kill_urb(port->write_urb);

	mxport->mxp_write_urb_in_use = 0;

	spin_lock_irqsave(&port->lock, flags);
	kfifo_reset_out(&port->write_fifo);
	spin_unlock_irqrestore(&port->lock, flags);

	port_number = port->port_number - port->minor;

	pr_debug("%s - sending MXU1_CLOSE_PORT", __func__);
	status = mxu1_send_ctrl_urb(port->serial,
				    MXU1_CLOSE_PORT,
				    0, (__u8)(MXU1_UART1_PORT + port_number));
	if (status)
		dev_err(&port->dev,
			"%s - cannot send close port command, %d\n",
			__func__,
			status);

	mutex_lock(&mxdev->mxd_lock);
	--mxport->mxp_mxdev->mxd_open_port_count;
	if (mxport->mxp_mxdev->mxd_open_port_count <= 0) {
		usb_kill_urb(port->serial->port[0]->interrupt_in_urb);

		mxport->mxp_mxdev->mxd_open_port_count = 0;
	}
	mutex_unlock(&mxdev->mxd_lock);

	pr_debug("%s - exit", __func__);
}

static void mxu1_handle_new_msr(struct mxu1_port *mxport, __u8 msr)
{
	struct async_icount *icount;
	struct tty_struct *tty;
	unsigned long flags;

	pr_debug("%s - msr 0x%02X", __func__, msr);

	if (msr & MXU1_MSR_DELTA_MASK) {
		spin_lock_irqsave(&mxport->mxp_lock, flags);
		pr_debug("mxp_lock passed");
		icount = &mxport->mxp_icount;
		if (msr & MXU1_MSR_DELTA_CTS)
			icount->cts++;
		if (msr & MXU1_MSR_DELTA_DSR)
			icount->dsr++;
		if (msr & MXU1_MSR_DELTA_CD)
			icount->dcd++;
		if (msr & MXU1_MSR_DELTA_RI)
			icount->rng++;
		if (mxport->mxp_msr_wait_flags == 1) {
			mxport->mxp_msr_wait_flags = 0;
			pr_debug("wait interruptible");
			wake_up_interruptible(&mxport->mxp_msr_wait);
		}
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
	}

	mxport->mxp_msr = msr & MXU1_MSR_MASK;

	/* handle CTS flow control */
	tty = mxport->mxp_port->port.tty;

	if (tty && C_CRTSCTS(tty)) {
		if (msr & MXU1_MSR_CTS) {
			tty->hw_stopped = 0;

			tty_wakeup(tty);
		} else {
			tty->hw_stopped = 1;
		}
	}

	pr_debug("end msr");
}

static void mxu1_interrupt_callback(struct urb *urb)
{
	struct mxu1_device *mxdev = (struct mxu1_device *)urb->context;
	struct usb_serial_port *port;
	struct usb_serial *serial = mxdev->mxd_serial;
	struct mxu1_port *mxport;
	struct device *dev = &urb->dev->dev;
	unsigned char *data = urb->transfer_buffer;
	int length = urb->actual_length;
	int port_number;
	int function;
	int status = 0;
	__u8 msr;

	pr_debug("%s", __func__);

	/* Check port is valid or not */
	if (mxdev == NULL)
		return;


	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		pr_debug("%s - urb shutting down, %d", __func__, urb->status);
		return;
	default:
		dev_err(dev, "%s - nonzero urb status, %d\n",
			__func__, urb->status);
		goto exit;
	}

	if (length != 2) {
		pr_debug("%s - bad packet size, %d", __func__, length);
		goto exit;
	}

	if (data[0] == MXU1_CODE_HARDWARE_ERROR) {
		dev_err(dev, "%s - hardware error, %d\n", __func__, data[1]);
		goto exit;
	}

	port_number = MXU1_GET_PORT_FROM_CODE(data[0]);
	function = MXU1_GET_FUNC_FROM_CODE(data[0]);

	pr_debug("%s - port_number %d, function %d, data 0x%02X",
		 __func__,
		 port_number,
		 function,
		 data[1]);

	if (port_number >= serial->num_ports) {
		dev_err(dev, "%s - bad port number, %d\n",
			__func__, port_number);
		goto exit;
	}

	port = serial->port[port_number];

	mxport = usb_get_serial_port_data(port);
	if (!mxport)
		goto exit;

	switch (function) {
	case MXU1_CODE_DATA_ERROR:
		pr_debug("%s - DATA ERROR, port %d, data 0x%02X\n",
			 __func__,
			 port_number,
			 data[1]);
		break;

	case MXU1_CODE_MODEM_STATUS:
		msr = data[1];
		pr_debug("%s - port %d, msr 0x%02X",
			 __func__, port_number, msr);
		mxu1_handle_new_msr(mxport, msr);
		break;

	default:
		dev_err(dev, "%s - unknown interrupt code, 0x%02X\n",
			__func__, data[1]);
		break;
	}

exit:
	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status)
		dev_err(dev, "%s - resubmit interrupt urb failed, %d\n",
			__func__, status);
}

static void mxu1_bulk_in_callback(struct urb *urb)
{
	struct mxu1_port *mxport = (struct mxu1_port *)urb->context;
	struct usb_serial_port *port = mxport->mxp_port;
	struct device *dev = &urb->dev->dev;
	int status = 0;

	pr_debug("%s", __func__);

	/*Check port is valid or not*/
	if (mxport == NULL)
		return;

	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		pr_debug("%s - urb shutting down, %d", __func__, urb->status);
		return;
	default:
		dev_err(dev, "%s - nonzero urb status, %d\n",
			__func__, urb->status);
	}

	if (urb->status == -EPIPE)
		goto exit;

	if (urb->status) {
		dev_err(dev, "%s - stopping read!\n", __func__);
		return;
	}
	if (port->port.tty && urb->actual_length) {

		usb_serial_debug_data(dev, __func__,
				      urb->actual_length, urb->transfer_buffer);

		if (!mxport->mxp_is_open)
			pr_debug("%s - port closed, dropping data", __func__);
		else
			mxu1_recv(mxport,
				  urb->transfer_buffer, urb->actual_length);

		spin_lock(&mxport->mxp_lock);
		mxport->mxp_icount.rx += urb->actual_length;
		spin_unlock(&mxport->mxp_lock);
	}

exit:
	/* continue to read unless stopping */
	spin_lock(&mxport->mxp_lock);
	if (mxport->mxp_read_urb_state == MXU1_READ_URB_RUNNING) {
		urb->dev = port->serial->dev;
		status = usb_submit_urb(urb, GFP_ATOMIC);
	} else if (mxport->mxp_read_urb_state == MXU1_READ_URB_STOPPING) {
		mxport->mxp_read_urb_state = MXU1_READ_URB_STOPPED;
	}
	spin_unlock(&mxport->mxp_lock);
	if (status)
		dev_err(dev, "%s - resubmit read urb failed, %d\n",
			__func__, status);
}


static void mxu1_bulk_out_callback(struct urb *urb)
{
	struct mxu1_port *mxport = (struct mxu1_port *)urb->context;
	struct usb_serial_port *port = mxport->mxp_port;

	pr_debug("%s - port %d", __func__, port->port_number);

	/* Check port is valid or not */
	if (mxport == NULL)
		return;

	mxport->mxp_write_urb_in_use = 0;

	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		pr_debug("%s - urb shutting down, %d", __func__, urb->status);
		return;
	default:
		dev_err_console(port, "%s - nonzero urb status, %d\n",
				__func__, urb->status);
	}

	/* send any buffered data */
	mxu1_send(mxport);
}

static void mxu1_send(struct mxu1_port *mxport)
{
	int count, result;
	struct usb_serial_port *port = mxport->mxp_port;

	struct tty_struct *tty = port->port.tty;

	unsigned long flags;

	pr_debug("%s - port %d", __func__, port->port_number);

	spin_lock_irqsave(&mxport->mxp_lock, flags);

	if (mxport->mxp_write_urb_in_use) {
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
		return;
	}

	count = kfifo_out(&port->write_fifo,
			  port->write_urb->transfer_buffer,
			  port->bulk_out_size);

	if (count == 0) {
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
		return;
	}

	mxport->mxp_write_urb_in_use = 1;

	spin_unlock_irqrestore(&mxport->mxp_lock, flags);

	usb_serial_debug_data(&port->dev,
			      __func__,
			      count,
			      port->write_urb->transfer_buffer);

	usb_fill_bulk_urb(port->write_urb, port->serial->dev,
			  usb_sndbulkpipe(port->serial->dev,
					  port->bulk_out_endpointAddress),
			  port->write_urb->transfer_buffer, count,
			  mxu1_bulk_out_callback, mxport);

	result = usb_submit_urb(port->write_urb, GFP_ATOMIC);
	if (result) {
		dev_err_console(port, "%s - submit write urb failed, %d\n",
				__func__, result);

		mxport->mxp_write_urb_in_use = 0;
		/* TODO: reschedule mxu1_send */
	} else {
		spin_lock_irqsave(&mxport->mxp_lock, flags);
		mxport->mxp_icount.tx += count;
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
	}

	/* more room in the buffer for new writes, wakeup */
	if (tty)
		tty_wakeup(tty);
}

static int mxu1_write(struct tty_struct *tty,
		      struct usb_serial_port *port,
		      const unsigned char *data,
		      int count)
{
	struct mxu1_port *mxport = usb_get_serial_port_data(port);

	pr_debug("%s - port %d", __func__, port->port_number);

	if (count == 0) {
		pr_debug("%s - write request of 0 bytes", __func__);
		return 0;
	}

	if (mxport == NULL || !mxport->mxp_is_open)
		return -ENODEV;

	count = kfifo_in_locked(&port->write_fifo, data, count,
				&mxport->mxp_lock);
	mxu1_send(mxport);

	return count;
}

static void mxu1_recv(struct mxu1_port *mxport,
		      unsigned char *data, int length)
{
	int queued;
	struct tty_struct *tty;

	tty = mxport->mxp_port->port.tty;

	queued = tty_insert_flip_string(tty->port, data, length);
	if (queued < length)
		dev_err(&mxport->mxp_port->dev,
			"%s - dropping data, %d bytes lost\n",
			__func__, length - queued);
	tty_flip_buffer_push(tty->port);
}

static struct usb_serial_driver mxuport11_device = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "mxuport11",
	},
	.description		= "MOXA UPort 11x0",
	.id_table		= mxuport11_idtable,
	.num_ports		= 1,
	.port_probe             = mxu1_port_probe,
	.attach			= mxu1_startup,
	.open			= mxu1_open,
	.close			= mxu1_close,
	.ioctl			= mxu1_ioctl,
	.throttle		= mxu1_throttle,
	.unthrottle		= mxu1_unthrottle,
	.set_termios		= mxu1_set_termios,
	.tiocmget		= mxu1_tiocmget,
	.tiocmset		= mxu1_tiocmset,
	.get_icount		= mxu1_get_icount,
	.write                  = mxu1_write,
	.break_ctl		= mxu1_break,
	.read_int_callback	= mxu1_interrupt_callback,
	.read_bulk_callback	= mxu1_bulk_in_callback,
	.write_bulk_callback	= mxu1_bulk_out_callback

};

static struct usb_serial_driver *const serial_drivers[] = {
	&mxuport11_device, NULL
};

module_usb_serial_driver(serial_drivers, mxuport11_idtable);

MODULE_AUTHOR("Ken Huang");
MODULE_AUTHOR("Mathieu Othacehe <m.othacehe@gmail.com>");
MODULE_DESCRIPTION("MOXA UPort 11x0 USB to Serial Hub Driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("moxa/moxa-1110.fw");
MODULE_FIRMWARE("moxa/moxa-1130.fw");
MODULE_FIRMWARE("moxa/moxa-1131.fw");
MODULE_FIRMWARE("moxa/moxa-1150.fw");
MODULE_FIRMWARE("moxa/moxa-1151.fw");


module_param(closing_wait, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(closing_wait, "Maximum wait for data to drain, in .01 secs");
