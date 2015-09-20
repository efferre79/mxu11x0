/*
 *
 * USB Moxa UPORT 11x0 Serial Driver
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

/* Vendor and product ids */
#define MXU1_VENDOR_ID				0x110a
#define MXU1_1110_PRODUCT_ID			0x1110
#define MXU1_1130_PRODUCT_ID			0x1130
#define MXU1_1150_PRODUCT_ID			0x1150
#define MXU1_1151_PRODUCT_ID			0x1151
#define MXU1_1131_PRODUCT_ID			0x1131

/* Commands */
#define MXU1_GET_VERSION			0x01
#define MXU1_GET_PORT_STATUS			0x02
#define MXU1_GET_PORT_DEV_INFO			0x03
#define MXU1_GET_CONFIG				0x04
#define MXU1_SET_CONFIG				0x05
#define MXU1_OPEN_PORT				0x06
#define MXU1_CLOSE_PORT				0x07
#define MXU1_START_PORT				0x08
#define MXU1_STOP_PORT				0x09
#define MXU1_TEST_PORT				0x0A
#define MXU1_PURGE_PORT				0x0B
#define MXU1_RESET_EXT_DEVICE			0x0C
#define MXU1_GET_OUTQUEUE			0x0D
#define MXU1_WRITE_DATA				0x80
#define MXU1_READ_DATA				0x81
#define MXU1_REQ_TYPE_CLASS			0x82

/* Module identifiers */
#define MXU1_I2C_PORT				0x01
#define MXU1_IEEE1284_PORT			0x02
#define MXU1_UART1_PORT				0x03
#define MXU1_UART2_PORT				0x04
#define MXU1_RAM_PORT				0x05

/* Modem status */
#define MXU1_MSR_DELTA_CTS			0x01
#define MXU1_MSR_DELTA_DSR			0x02
#define MXU1_MSR_DELTA_RI			0x04
#define MXU1_MSR_DELTA_CD			0x08
#define MXU1_MSR_CTS				0x10
#define MXU1_MSR_DSR				0x20
#define MXU1_MSR_RI				0x40
#define MXU1_MSR_CD				0x80
#define MXU1_MSR_DELTA_MASK			0x0F
#define MXU1_MSR_MASK				0xF0

/* Line status */
#define MXU1_LSR_OVERRUN_ERROR			0x01
#define MXU1_LSR_PARITY_ERROR			0x02
#define MXU1_LSR_FRAMING_ERROR			0x04
#define MXU1_LSR_BREAK				0x08
#define MXU1_LSR_ERROR				0x0F
#define MXU1_LSR_RX_FULL			0x10
#define MXU1_LSR_TX_EMPTY			0x20

/* Line control */
#define MXU1_LCR_BREAK				0x40

/* Modem control */
#define MXU1_MCR_LOOP				0x04
#define MXU1_MCR_DTR				0x10
#define MXU1_MCR_RTS				0x20

/* Mask settings */
#define MXU1_UART_ENABLE_RTS_IN			0x0001
#define MXU1_UART_DISABLE_RTS			0x0002
#define MXU1_UART_ENABLE_PARITY_CHECKING	0x0008
#define MXU1_UART_ENABLE_DSR_OUT		0x0010
#define MXU1_UART_ENABLE_CTS_OUT		0x0020
#define MXU1_UART_ENABLE_X_OUT			0x0040
#define MXU1_UART_ENABLE_XA_OUT			0x0080
#define MXU1_UART_ENABLE_X_IN			0x0100
#define MXU1_UART_ENABLE_DTR_IN			0x0800
#define MXU1_UART_DISABLE_DTR			0x1000
#define MXU1_UART_ENABLE_MS_INTS		0x2000
#define MXU1_UART_ENABLE_AUTO_START_DMA		0x4000
#define MXU1_UART_SEND_BREAK_SIGNAL		0x8000

/* Parity */
#define MXU1_UART_NO_PARITY			0x00
#define MXU1_UART_ODD_PARITY			0x01
#define MXU1_UART_EVEN_PARITY			0x02
#define MXU1_UART_MARK_PARITY			0x03
#define MXU1_UART_SPACE_PARITY			0x04

/* Stop bits */
#define MXU1_UART_1_STOP_BITS			0x00
#define MXU1_UART_1_5_STOP_BITS			0x01
#define MXU1_UART_2_STOP_BITS			0x02

/* Bits per character */
#define MXU1_UART_5_DATA_BITS			0x00
#define MXU1_UART_6_DATA_BITS			0x01
#define MXU1_UART_7_DATA_BITS			0x02
#define MXU1_UART_8_DATA_BITS			0x03

/* Operation modes */
#define MXU1_UART_232				0x00
#define MXU1_UART_485_RECEIVER_DISABLED		0x01
#define MXU1_UART_485_RECEIVER_ENABLED		0x02
#define MXU1_TYPE_RS232				(1 << 0)
#define MXU1_TYPE_RS422				(1 << 1)
#define MXU1_TYPE_RS485				(1 << 2)

/* Pipe transfer mode and timeout */
#define MXU1_PIPE_MODE_CONTINUOUS		0x01
#define MXU1_PIPE_MODE_MASK			0x03
#define MXU1_PIPE_TIMEOUT_MASK			0x7C
#define MXU1_PIPE_TIMEOUT_ENABLE		0x80

/* User define ioctl */
#define MOXA					404
#define MOXA_SET_INTERFACE			(MOXA + 1)

/* Config struct */
struct mxu1_uart_config {
	__be16	wBaudRate;
	__be16	wFlags;
	u8	bDataBits;
	u8	bParity;
	u8	bStopBits;
	char	cXon;
	char	cXoff;
	u8	bUartMode;
} __packed;

/* Purge modes */
#define MXU1_PURGE_OUTPUT			0x00
#define MXU1_PURGE_INPUT			0x80

/* Read/Write data */
#define MXU1_RW_DATA_ADDR_SFR			0x10
#define MXU1_RW_DATA_ADDR_IDATA			0x20
#define MXU1_RW_DATA_ADDR_XDATA			0x30
#define MXU1_RW_DATA_ADDR_CODE			0x40
#define MXU1_RW_DATA_ADDR_GPIO			0x50
#define MXU1_RW_DATA_ADDR_I2C			0x60
#define MXU1_RW_DATA_ADDR_FLASH			0x70
#define MXU1_RW_DATA_ADDR_DSP			0x80

#define MXU1_RW_DATA_UNSPECIFIED		0x00
#define MXU1_RW_DATA_BYTE			0x01
#define MXU1_RW_DATA_WORD			0x02
#define MXU1_RW_DATA_DOUBLE_WORD		0x04

struct mxu1_write_data_bytes {
	u8	bAddrType;
	u8	bDataType;
	u8	bDataCounter;
	__be16	wBaseAddrHi;
	__be16	wBaseAddrLo;
	u8	bData[0];
} __packed;

/* Interrupt codes */
#define MXU1_GET_PORT_FROM_CODE(c)		(((c) >> 4) - 3)
#define MXU1_GET_FUNC_FROM_CODE(c)		((c) & 0x0f)
#define MXU1_CODE_HARDWARE_ERROR		0xFF
#define MXU1_CODE_DATA_ERROR			0x03
#define MXU1_CODE_MODEM_STATUS			0x04

/* Download firmware max packet size */
#define MXU1_DOWNLOAD_MAX_PACKET_SIZE		64

/* Firmware image header */
struct mxu1_firmware_header {
	__le16 wLength;
	u8 bCheckSum;
} __packed;

/* UART addresses */
/* UART 1 base address */
#define MXU1_UART1_BASE_ADDR			0xFFA0
/* UART 2 base address*/
#define MXU1_UART2_BASE_ADDR			0xFFB0
#define MXU1_UART_OFFSET_LCR			0x0002
/*UART MCR register offset */
#define MXU1_UART_OFFSET_MCR			0x0004

/* supported setserial flags */
#define MXU1_SET_SERIAL_FLAGS	    (ASYNC_LOW_LATENCY)

#define MXU1_DEFAULT_LOW_LATENCY    1

#define MXU1_TRANSFER_TIMEOUT	    2
#define MXU1_MSR_WAIT_TIMEOUT	    (5 * HZ)

/* Configuration ids */
#define MXU1_BOOT_CONFIG	    1
#define MXU1_ACTIVE_CONFIG	    2

#define MXU1_DEFAULT_CLOSING_WAIT   4000		/* in .01 secs */

struct mxu1_port {
	u8 mxp_msr;
	u8 mxp_lsr;
	u8 mxp_shadow_mcr;
	u8 mxp_uart_types;
	u8 mxp_uart_mode;
	unsigned int mxp_uart_base_addr;
	int mxp_flags;
	int mxp_msr_wait_flags;
	wait_queue_head_t mxp_msr_wait;	/* wait for msr change */
	struct mxu1_device *mxp_mxdev;
	struct usb_serial_port *mxp_port;
	spinlock_t mxp_lock;
	int mxp_send_break;
	int mxp_set_B0;
};

struct mxu1_device {
	struct mutex mxd_lock;
	struct usb_serial *mxd_serial;
	u16 mxd_model;
};

static const struct usb_device_id mxuport11_idtable[] = {
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1110_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1130_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1150_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1151_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1131_PRODUCT_ID) },
	{ }
};

MODULE_DEVICE_TABLE(usb, mxuport11_idtable);

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

static int mxu1_download_firmware(struct usb_serial *serial,
				  const struct firmware *fw_p)
{
	int status = 0;
	int buffer_size;
	int pos;
	int len;
	int done;
	u8 cs = 0;
	u8 *buffer;
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
		cs = (u8)(cs + buffer[pos]);

	header = (struct mxu1_firmware_header *)buffer;
	header->wLength = cpu_to_le16(
		(__u16)(buffer_size - sizeof(struct mxu1_firmware_header)));
	header->bCheckSum = cs;

	dev_dbg(&dev->dev, "%s - downloading firmware", __func__);

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

	dev_dbg(&dev->dev, "%s - download successful (%d)", __func__, status);

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

	switch (mxport->mxp_mxdev->mxd_model) {
	case MXU1_1110_PRODUCT_ID:
		mxport->mxp_uart_types = MXU1_TYPE_RS232;
		mxport->mxp_uart_mode = MXU1_UART_232;
		break;
	case MXU1_1130_PRODUCT_ID:
	case MXU1_1131_PRODUCT_ID:
		mxport->mxp_uart_types = MXU1_TYPE_RS422 | MXU1_TYPE_RS485;
		mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_DISABLED;
		break;
	case MXU1_1150_PRODUCT_ID:
	case MXU1_1151_PRODUCT_ID:
		mxport->mxp_uart_types =
			MXU1_TYPE_RS232 | MXU1_TYPE_RS422 | MXU1_TYPE_RS485;
		mxport->mxp_uart_mode = MXU1_UART_232;
		break;
	}
	
	usb_set_serial_port_data(port, mxport);

	port->port.closing_wait =		
		msecs_to_jiffies(MXU1_DEFAULT_CLOSING_WAIT * 10);
	port->port.drain_delay = 1;

	return 0;
}

static int mxu1_startup(struct usb_serial *serial)
{
	struct mxu1_device *mxdev;
	struct usb_device *dev = serial->dev;
	char fw_name[32];
	const struct firmware *fw_p = NULL;
	int err;

	dev_dbg(&dev->dev, "%s - product 0x%4X, num configurations %d, configuration value %d",
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

	mxdev->mxd_model = le16_to_cpu(dev->descriptor.idProduct);

	/* if we have only 1 configuration, download firmware */
	if (dev->config->interface[0]->cur_altsetting->
	    desc.bNumEndpoints == 1) {

		snprintf(fw_name,
			 sizeof(fw_name) - 1,
			 "moxa/moxa-%04x.fw",
			 mxdev->mxd_model);

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
			   u8 mask, u8 byte)
{
	int status = 0;
	unsigned int size;
	struct mxu1_write_data_bytes *data;
	struct device *dev = &mxdev->mxd_serial->dev->dev;

	dev_dbg(dev, "%s - addr 0x%08lX, mask 0x%02X, byte 0x%02X", __func__,
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
					 (u8 *)data,
					 size);

	if (status < 0)
		dev_err(dev, "%s - failed, %d\n", __func__, status);

	kfree(data);

	return status;
}

static int mxu1_set_mcr(struct mxu1_port *mxport, unsigned int mcr)
{
	int status = 0;
	unsigned long flags;
	
	status = mxu1_write_byte(mxport->mxp_mxdev,
				 mxport->mxp_uart_base_addr +
				 MXU1_UART_OFFSET_MCR,
				 MXU1_MCR_RTS | MXU1_MCR_DTR | MXU1_MCR_LOOP,
				 mcr);

	spin_lock_irqsave(&mxport->mxp_lock, flags);	
	if (!status)
		mxport->mxp_shadow_mcr = mcr;
	spin_unlock_irqrestore(&mxport->mxp_lock, flags);	
	
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

	dev_dbg(&port->dev, "%s - port %d", __func__, port->port_number);

	if (!tty) {
		dev_dbg(&port->dev, "%s - no tty or termios", __func__);
		return;
	}

	cflag = tty->termios.c_cflag;
	iflag = tty->termios.c_iflag;

	if (old_termios && cflag == old_termios->c_cflag
	    && iflag == old_termios->c_iflag) {
		dev_dbg(&port->dev, "%s - nothing to change", __func__);
		return;
	}

	dev_dbg(&port->dev,
		"%s - clfag %08x, iflag %08x", __func__, cflag, iflag);

	if (old_termios)
		dev_dbg(&port->dev, "%s - old clfag %08x, old iflag %08x",
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
	config->bUartMode = (u8)(mxport->mxp_uart_mode);

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
		//mxu1_restart_read(mxport, tty);
	}

	if (I_IXOFF(tty) || I_IXON(tty)) {
		config->cXon  = START_CHAR(tty);
		config->cXoff = STOP_CHAR(tty);

		if (I_IXOFF(tty))
			config->wFlags |= MXU1_UART_ENABLE_X_IN;
		else
			;//mxu1_restart_read(mxport, tty);

		if (I_IXON(tty))
			config->wFlags |= MXU1_UART_ENABLE_X_OUT;
	}

	baud = tty_get_baud_rate(tty);
	if (!baud)
		baud = 9600;
	config->wBaudRate = (__u16)(923077 / baud);

	dev_dbg(&port->dev, "%s - BaudRate=%d, wBaudRate=%d, wFlags=0x%04X, bDataBits=%d, bParity=%d, bStopBits=%d, cXon=%d, cXoff=%d, bUartMode=%d",
		__func__, baud, config->wBaudRate, config->wFlags,
		config->bDataBits, config->bParity, config->bStopBits,
		config->cXon, config->cXoff, config->bUartMode);

	cpu_to_be16s(&config->wBaudRate);
	cpu_to_be16s(&config->wFlags);

	status = mxu1_send_ctrl_data_urb(port->serial, MXU1_SET_CONFIG, 0,
					 (u8)(MXU1_UART1_PORT + port_number),
					 (u8 *)config,
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

static int mxu1_ioctl_get_rs485(struct mxu1_port *mxport,
				struct serial_rs485 __user *rs485) {
	struct serial_rs485 aux;

	memset(&aux, 0, sizeof(aux));
	
	if (mxport->mxp_uart_mode == MXU1_UART_485_RECEIVER_ENABLED) {
		aux.flags = SER_RS485_ENABLED;
	}
	
	if (copy_to_user(rs485, &aux, sizeof(aux)))
		return -EFAULT;

	return 0;
}

static int mxu1_ioctl_set_rs485(struct mxu1_port *mxport,
				struct serial_rs485 __user *rs485_user) {
	struct serial_rs485 rs485;
	struct usb_serial_port *port = mxport->mxp_port;
	if (copy_from_user(&rs485, rs485_user, sizeof(*rs485_user)))
		return -EFAULT;

	if (mxport->mxp_uart_types &
	    (MXU1_TYPE_RS422 | MXU1_TYPE_RS485)) {
		
		if (rs485.flags & SER_RS485_ENABLED) {
			mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_ENABLED;
		} else {
			mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_DISABLED;
		}		
	} else {
		dev_err(&port->dev, "%s rs485 not handled by MOXA UPort %04x\n",
			__func__, mxport->mxp_mxdev->mxd_model);
		return  -EINVAL;
	}

	mxu1_set_termios(NULL, mxport->mxp_port, NULL);
	
	return 0;
}

static int mxu1_ioctl(struct tty_struct *tty,
		      unsigned int cmd, unsigned long arg)
{
	struct usb_serial_port *port = tty->driver_data;

	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	
	if (mxport == NULL)
		return -ENODEV;

	switch (cmd) {
	case TIOCGRS485:
		return mxu1_ioctl_get_rs485(mxport,
					    (struct serial_rs485 __user *)
					    arg);
	case TIOCSRS485:
		return mxu1_ioctl_set_rs485(mxport,
					    (struct serial_rs485 __user *)
					    arg);
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
	unsigned long flags;
	
	dev_dbg(&port->dev, "%s - port %d", __func__, port->port_number);

	if (mxport == NULL)
		return -ENODEV;

	spin_lock_irqsave(&mxport->mxp_lock, flags);
	msr = mxport->mxp_msr;
	mcr = mxport->mxp_shadow_mcr;
	spin_unlock_irqrestore(&mxport->mxp_lock, flags);	
	
	result = ((mcr & MXU1_MCR_DTR) ? TIOCM_DTR : 0)
		| ((mcr & MXU1_MCR_RTS) ? TIOCM_RTS : 0)
		| ((mcr & MXU1_MCR_LOOP) ? TIOCM_LOOP : 0)
		| ((msr & MXU1_MSR_CTS) ? TIOCM_CTS : 0)
		| ((msr & MXU1_MSR_CD) ? TIOCM_CAR : 0)
		| ((msr & MXU1_MSR_RI) ? TIOCM_RI : 0)
		| ((msr & MXU1_MSR_DSR) ? TIOCM_DSR : 0);

	dev_dbg(&port->dev, "%s - 0x%04X", __func__, result);

	return result;
}

static int mxu1_tiocmset(struct tty_struct *tty,
			 unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;

	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	unsigned int mcr;
	unsigned long flags;
	
	dev_dbg(&port->dev, "%s - port %d", __func__, port->port_number);

	if (mxport == NULL)
		return -ENODEV;

	spin_lock_irqsave(&mxport->mxp_lock, flags);
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
	
	spin_unlock_irqrestore(&mxport->mxp_lock, flags);
	
	return mxu1_set_mcr(mxport, mcr);
}

static void mxu1_break(struct tty_struct *tty, int break_state)
{
	struct usb_serial_port *port = tty->driver_data;

	struct mxu1_port *mxport = usb_get_serial_port_data(port);

	dev_dbg(&port->dev, "%s - state = %d", __func__, break_state);

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
	__u16 open_settings = (u8)(MXU1_PIPE_MODE_CONTINUOUS |
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

	dev_dbg(&port->dev, "%s - start interrupt in urb", __func__);
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

	mxu1_set_termios(NULL, port, NULL);

	dev_dbg(&port->dev, "%s - sending MXU1_OPEN_PORT", __func__);
	status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_OPEN_PORT,
				    open_settings,
				    (u8)(MXU1_UART1_PORT + port_number));
	if (status) {
		dev_err(&port->dev, "%s - cannot send open command, %d\n",
			__func__,
			status);
		goto unlink_int_urb;
	}

	dev_dbg(&port->dev, "%s - sending MXU1_START_PORT", __func__);
	status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_START_PORT,
				    0, (u8)(MXU1_UART1_PORT + port_number));
	if (status) {
		dev_err(&port->dev, "%s - cannot send start command, %d\n",
			__func__,
			status);
		goto unlink_int_urb;
	}

	dev_dbg(&port->dev, "%s - sending MXU1_PURGE_PORT", __func__);
	status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_PURGE_PORT,
				    MXU1_PURGE_INPUT,
				    (u8)(MXU1_UART1_PORT + port_number));
	if (status) {
		dev_err(&port->dev,
			"%s - cannot clear input buffers, %d\n",
			__func__,
			status);

		goto unlink_int_urb;
	}
	status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_PURGE_PORT,
				    MXU1_PURGE_OUTPUT,
				    (u8)(MXU1_UART1_PORT + port_number));
	if (status) {
		dev_err(&port->dev,
			"%s - cannot clear output buffers, %d\n",
			__func__,
			status);

		goto unlink_int_urb;
	}

	/* reset the data toggle on the bulk endpoints to work around bug in
	 * host controllers where things get out of sync some times
	 */
	usb_clear_halt(dev, port->write_urb->pipe);
	usb_clear_halt(dev, port->read_urb->pipe);

	mxu1_set_termios(NULL, port, NULL);

	dev_dbg(&port->dev, "%s - sending MXU1_OPEN_PORT (2)", __func__);
	status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_OPEN_PORT,
				    open_settings,
				    (u8)(MXU1_UART1_PORT + port_number));
	if (status) {
		dev_err(&port->dev, "%s - cannot send open command, %d\n",
			__func__,
			status);
		goto unlink_int_urb;
	}

	dev_dbg(&port->dev, "%s - sending MXU1_START_PORT (2)", __func__);
	status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_START_PORT,
				    0, (u8)(MXU1_UART1_PORT + port_number));
	if (status) {
		dev_err(&port->dev, "%s - cannot send start command, %d\n",
			__func__,
			status);
		goto unlink_int_urb;
	}

	/* start read urb */
	dev_dbg(&port->dev, "%s - start read urb", __func__);
	urb = port->read_urb;

	if (!urb) {
		dev_err(&port->dev, "%s - no read urb\n", __func__);
		status = -EINVAL;
		goto unlink_int_urb;
	}
	
	urb->context = port;
	urb->dev = dev;
	status = usb_submit_urb(urb, GFP_KERNEL);

	if (status) {
		dev_err(&port->dev, "%s - submit read urb failed, %d\n",
			__func__, status);
		goto unlink_int_urb;
	}

	goto release_mxd_lock;

unlink_int_urb:
	usb_kill_urb(port->serial->port[0]->interrupt_in_urb);

release_mxd_lock:
	mutex_unlock(&mxdev->mxd_lock);
	dev_dbg(&port->dev, "%s - exit %d", __func__, status);

	return status;
}

static void mxu1_close(struct usb_serial_port *port)
{
	struct mxu1_device *mxdev;
	struct mxu1_port *mxport;
	int port_number;
	unsigned long flags;
	int status = 0;

	dev_dbg(&port->dev, "%s - port %d", __func__, port->port_number);

	mxdev = usb_get_serial_data(port->serial);
	mxport = usb_get_serial_port_data(port);
	if (mxdev == NULL || mxport == NULL)
		return;

	usb_kill_urb(port->read_urb);
	usb_kill_urb(port->write_urb);

	spin_lock_irqsave(&port->lock, flags);
	kfifo_reset_out(&port->write_fifo);
	spin_unlock_irqrestore(&port->lock, flags);

	port_number = port->port_number - port->minor;

	dev_dbg(&port->dev, "%s - sending MXU1_CLOSE_PORT", __func__);
	status = mxu1_send_ctrl_urb(port->serial,
				    MXU1_CLOSE_PORT,
				    0, (u8)(MXU1_UART1_PORT + port_number));
	if (status)
		dev_err(&port->dev,
			"%s - cannot send close port command, %d\n",
			__func__,
			status);

	mutex_lock(&mxdev->mxd_lock);
	usb_kill_urb(port->serial->port[0]->interrupt_in_urb);
	mutex_unlock(&mxdev->mxd_lock);

	dev_dbg(&port->dev, "%s - exit", __func__);
}

static void mxu1_handle_new_msr(struct mxu1_port *mxport, u8 msr)
{
	struct async_icount *icount;
	struct tty_struct *tty;
	unsigned long flags;

	dev_dbg(&mxport->mxp_port->dev, "%s - msr 0x%02X", __func__, msr);

	if (msr & MXU1_MSR_DELTA_MASK) {
		spin_lock_irqsave(&mxport->mxp_lock, flags);
		icount = &mxport->mxp_port->icount;		
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
	u8 msr;

	dev_dbg(&urb->dev->dev, "%s", __func__);

	/* Check port is valid or not */
	if (mxdev == NULL)
		return;


	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		dev_dbg(dev, "%s - urb shutting down, %d", __func__, urb->status);
		return;
	default:
		dev_err(dev, "%s - nonzero urb status, %d\n",
			__func__, urb->status);
		goto exit;
	}

	if (length != 2) {
		dev_dbg(dev, "%s - bad packet size, %d", __func__, length);
		goto exit;
	}

	if (data[0] == MXU1_CODE_HARDWARE_ERROR) {
		dev_err(dev, "%s - hardware error, %d\n", __func__, data[1]);
		goto exit;
	}

	port_number = MXU1_GET_PORT_FROM_CODE(data[0]);
	function = MXU1_GET_FUNC_FROM_CODE(data[0]);

	dev_dbg(dev, "%s - port_number %d, function %d, data 0x%02X",
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
		dev_dbg(dev, "%s - DATA ERROR, port %d, data 0x%02X\n",
			 __func__,
			 port_number,
			 data[1]);
		break;

	case MXU1_CODE_MODEM_STATUS:
		msr = data[1];
		dev_dbg(dev, "%s - port %d, msr 0x%02X",
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
	.set_termios		= mxu1_set_termios,
	.tiocmget		= mxu1_tiocmget,
	.tiocmset		= mxu1_tiocmset,
	.get_icount		= usb_serial_generic_get_icount,
	.break_ctl		= mxu1_break,
	.read_int_callback	= mxu1_interrupt_callback,
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
