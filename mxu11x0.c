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

#include "mxu11x0r.h"

/* read urb states */
#define MXU1_READ_URB_RUNNING	0
#define MXU1_READ_URB_STOPPING	1
#define MXU1_READ_URB_STOPPED	2


/* Structures */

struct mxu1_port {
    int			mxp_is_open;
    __u8			mxp_msr;
    __u8			mxp_lsr;
    __u8			mxp_shadow_mcr;
    __u8			mxp_uart_mode;	/* 232 or 485 modes */
    __u8			mxp_user_get_uart_mode;
    unsigned int		mxp_uart_base_addr;
    int			mxp_flags;
    int			mxp_msr_wait_flags;
    wait_queue_head_t	mxp_msr_wait;	/* wait for msr change */
    struct mxu1_device	*mxp_mxdev;
    struct usb_serial_port	*mxp_port;
    spinlock_t		mxp_lock;
    int			mxp_read_urb_state;
    int			mxp_send_break;
    int			mxp_set_B0;
};

struct mxu1_device {
        struct mutex mxd_lock;
    int			mxd_open_port_count;
    struct usb_serial	*mxd_serial;
    int			mxd_model_name;
};

/* read urb states */
#define MXU1_READ_URB_RUNNING	0
#define MXU1_READ_URB_STOPPING	1
#define MXU1_READ_URB_STOPPED	2

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
	__u8 cs = 0;
	__u8 *buffer;	
	struct usb_device *dev = serial->dev;
	struct mxu1_firmware_header *header;	
	unsigned int pipe = usb_sndbulkpipe(dev, serial->port[0]->
					    bulk_out_endpointAddress);
	
	buffer_size = MXU1_FIRMWARE_BUF_SIZE +
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

	pr_info("%s - downloading firmware", __func__);

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
				    	
	pr_info("%s - download successful (%d)", __func__, status);
	
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
    
    port->port.closing_wait = msecs_to_jiffies(10 * 10);
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

	pr_info("%s - product 0x%4X, num configurations %d, configuration value %d",
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
	    
	    snprintf(fw_name, sizeof(fw_name) - 1, "moxa/moxa-%04x.fw", le16_to_cpu(product_id));

	    err = request_firmware(&fw_p, fw_name, &serial->interface->dev);
	    if (err) {
		dev_err(&serial->interface->dev, "Firmware %s not found\n",
			fw_name);
		kfree(mxdev);
		return err;    
	    } else {
		err = mxu1_download_firmware(serial, fw_p);
		if (err) {
		    kfree(mxdev);
		    return err;
		}
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

	pr_info("%s - addr 0x%08lX, mask 0x%02X, byte 0x%02X", __func__,
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

	pr_info("%s - port %d", __func__, port->port_number);

	if (!tty) {
		pr_info("%s - no tty or termios", __func__);
		return;
	}

	cflag = tty->termios.c_cflag;
	iflag = tty->termios.c_iflag;

	if (old_termios && cflag == old_termios->c_cflag
	    && iflag == old_termios->c_iflag) {
		pr_info("%s - nothing to change", __func__);
		return;
	}

	pr_info("%s - clfag %08x, iflag %08x", __func__, cflag, iflag);

	if (old_termios)
		pr_info("%s - old clfag %08x, old iflag %08x",
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
		//mxu1_restart_read(mxport, tty);
	}

	if (I_IXOFF(tty) || I_IXON(tty)) {
		config->cXon  = START_CHAR(tty);
		config->cXoff = STOP_CHAR(tty);

		if (I_IXOFF(tty))
			config->wFlags |= MXU1_UART_ENABLE_X_IN;
		else
		     //mxu1_restart_read(mxport, tty);

		if (I_IXON(tty))
			config->wFlags |= MXU1_UART_ENABLE_X_OUT;
	}

	baud = tty_get_baud_rate(tty);
	if (!baud)
		baud = 9600;
	config->wBaudRate = (__u16)(923077 / baud);

	pr_info("%s - BaudRate=%d, wBaudRate=%d, wFlags=0x%04X, bDataBits=%d, bParity=%d, bStopBits=%d, cXon=%d, cXoff=%d, bUartMode=%d",
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
    
    /* /\* start interrupt urb the first time a port is opened on this device *\/ */
    /* if (mxdev->mxd_open_port_count == 0) { */
    /* 	pr_info("%s - start interrupt in urb", __func__); */
    /* 	urb = mxdev->mxd_serial->port[0]->interrupt_in_urb; */
    /* 	if (!urb) { */
    /* 	    dev_err(&port->dev, */
    /* 		    "%s - no interrupt urb\n", */
    /* 		    __func__); */
    /* 	    status = -EINVAL; */
    /* 	    goto release_mxd_lock; */
    /* 	} */
    /* 	urb->context = mxdev; */
    /* 	status = usb_submit_urb(urb, GFP_KERNEL); */
    /* 	if (status) { */
    /* 	    dev_err(&port->dev, */
    /* 		    "%s - submit interrupt urb failed, %d\n", */
    /* 		    __func__, */
    /* 		    status); */
    /* 	    goto release_mxd_lock; */
    /* 	} */
    /* } */

    mxu1_set_termios(NULL, port, NULL);

    pr_info("%s - sending MXU1_OPEN_PORT", __func__);
    status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_OPEN_PORT,
				open_settings, (__u8)(MXU1_UART1_PORT + port_number));
    if (status) {
	dev_err(&port->dev, "%s - cannot send open command, %d\n",
		__func__,
		status);
	goto unlink_int_urb;
    }

    pr_info("%s - sending MXU1_START_PORT", __func__);
    status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_START_PORT,
				0, (__u8)(MXU1_UART1_PORT + port_number));
    if (status) {
	dev_err(&port->dev, "%s - cannot send start command, %d\n",
		__func__,
		status);
	goto unlink_int_urb;
    }

    pr_info("%s - sending MXU1_PURGE_PORT", __func__);
    status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_PURGE_PORT,
				MXU1_PURGE_INPUT, (__u8)(MXU1_UART1_PORT + port_number));
    if (status) {
	dev_err(&port->dev,
		"%s - cannot clear input buffers, %d\n",
		__func__,
		status);

	goto unlink_int_urb;
    }
    status = mxu1_send_ctrl_urb(mxdev->mxd_serial, MXU1_PURGE_PORT,
				MXU1_PURGE_OUTPUT, (__u8)(MXU1_UART1_PORT + port_number));
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

    /* start read urb */
    pr_info("%s - start read urb", __func__);
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
    pr_info("%s - exit %d", __func__, status);
    
    return status;
}

static void mxu1_close(struct usb_serial_port *port)
{
    struct mxu1_device *mxdev;
    struct mxu1_port *mxport;
    int port_number;
    int status = 0;

    pr_info("%s - port %d", __func__, port->port_number);

    mxdev = usb_get_serial_data(port->serial);
    mxport = usb_get_serial_port_data(port);
    if (mxdev == NULL || mxport == NULL)
	return;

    mxport->mxp_is_open = 0;

    pr_info("reste %d chars", usb_serial_generic_chars_in_buffer(port->port.tty));
	
    usb_kill_urb(port->read_urb);
    usb_kill_urb(port->write_urb);

    port_number = port->port_number - port->minor;

    pr_info("%s - sending MXU1_CLOSE_PORT", __func__);
    status = mxu1_send_ctrl_urb(port->serial, MXU1_CLOSE_PORT, 0, (__u8)(MXU1_UART1_PORT + port_number));				
    if (status)
	dev_err(&port->dev,
		"%s - cannot send close port command, %d\n",
		__func__,
		status);

    mutex_lock_interruptible(&mxdev->mxd_lock);
    --mxport->mxp_mxdev->mxd_open_port_count;
    if (mxport->mxp_mxdev->mxd_open_port_count <= 0) {
	//usb_kill_urb(port->serial->port[0]->interrupt_in_urb);

	mxport->mxp_mxdev->mxd_open_port_count = 0;
    }
    mutex_unlock(&mxdev->mxd_lock);
    
    pr_info("%s - exit", __func__);
}

static struct usb_serial_driver mxuport11_device = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "mxuport11",
	},
	.description		= "MOXA UPort 11xx",
	.id_table		= mxuport11_idtable,
	.num_ports		= 1,
	.port_probe             = mxu1_port_probe,
	.attach			= mxu1_startup,
	.open			= mxu1_open,
	.close			= mxu1_close
};

static struct usb_serial_driver *const serial_drivers[] = {
    &mxuport11_device, NULL
};

module_usb_serial_driver(serial_drivers, mxuport11_idtable);

MODULE_AUTHOR("Ken Huang");
MODULE_AUTHOR("Mathieu Othacehe <m.othacehe@gmail.com>");
MODULE_DESCRIPTION("MOXA UPort 11xx USB to Serial Hub Driver");
MODULE_LICENSE("GPL");
