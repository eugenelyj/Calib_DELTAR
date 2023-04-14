/**************************************************************************
 * Copyright (c) 2016, STMicroelectronics - All Rights Reserved

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/atomic.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include "stmvl53l5cx_i2c.h"


/*
 * GPIO number connected to L5 interrupt signal GPIO1
 */
#define IRQ_GPIO -1

static int intr_gpio_nb = IRQ_GPIO;

module_param(intr_gpio_nb, int, 0000);
MODULE_PARM_DESC(intr_gpio_nb, "select gpio number to use for vl53l5 interrupt");

#define STMVL53L5_DRV_NAME		"stmvl53l5cx"
#define STMVL53L5_SLAVE_ADDR		0x29

#define ST_TOF_IOCTL_TRANSFER 		_IOWR('a',0x1, void*)
#define ST_TOF_IOCTL_WAIT_FOR_INTERRUPT	_IO('a',0x2)

struct stmvl53l5cx_comms_struct {
	__u16   len;
	__u16   reg_index;
	__u8    *buf;
	__u8    write_not_read;
};

struct gpio_own_flags_t {
	unsigned intr_gpio_owned:1; /*!< set if intr gpio is owned*/
} gpio_own_flags;

static struct miscdevice st_tof_miscdev;
static uint8_t * raw_data_buffer = NULL;

static uint8_t i2c_driver_added = 0;
static uint8_t misc_registered = 0;

static uint8_t irq_handler_registered = 0;

static wait_queue_head_t wq;

static atomic_t intr_ready_flag;

static atomic_t force_wakeup;

static unsigned int st_tof_irq_num;
static const struct i2c_device_id stmvl53l5cx_i2c_id[] = {
	{STMVL53L5_DRV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, stmvl53l5cx_i2c_id);

static const struct of_device_id st_tof_of_match[] = {
	{
		/* An older compatible */
		.compatible = "st,stmvl53l5cx",
		.data = STMVL53L5_DRV_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, st_tof_of_match);  // add to the kernel device tree table

static struct i2c_client *stmvl53l5cx_i2c_client = NULL;

static int get_gpio(int gpio_number, const char *name, int direction)
{
	int rc = 0;

	if (gpio_number == -1) {
		pr_err("%s(%d) : gpio is required\n", __func__, __LINE__);
		rc = -ENODEV;
		goto no_gpio;
	}

	pr_debug("request gpio_number %d", gpio_number);
	rc = gpio_request(gpio_number, name);
	if (rc) {
		pr_err("fail to acquire gpio(%d), error = %d", gpio_number, rc);
		goto request_failed;
	}

	if (direction == 1)
	{
		rc = gpio_direction_output(gpio_number, 0);
		if (rc) {
			pr_err("fail to configure gpio_number(%d) as output %d", gpio_number, rc);
			goto direction_failed;
		}
	}
	else
	{
		rc = gpio_direction_input(gpio_number);
		if (rc) {
			pr_err("fail to configure gpio_number(%d) as input %d", gpio_number, rc);
			goto direction_failed;
		}
	}


	return rc;

direction_failed:
	gpio_free(gpio_number);

request_failed:
no_gpio:
	return rc;
}

static void put_gpio(int gpio_number)
{
	pr_debug("release gpio_number %d", gpio_number);
	gpio_free(gpio_number);
}

static int stmvl53l5cx_open(struct inode *inode, struct file *file)
{
	pr_debug("stmvl53l5cx : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static int stmvl53l5cx_release(struct inode *inode, struct file *file)
{
	pr_debug("stmvl53l5cx : %s(%d)\n", __func__, __LINE__);
	return 0;
}

/* Interrupt handler */
static irqreturn_t st_tof_intr_handler(int st_tof_irq_num, void *dev_id)
{
	/* Update interrupt flag */
	atomic_set(&intr_ready_flag, 1);

	/* Unblock the IOCTL call to return to user space */
	wake_up_interruptible(&wq);

	return IRQ_HANDLED;
}

static long stmvl53l5cx_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct i2c_msg st_i2c_message;
	struct stmvl53l5cx_comms_struct comms_struct = {0};
	int32_t ret = 0;
	uint16_t index, transfer_size, chunk_size;
	void __user *data_ptr = NULL;

	pr_debug("stmvl53l5cx_ioctl : cmd = %u\n", cmd);
	switch (cmd) {
		case ST_TOF_IOCTL_WAIT_FOR_INTERRUPT:
			pr_debug("%s(%d)\n", __func__, __LINE__);
			atomic_set(&force_wakeup, 0);
			ret = wait_event_interruptible(wq, atomic_read(&intr_ready_flag) != 0);
			if (ret || atomic_read(&force_wakeup)) {
				printk("wait_event_interruptible ret = %d, force_wakeup flag = %d\n", ret, atomic_read(&force_wakeup));
				atomic_set(&intr_ready_flag, 0);
				atomic_set(&force_wakeup, 0);
				return -EINTR;
			}
			atomic_set(&intr_ready_flag, 0);
			break;
		case ST_TOF_IOCTL_TRANSFER:

			ret = copy_from_user(&comms_struct, (void __user *)arg, sizeof(comms_struct));
			if (ret) {
				pr_err("Error at %s(%d)\n", __func__, __LINE__);
				return -EINVAL;
			}

			// printk("Transfer. write_not_read = %d, reg_index = 0x%x size = %d\n", comms_struct.write_not_read, comms_struct.reg_index, comms_struct.len);
			// address and buis the same whatever the transfers to be done !
			st_i2c_message.addr = 0x29;
			// st_i2c_message.buf is the same whatever the transfers to be done
			st_i2c_message.buf = raw_data_buffer;

			if (!comms_struct.write_not_read) {
				data_ptr = (u8 __user *)(comms_struct.buf);
				comms_struct.buf  = memdup_user(data_ptr,  comms_struct.len);
			}

			// in case of i2c write, it is a single transfer with read index set in the 2 first bytes
			// the other case use fully the raw data buffer for raw data transfers
			if (comms_struct.write_not_read)
				chunk_size = VL53L5CX_COMMS_CHUNK_SIZE - 2;
			else
				chunk_size = VL53L5CX_COMMS_CHUNK_SIZE;

			// index is the number of bytes already transfered
			index = 0;

			do {
				// take the max number of bytes that can be transfered
				transfer_size = (comms_struct.len - index) > chunk_size ?  chunk_size : (comms_struct.len - index);

				// ----- WRITE
				if (comms_struct.write_not_read) {
					// put red index at the beginning of the buffer
					raw_data_buffer[0] = (uint8_t)(((comms_struct.reg_index + index) & 0xFF00) >> 8);
					raw_data_buffer[1] = (uint8_t)((comms_struct.reg_index + index) & 0x00FF);

					ret = copy_from_user(&raw_data_buffer[2], comms_struct.buf + index, transfer_size);
					if (ret) {
						pr_err("Error at %s(%d)\n", __func__, __LINE__);
						return -EINVAL;
					}

					st_i2c_message.len = transfer_size + 2;
					st_i2c_message.flags = 0;
					ret = i2c_transfer(stmvl53l5cx_i2c_client->adapter, &st_i2c_message, 1);
					if (ret != 1) {
						pr_err("Error %d at %s(%d)\n",ret,  __func__, __LINE__);
						return -EIO;
					}
				}
				// ----- READ
				else {
					// write reg_index
					st_i2c_message.len = 2;
					st_i2c_message.flags = 0;
					raw_data_buffer[0] = (uint8_t)(((comms_struct.reg_index + index) & 0xFF00) >> 8);
					raw_data_buffer[1] = (uint8_t)((comms_struct.reg_index + index) & 0x00FF);

					ret = i2c_transfer(stmvl53l5cx_i2c_client->adapter, &st_i2c_message, 1);
					if (ret != 1) {
						pr_err("Error at %s(%d)\n", __func__, __LINE__);
						return -EIO;
					}

					st_i2c_message.len = transfer_size;
					st_i2c_message.flags = 1;

					ret = i2c_transfer(stmvl53l5cx_i2c_client->adapter, &st_i2c_message, 1);
					if (ret != 1) {
						pr_err("Error at %s(%d)\n", __func__, __LINE__);
						return -EIO;
					}

					// copy to user buffer the read transfer
					ret = copy_to_user(data_ptr + index, raw_data_buffer, transfer_size);

					if (ret) {
						pr_err("Error at %s(%d)\n", __func__, __LINE__);
						return -EINVAL;
					}

				} // ----- READ

				index += transfer_size;

			} while (index < comms_struct.len);
			break;

		default:
			return -EINVAL;

	}
	return 0;
}

static const struct file_operations stmvl53l5cx_ranging_fops = {
	.owner 			= THIS_MODULE,
	.unlocked_ioctl		= stmvl53l5cx_ioctl,
	.open 			= stmvl53l5cx_open,
	.release 		= stmvl53l5cx_release,
};


static int stmvl53l5cx_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;
	uint8_t page = 0, revision_id = 0, device_id = 0;
	stmvl53l5cx_i2c_client = client;

	printk("stmvl53l5cx: probing i2c\n");

	raw_data_buffer = kzalloc(VL53L5CX_COMMS_CHUNK_SIZE, GFP_DMA | GFP_KERNEL);
	if (raw_data_buffer == NULL)
		 return -ENOMEM;

	ret = stmvl53l5cx_write_multi(client, raw_data_buffer, 0x7FFF, &page, 1);
	ret |= stmvl53l5cx_read_multi(client, raw_data_buffer, 0x00, &device_id, 1);
	ret |= stmvl53l5cx_read_multi(client, raw_data_buffer, 0x01, &revision_id, 1);

	if ((device_id != 0xF0) || (revision_id != 0x02)) {
		pr_err("stmvl53l5cx: Error. Could not read device and revision id registers\n");
		return ret;
	}
	printk("stmvl53l5cx: device_id : 0x%x. revision_id : 0x%x\n", device_id, revision_id);

	st_tof_miscdev.minor = MISC_DYNAMIC_MINOR;
	st_tof_miscdev.name = "stmvl53l5cx";
	st_tof_miscdev.fops = &stmvl53l5cx_ranging_fops;
	st_tof_miscdev.mode = 0444;

	ret = misc_register(&st_tof_miscdev);
	if (ret) {
		pr_err("stmvl53l5cx : Failed to create misc device, err = %d\n", ret);
		return -1;
	}

	misc_registered = 1;

	if (intr_gpio_nb != -1)
	{

		ret = get_gpio(intr_gpio_nb, "vl53l5_intr_gpio", 0);
		if (ret != 0) {
			pr_err("Failed to acquire INTR GPIO(%d)\n", intr_gpio_nb);
		} else {
			gpio_own_flags.intr_gpio_owned = 1;

			st_tof_irq_num = gpio_to_irq(intr_gpio_nb);

			init_waitqueue_head(&wq);

			ret = request_threaded_irq(st_tof_irq_num,
						NULL,
						st_tof_intr_handler,
						IRQF_TRIGGER_RISING|IRQF_ONESHOT,
						"st_tof_sensor",
						NULL);

			if (ret) {
				pr_err("stmvl53l5cx: Failed to Register IRQ handler,"
				       " GPIO = %d, st_tof_irq_num = %d\n",
				       IRQ_GPIO, st_tof_irq_num);
				kfree(raw_data_buffer);
				misc_deregister(&st_tof_miscdev);
				misc_registered = 0;
				return -EPERM;
			} else {
				printk("stmvl53l5cx: Registered IRQ handler, GPIO = %d, "
				       "st_tof_irq_num = %d\n",
				       IRQ_GPIO, st_tof_irq_num);
				irq_handler_registered = 1;
			}
		}
	}



	return ret;
}

static int stmvl53l5cx_remove(struct i2c_client *client)
{

	if (raw_data_buffer)
		kfree(raw_data_buffer);
	return 0;
}

static struct i2c_driver stmvl53l5cx_i2c_driver = {
	.driver = {
		.name = STMVL53L5_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(st_tof_of_match), // for platform register to pick up the dts info
	},
	.probe = stmvl53l5cx_probe,
	.remove = stmvl53l5cx_remove,
	.id_table = stmvl53l5cx_i2c_id,
};


static int __init st_tof_module_init(void)
{
	int ret = 0;

	printk("stmvl53l5cx: module init\n");

	/* register as a i2c client device */
	ret = i2c_add_driver(&stmvl53l5cx_i2c_driver);

	if (ret) {
		i2c_del_driver(&stmvl53l5cx_i2c_driver);
		printk("stmvl53l5cx: could not add i2c driver\n");
		return ret;
	}

	i2c_driver_added = 1;

	return ret;
}

static void __exit st_tof_module_exit(void)
{

	pr_debug("stmvl53l5cx : module exit\n");

	if (misc_registered) {
		misc_deregister(&st_tof_miscdev);
		misc_registered = 0;
	}

	if (i2c_driver_added) {
		i2c_del_driver(&stmvl53l5cx_i2c_driver);
		i2c_driver_added = 0;
	}

	if (irq_handler_registered) {
		free_irq(st_tof_irq_num, NULL);
	}

	if (gpio_own_flags.intr_gpio_owned == 1) {
		put_gpio(intr_gpio_nb);
	}
}

module_init(st_tof_module_init);
module_exit(st_tof_module_exit);
MODULE_LICENSE("GPL");
