/*
 * gpio-ts.c
 *
 * Author:
 *   Alex Konshin <akonshin@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
//#include <linux/gpio/consumer.h> // it is not available in BPi-M3 kernel 3.4.39
#include <linux/gpio.h>
#include <linux/interrupt.h>
//#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/delay.h>  // msleep
//#include <linux/atomic.h>

#include <asm/div64.h>

#include "gpio-ts.h"

#define GPIO_TS_VERSION_MAJOR 1
#define GPIO_TS_VERSION_MINOR 0

//------------------- Module parameters -------------------------------------

int gpios[MAX_GPIO_ENTRIES];
int n_of_gpios;
int min_duration = DEFAULT_MIN_DURATION;
int max_duration = DEFAULT_MAX_DURATION;
int min_seq_len = DEFAULT_MIN_SEQ_LEN;

module_param_array(gpios, int, &n_of_gpios, 0644);
module_param(min_duration,int,0444);
module_param(max_duration,int,0444);
module_param(min_seq_len,int,0444);
MODULE_PARM_DESC(gpios, "GPIOs devices to be created.");
MODULE_PARM_DESC(min_duration, "Minimum duration of the same level to be accepted.");
MODULE_PARM_DESC(min_duration, "Maximum duration of the same level to be accepted.");
MODULE_PARM_DESC(min_duration, "Minimum sequence length to be accepted.");

// ------------------ Driver private methods -------------------------------

static irqreturn_t gpio_ts_isr(int irq, void *pfile);
static int gpio_ts_open(struct inode *inode, struct file *pfile);
static int gpio_ts_release(struct inode *inode, struct file *pfile);
static unsigned int gpio_ts_poll(struct file* pfile, poll_table* wait);
static ssize_t gpio_ts_read(struct file *pfile, char *buf, size_t count, loff_t *pos);
static ssize_t gpio_ts_read(struct file *pfile, char *buf, size_t count, loff_t *pos);
static long gpio_ts_ioctl(struct file *f, unsigned int cmd, unsigned long arg);
static void gpio_ts_irq_tasklet_function(unsigned long slot_index);

// ------------------ Driver private global data ----------------------------

static struct file_operations gpio_ts_fops = {
    .owner   = THIS_MODULE,
    .open    = gpio_ts_open,
    .release = gpio_ts_release,
    .read    = gpio_ts_read,
    .poll    = gpio_ts_poll,
    .unlocked_ioctl = gpio_ts_ioctl
};

static dev_t          gpio_ts_dev;
static struct cdev    gpio_ts_cdev;
static struct class*  gpio_ts_class = NULL;

static gpio_ts_statistics_t statistics;

static gpio_ts_tasklet_data_t tasklets[MAX_GPIO_ENTRIES];

static void init_irq_tasklet_data_slots(void) {
    int gpio_index;

    for (gpio_index = 0; gpio_index < MAX_GPIO_ENTRIES; gpio_index++) {
        gpio_ts_tasklet_data_t* t = &tasklets[gpio_index];
        t->write_index = 0;
        t->read_index = 0;
        tasklet_init(&t->tasklet, gpio_ts_irq_tasklet_function, (unsigned long)gpio_index);
    }
}

// It is required for automatic setting of permissions to created devices
static char* gpio_ts_devnode(struct device* dev, umode_t* mode) {
    if (mode != NULL) *mode = 0666;
    return NULL;
}

static void enable(struct gpio_ts_data* ts_data) {
    if (!ts_data->enabled) {
        ts_data->enabled = true;
        printk(KERN_INFO "%s: Started receiving data from GPIO %d. Parameters: min_duration=%d max_duration=%d min_seq_len=%d\n",
            THIS_MODULE->name, ts_data->gpio, ts_data->min_duration, ts_data->max_duration, ts_data->min_seq_len);
    }
}

// ------------------ Driver initialization --------------------------

static int __init gpio_ts_init(void) {
    int err;
    int i;

    printk(KERN_INFO "%s: GPIO TS driver version %d.%d.\n", THIS_MODULE->name, GPIO_TS_VERSION_MAJOR, GPIO_TS_VERSION_MINOR);
    if (n_of_gpios < 1) { // TODO default table
        printk(KERN_ERR "%s: At least one GPIO input is required\n", THIS_MODULE->name);
        return -EINVAL;
    }

    err = alloc_chrdev_region(&gpio_ts_dev, 0, n_of_gpios, THIS_MODULE->name);
    if (err != 0) return err;

    gpio_ts_class = class_create(THIS_MODULE, GPIO_TS_CLASS_NAME);
    if (IS_ERR(gpio_ts_class)) {
        unregister_chrdev_region(gpio_ts_dev, n_of_gpios);
        return -EINVAL;
    }
    gpio_ts_class->devnode = gpio_ts_devnode;

    for (i = 0; i < n_of_gpios; i++)
        device_create(gpio_ts_class, NULL, MKDEV(MAJOR(gpio_ts_dev), i), NULL, GPIO_TS_ENTRIES_NAME, gpios[i]);

    cdev_init(&gpio_ts_cdev, &gpio_ts_fops);

    err = cdev_add(&gpio_ts_cdev, gpio_ts_dev, n_of_gpios);
    if (err != 0) {
        for (i = 0; i < n_of_gpios; i++)
            device_destroy(gpio_ts_class, MKDEV(MAJOR(gpio_ts_dev), i));
        class_destroy(gpio_ts_class);
        unregister_chrdev_region(gpio_ts_dev, n_of_gpios);
        return err;
    }
    init_irq_tasklet_data_slots();
    memset(&statistics, 0, sizeof(gpio_ts_statistics_t));

    printk(KERN_INFO "%s: The module has been initialized.\n", THIS_MODULE->name);
    return 0;
}

// ------------------ Driver removing --------------------------
void __exit gpio_ts_exit (void) {
    int i;

    printk(KERN_INFO "%s: Unloading module...\n", THIS_MODULE->name);

    cdev_del(&gpio_ts_cdev);

    for (i = 0; i < n_of_gpios; i ++)
        device_destroy(gpio_ts_class, MKDEV(MAJOR(gpio_ts_dev), i));

    class_destroy(gpio_ts_class);
    gpio_ts_class = NULL;

    unregister_chrdev_region(gpio_ts_dev, n_of_gpios);

    printk(KERN_INFO "%s: The module has been unloaded.\n", THIS_MODULE->name);
}

// ------------------ IOCTL --------------------------------------

static long gpio_ts_ioctl(struct file* pfile, unsigned int cmd, unsigned long arg) {
    struct gpio_ts_data* ts_data = pfile->private_data;

    if (ts_data == NULL) return -EBADF;

    switch (cmd) {
    case GPIOTS_IOCTL_START:
        enable(ts_data);
        break;

    case GPIOTS_IOCTL_SUSPEND:
        if (ts_data->enabled) {
            ts_data->enabled = false;
            printk(KERN_INFO "%s: Suspended receiving data from GPIO %d.\n", THIS_MODULE->name, ts_data->gpio);
        }
        break;

    case GPIOTS_IOCTL_SET_MIN_DURATION:
        if (ts_data->enabled) {
            printk(KERN_WARNING "%s: Changing parameters is allowed before read/poll/select call.\n", THIS_MODULE->name);
            return -EBUSY;
        }
        if (arg<0 || arg>GPIO_TS_MAX_DURATION || arg>=ts_data->max_duration) {
            printk(KERN_WARNING "%s: Invalid argument %ld of command GPIOTS_IOCTL_SET_MIN_DURATION\n", THIS_MODULE->name, arg);
            return -EINVAL;
        }
        ts_data->min_duration = (uint32_t)arg;
        break;

    case GPIOTS_IOCTL_SET_MAX_DURATION:
        if (ts_data->enabled) {
            printk(KERN_WARNING "%s: Changing parameters is allowed before read/poll/select call.\n", THIS_MODULE->name);
            return -EBUSY;
        }
        if (arg<0 || arg>GPIO_TS_MAX_DURATION || arg<=ts_data->min_duration) {
            printk(KERN_WARNING "%s: Invalid argument %ld of command GPIOTS_IOCTL_SET_MAX_DURATION\n", THIS_MODULE->name, arg);
            return -EINVAL;
        }
        ts_data->max_duration = (uint32_t)arg;
        break;

    case GPIOTS_IOCTL_SET_MIN_SEQ_LEN:
        if (ts_data->enabled) {
            printk(KERN_WARNING "%s: Changing parameters is allowed before read/poll/select call.\n", THIS_MODULE->name);
            return -EBUSY;
        }
        if (arg<0 || arg>=(N_BUFFER_ITEMS/2)) {
            printk(KERN_WARNING "%s: Invalid argument %ld of command GPIOTS_IOCTL_SET_MIN_SEQ_LEN\n", THIS_MODULE->name, arg);
            return -EINVAL;
        }
        ts_data->min_seq_len = (unsigned)arg;
        break;

    default:
        printk(KERN_WARNING "%s: Invalid IOCTL command %08x\n", THIS_MODULE->name, cmd);
        return -EINVAL;
    }
    return 0;
}

// ------------------ Open file -----------------------------------

static void ts_data_reset(unsigned gpio, unsigned gpio_index, struct gpio_ts_data* ts_data) {
    ktime_t kt;

    kt = ktime_get();
    ts_data->last_timestamp = ts_data->current_seq_timestamp = ts_data->last_recorded_timestamp = ktime_to_ns(kt);
    ts_data->gpio = gpio;
    ts_data->gpio_index = gpio_index;
    ts_data->last_status = ts_data->delayed_status = STATUS_NOISE;
    ts_data->read_index = 0;
    ts_data->write_index = 0;
    ts_data->current_seq_index = 0;
    ts_data->current_seq_len = 0;
    ts_data->buffer_seq_len = 0;
    ts_data->min_duration = min_duration;
    ts_data->max_duration = max_duration;
    ts_data->min_seq_len = min_seq_len;
    ts_data->signal = false;
    ts_data->enabled = false; // input disabled until the first read() or poll() calls
}

static int gpio_ts_open(struct inode* inode, struct file* pfile) {
    struct gpio_ts_data* ts_data;
    int err;
    unsigned gpio_index;
    unsigned gpio;
    int irq;

    //struct gpio_chip* chip;

    gpio_index = iminor(inode);
    gpio = (unsigned)gpios[gpio_index];

    err = gpio_request(gpio, THIS_MODULE->name);
    if (err != 0) {
        printk(KERN_ERR "%s: unable to reserve GPIO %d\n", THIS_MODULE->name, gpio);
        return err;
    }

    err = gpio_direction_input(gpio);
    if (err != 0) {
        printk(KERN_ERR "%s: unable to set GPIO %d as input\n", THIS_MODULE->name, gpio);
        gpio_free(gpio);
        return err;
    }

    irq = gpio_to_irq(gpio); // FIXME it does not work for BPi-M3
    /*
      BPI-M3 linux/drivers/pinctrl/pinctrl-sunxi.c

    chip = gpio_to_chip(gpio);
    if (chip == NULL) {
        printk(KERN_ERR "%s: Invalid GPIO %d - could not get chip\n", THIS_MODULE->name, gpio);
        gpio_free(gpio);
        return -ENXIO;
    }
    if (chip->to_irq == NULL) {
        printk(KERN_ERR "%s: Invalid GPIO %d - function to_irq is not set\n", THIS_MODULE->name, gpio);
        gpio_free(gpio);
        return -ENXIO;
    }
    irq = chip->to_irq(chip, gpio - chip->base);
    if (irq < 0) {
        printk(KERN_ERR "%s: Cannot get IRQ number for GPIO %d (error %d)\n", THIS_MODULE->name, gpio, -irq);
        gpio_free(gpio);
        return irq;
    }
    */

    err = request_irq(irq, gpio_ts_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, THIS_MODULE->name, pfile);
    if (err != 0) {
        printk(KERN_ERR "%s: unable to handle GPIO %d IRQ (error %d).\n", THIS_MODULE->name, gpio, -err);
        gpio_free(gpio);
        return err;
    }

    ts_data = kzalloc(sizeof(struct gpio_ts_data), GFP_KERNEL);
    if (ts_data == NULL) return -ENOMEM;

    spin_lock_init(&ts_data->spinlock);
    init_waitqueue_head(&ts_data->read_queue);
    ts_data_reset(gpio, gpio_index, ts_data);

    pfile->private_data = ts_data;
    printk(KERN_INFO "%s: Opened GPIO %d (IRQ %d)\n", THIS_MODULE->name, gpio, irq);
    return 0;
}

// ------------------ Close file -----------------------------------

static int gpio_ts_release(struct inode* inode,  struct file* pfile) {
    gpio_ts_tasklet_data_t* tasklet_data;
    struct gpio_ts_data* ts_data;
    unsigned gpio_index = iminor(inode);
    unsigned gpio = (unsigned)gpios[gpio_index];

    if ((ts_data = pfile->private_data) == NULL) return 0;

    printk(KERN_INFO "%s: Closing GPIO %d...\n", THIS_MODULE->name, gpio);
    pfile->private_data = NULL;
    ts_data->enabled = false;

    free_irq(gpio_to_irq(gpio), pfile);
    gpio_free(gpio);

    tasklet_data = &tasklets[ts_data->gpio_index];
    tasklet_data->read_index = 0;
    tasklet_data->write_index = 0;

    tasklet_kill(&tasklet_data->tasklet);

    if ((pfile->f_flags & O_NONBLOCK) == 0) {
        // unblock waiting processes
        wake_up_interruptible(&ts_data->read_queue);
    }

    // short sleep to give a chance to user process to finish with this file
    if (waitqueue_active(&ts_data->read_queue)) {
        msleep(200);
    } else {
        msleep(100);
    }

    kfree(ts_data);
    return 0;
}

// ------------------ Interruption handler -------------------------
/**
 * Actual processing of this event is done later during execution of tasklet.
 */
static irqreturn_t gpio_ts_isr(int irq, void* filedata) {
    struct file* pfile;
    struct gpio_ts_data* ts_data;
    struct gpio_ts_irq_data* irq_data;
    gpio_ts_tasklet_data_t* tasklet_data;
    ktime_t kt;

    unsigned irq_data_index;
    unsigned irq_data_next_index;
    unsigned gpio;
    unsigned gpio_level;

    statistics.isr_counter++; // this counter is not critical and no needs to make it atomic

    if (filedata == NULL) return IRQ_NONE;

    pfile = (struct file*)filedata;
    ts_data = pfile->private_data;
    if (ts_data == NULL) return IRQ_NONE;
    if (!ts_data->enabled) return IRQ_HANDLED;

    tasklet_data = &tasklets[ts_data->gpio_index];
    irq_data_index = tasklet_data->write_index;
    irq_data_next_index = (irq_data_index+1)&(N_IRQ_DATA_SLOTS-1);
    if (tasklet_data->read_index == irq_data_next_index) {
      // ignore this interruption due to irq_data buffer overflow
      statistics.irq_data_overflow_counter++; // this counter is not critical and no needs to make it atomic

      if (tasklet_data->tasklet.state == 0)
        tasklet_schedule(&tasklet_data->tasklet); // the tasklet was not scheduled => fixing it
      return IRQ_HANDLED;
    }

    kt = ktime_get();

    gpio = ts_data->gpio;
    gpio_level = gpio_get_value(gpio); // TODO how to avoid it?

    irq_data = &tasklet_data->irq_data[irq_data_index];
    irq_data->kt = kt;
    irq_data->pfile = pfile;
    irq_data->gpio_level = gpio_level;

    tasklet_data->write_index = irq_data_next_index;
    tasklet_schedule(&tasklet_data->tasklet);

    return IRQ_HANDLED;
}

static uint32_t ns2us(s64 ns) {
    s64 n = ns + 500;
    do_div(n, 1000);
    return (uint32_t)n;
}

static void gpio_ts_irq_tasklet_function(unsigned long gpio_index) {
    struct file* pfile;
    struct gpio_ts_data* ts_data;
    struct gpio_ts_irq_data* irq_data;
    gpio_ts_tasklet_data_t* tasklet_data;
    unsigned irq_data_index;

    ktime_t kt;
    s64 time_ns;
    s64 duration_ns;
    uint32_t duration;

    unsigned last_status;
    unsigned new_status;
    unsigned level;
    unsigned current_index;
    unsigned next_index;
    unsigned sequence_length;
    bool new_signal;
    bool close_sequence;

    tasklet_data = &tasklets[gpio_index];

    while (true) {

        irq_data_index = tasklet_data->read_index;
        if (tasklet_data->write_index == irq_data_index) break;
        irq_data = &tasklet_data->irq_data[irq_data_index];
        tasklet_data->read_index = (irq_data_index+1)&(N_IRQ_DATA_SLOTS-1);

        pfile = (struct file*)irq_data->pfile;
        ts_data = pfile == NULL ? NULL : (struct gpio_ts_data*)pfile->private_data;
        if (ts_data == NULL || !ts_data->enabled) continue; // file close(closing) => skip this item

        level = irq_data->gpio_level;
        kt = irq_data->kt;

        time_ns = ktime_to_ns(kt);
        close_sequence = false;

        //spin_lock(&(ts_data->spinlock));

        last_status = ts_data->last_status;
        if (last_status == level) goto irq_handled; // Missed IRQ - short spike - ignore it.
        if (last_status == STATUS_NOISE || last_status == STATUS_LOST_DATA) {
            if (level == 0) {
              // previous level is unknown and new level is low but sequence must start with high level => ignore
              goto irq_handled;
            }
            new_status = level;
            goto set_ts_and_status;
        }

        duration_ns = time_ns - ts_data->last_timestamp;
        duration = duration_ns < 0 || duration_ns >= GPIO_TS_MAX_DURATION_NS ? GPIO_TS_MAX_DURATION : ns2us(duration_ns);
        if (duration > ts_data->max_duration || duration < ts_data->min_duration) {
            // long/short low or high level
            new_status = level==1 ? level : STATUS_NOISE;
            if (!ts_data->signal) goto set_ts_and_status;
            if (duration > GPIO_TS_MAX_DURATION) duration = GPIO_TS_MAX_DURATION;
            last_status = STATUS_NOISE; // need to close current sequence -> write noise
            new_signal = false;
            close_sequence = true;
        } else {
            new_status = level;
        }

        current_index = ts_data->write_index;
        next_index = (current_index+1) & BUFFER_INDEX_MASK;
        if (next_index == ts_data->read_index) { // overflow
            statistics.buffer_overflow_counter++;
            new_status = STATUS_LOST_DATA;
            close_sequence = true;
            new_signal = false;
            time_ns = ts_data->last_timestamp; // do not change last_timestamp
            goto set_ts_and_status;
        }
        new_signal = (last_status==STATUS_LEVEL_0 || last_status==STATUS_LEVEL_1);
        if (new_signal != ts_data->signal) { // signal switching
            if (new_signal) { // start of new sequence
                // need to write 2 items: first with noise time then another with new signal
                duration_ns = time_ns - ts_data->last_recorded_timestamp - duration_ns;
                if (duration_ns >= NSEC_PER_USEC) {
                    if (((next_index+1) & BUFFER_INDEX_MASK) == ts_data->read_index) { // overflow
                        new_status = STATUS_LOST_DATA;
                        close_sequence = true;
                        time_ns = ts_data->last_timestamp; // do not change last_timestamp
                        goto set_ts_and_status;
                    }
                    ts_data->buffer[current_index] = make_item(ts_data->delayed_status, (duration_ns >= GPIO_TS_MAX_DURATION_NS ? GPIO_TS_MAX_DURATION : ns2us(duration_ns)));
                    current_index = next_index;
                    next_index = (next_index+1) & BUFFER_INDEX_MASK;
                }
                ts_data->last_recorded_timestamp = time_ns;
                ts_data->current_seq_index = ts_data->write_index;
            } else {
                ts_data->delayed_status = last_status;
                close_sequence = true;
            }
        }

        ts_data->buffer[current_index] = make_item(last_status, duration);
        ts_data->write_index = next_index;
        ts_data->signal = new_signal;
        ts_data->last_recorded_timestamp = time_ns;
        if (new_signal) {
          ts_data->current_seq_len++;
          if (++ts_data->buffer_seq_len > MAX_SEQ_LEN_IN_BUFFER) {
            close_sequence = true;
          }
        }

        if (close_sequence) {
            sequence_length = ts_data->current_seq_len;
            //FIXME printk(KERN_INFO "%s: sequence_length=%d read_index=%d write_index=%d\n", THIS_MODULE->name, sequence_length, ts_data->read_index, ts_data->write_index);
            if (ts_data->min_seq_len > 0 && sequence_length < ts_data->min_seq_len) {
                // New sequence is too short. Ignore it.
                ts_data->current_seq_len = 0;
                ts_data->buffer_seq_len = 0;
                ts_data->write_index = ts_data->current_seq_index;
                ts_data->last_recorded_timestamp = ts_data->current_seq_timestamp;
                if (new_status != STATUS_LOST_DATA) goto set_ts_and_status;
            } else {
                if (!new_signal) ts_data->current_seq_len = 0;
                ts_data->buffer_seq_len = 0;
                ts_data->last_recorded_timestamp = time_ns;
                ts_data->current_seq_index = ts_data->write_index;
                //printk(KERN_INFO "%s: current_seq_len=%d current_seq_index=%d read_index=%d write_index=%d irq_data_overflow_counter=%d\n",
                //    THIS_MODULE->name, ts_data->current_seq_len, ts_data->current_seq_index, ts_data->read_index, ts_data->write_index, statistics.irq_data_overflow_counter);
                wake_up_interruptible(&ts_data->read_queue); // wake up readers
            }
        }

    set_ts_and_status:
        ts_data->last_timestamp = time_ns;
        ts_data->last_status = new_status;

    irq_handled:
        //spin_unlock(&(ts_data->spinlock));
        continue;
    }
}

// ----------------------- Poll operation ---------------------------------

static inline bool is_data_available(struct gpio_ts_data* ts_data ) {
  return ts_data->read_index != ts_data->current_seq_index;
}

static unsigned int gpio_ts_poll(struct file* pfile, poll_table* wait) {
    unsigned int ret = 0;
    struct gpio_ts_data* ts_data = pfile->private_data;

    if (ts_data == NULL) return POLLHUP;

    if (!ts_data->enabled) {
      enable(ts_data); // ts_data->enabled = true;
    }

    poll_wait(pfile, &ts_data->read_queue, wait);

    //spin_lock(&ts_data->spinlock);
    ts_data = pfile->private_data;
    if (ts_data == NULL || !ts_data->enabled)
      ret = POLLHUP;
    else if (is_data_available(ts_data))
      ret |= POLLIN | POLLRDNORM;
    //spin_unlock(&ts_data->spinlock);

    return ret;
 }

// ----------------------- Read operation ---------------------------------
#define length_to_n_items(len) ((len)>>2)

static ssize_t gpio_ts_read(struct file* pfile, char* buffer, size_t length, loff_t* offset) {
    struct gpio_ts_data* ts_data;
    void* start;
    unsigned long to_copy, n_bytes;
    int n_items_to_copy;
    int rc;
    int n_items_requested;
    int n_items_available;
    int index;

    //printk(KERN_INFO "%s: gpio_ts_read() isr_counter=%d\n", THIS_MODULE->name, isr_counter);
    ts_data = pfile->private_data;
    if (ts_data == NULL) return -EBADF; // file was closed

    enable(ts_data); // ts_data->enabled = true;

    if (length == 0) return 0;

    if (pfile->f_flags & O_NONBLOCK) {
        if (!is_data_available(ts_data)) return -EAGAIN;
    } else {
        rc = wait_event_interruptible(ts_data->read_queue, pfile->private_data==NULL || is_data_available(ts_data));
        if (rc != 0) return rc;
        if (pfile->private_data == NULL) return -EBADF; // file was closed
        if (!is_data_available(ts_data)) return 0; // it should not happen but just in case it happens...
    }

    rc = 0;
    n_items_requested = (int)(length_to_n_items(length));
    if (n_items_requested != 0) {

        //spin_lock(&ts_data->spinlock);

        index = ts_data->read_index;
        n_items_available = ts_data->current_seq_index - index;

        start = &(ts_data->buffer[index]);
        if (n_items_available !=0) {
          if (n_items_available < 0) n_items_available += N_BUFFER_ITEMS;
          n_items_to_copy = n_items_requested < n_items_available ? n_items_requested : n_items_available;
          index += n_items_to_copy;
          to_copy = (unsigned long)n_items_to_copy * sizeof(uint32_t);

          if (index <= N_BUFFER_ITEMS) { // is not wrapped
              if (index == N_BUFFER_ITEMS) index = 0;
              rc = copy_to_user(buffer, start, to_copy);
          } else { // wrapped
              index -= N_BUFFER_ITEMS;
              n_bytes = (unsigned long)(n_items_to_copy - index) * sizeof(uint32_t);
              rc = copy_to_user(buffer, start, n_bytes);
              if (rc == 0)
                  rc = copy_to_user(buffer + n_bytes, ts_data->buffer, (unsigned long)index * sizeof(uint32_t));
          }

          if (rc == 0) {
              ts_data->read_index = index;
              rc = to_copy;
          }
        }

        //spin_unlock(&ts_data->spinlock);

    } else { // requested less than 4 bytes
        return -EINVAL;
    }
    //printk(KERN_INFO "%s: gpio_ts_read() => %d\n", THIS_MODULE->name, rc);
    return rc;
}

// -----------------------------------------------------------------
module_init(gpio_ts_init);
module_exit(gpio_ts_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("akonshin@gmail.com");
MODULE_DESCRIPTION("GPIO timestamps stream Linux Kernel Module");
