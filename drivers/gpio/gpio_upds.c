/****************************************************************************
 * drivers/gpio/gpio_upds.c
 *
 *   Copyright (C) 2021 Grr. All rights reserved.
 *   Author: Grr <gebbet00@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/irq.h>
#include <nuttx/gpio/gpio_upds.h>
#include <nuttx/fs/fs.h>


/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/


/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/
uint8_t         upds_create_pseudosysfs(void);
static int      upds_gpio_open(FAR struct file *filep);
static int      upds_gpio_close(FAR struct file *filep);
static ssize_t  upds_gpio_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
static ssize_t  upds_gpio_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static off_t    upds_gpio_seek(FAR struct file *filep, off_t offset, int whence);
static int      upds_gpio_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/
static const struct upds_gpioops_s g_gpioops =
{
  .gpio_read        = NULL,
  .gpio_write       = NULL,
};

static const struct file_operations g_gpio_fops =
{
  .open             = upds_gpio_open,   /* open */
  .close            = upds_gpio_close,  /* close */
  .read             = upds_gpio_read,   /* read */
  .write            = upds_gpio_write,  /* write */
  .seek             = upds_gpio_seek,   /* seek */
  .ioctl            = upds_gpio_ioctl,  /* ioctl */
  .poll             = NULL         /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  ,.unlink          = NULL       /* unlink */
#endif
};

#ifdef CONFIG_GPIO_1
static struct upds_gpiodev_s g_gpio1 =
{
  .num              = 1,
  .type             = GPIO_1_TYPE,
  .mode             = GPIO_1_MODE,
  .pin              = GPIO_1_PIN,
  .port             = GPIO_1_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_1 */

#ifdef CONFIG_GPIO_2
static struct upds_gpiodev_s g_gpio2 =
{
  .num              = 2,
  .type             = GPIO_2_TYPE,
  .mode             = GPIO_2_MODE,
  .pin              = GPIO_2_PIN,
  .port             = GPIO_2_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_2 */

#ifdef CONFIG_GPIO_3
static struct upds_gpiodev_s g_gpio3 =
{
  .num              = 3,
  .type             = GPIO_3_TYPE,
  .mode             = GPIO_3_MODE,
  .pin              = GPIO_3_PIN,
  .port             = GPIO_3_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_3 */

#ifdef CONFIG_GPIO_4
static struct upds_gpiodev_s g_gpio4 =
{
  .num              = 4,
  .type             = GPIO_4_TYPE,
  .mode             = GPIO_4_MODE,
  .pin              = GPIO_4_PIN,
  .port             = GPIO_4_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_4 */

#ifdef CONFIG_GPIO_5
static struct upds_gpiodev_s g_gpio5 =
{
  .num              = 5,
  .type             = GPIO_5_TYPE,
  .mode             = GPIO_5_MODE,
  .pin              = GPIO_5_PIN,
  .port             = GPIO_5_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_5 */

#ifdef CONFIG_GPIO_6
static struct upds_gpiodev_s g_gpio6 =
{
  .num              = 6,
  .type             = GPIO_6_TYPE,
  .mode             = GPIO_6_MODE,
  .pin              = GPIO_6_PIN,
  .port             = GPIO_6_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_6 */

#ifdef CONFIG_GPIO_7
static struct upds_gpiodev_s g_gpio7 =
{
  .num              = 7,
  .type             = GPIO_7_TYPE,
  .mode             = GPIO_7_MODE,
  .pin              = GPIO_7_PIN,
  .port             = GPIO_7_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_7 */

#ifdef CONFIG_GPIO_8
static struct upds_gpiodev_s g_gpio8 =
{
  .num              = 8,
  .type             = GPIO_8_TYPE,
  .mode             = GPIO_8_MODE,
  .pin              = GPIO_8_PIN,
  .port             = GPIO_8_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_8 */

#ifdef CONFIG_GPIO_9
static struct upds_gpiodev_s g_gpio9 =
{
  .num              = 9,
  .type             = GPIO_9_TYPE,
  .mode             = GPIO_9_MODE,
  .pin              = GPIO_9_PIN,
  .port             = GPIO_9_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_9 */

#ifdef CONFIG_GPIO_10
static struct upds_gpiodev_s g_gpio10 =
{
  .num              = 10,
  .type             = GPIO_10_TYPE,
  .mode             = GPIO_10_MODE,
  .pin              = GPIO_10_PIN,
  .port             = GPIO_10_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_10 */

#ifdef CONFIG_GPIO_11
static struct upds_gpiodev_s g_gpio11 =
{
  .num              = 11,
  .type             = GPIO_11_TYPE,
  .mode             = GPIO_11_MODE,
  .pin              = GPIO_11_PIN,
  .port             = GPIO_11_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_11 */

#ifdef CONFIG_GPIO_12
static struct upds_gpiodev_s g_gpio12 =
{
  .num              = 12,
  .type             = GPIO_12_TYPE,
  .mode             = GPIO_12_MODE,
  .pin              = GPIO_12_PIN,
  .port             = GPIO_12_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_12 */

#ifdef CONFIG_GPIO_13
static struct upds_gpiodev_s g_gpio13 =
{
  .num              = 13,
  .type             = GPIO_13_TYPE,
  .mode             = GPIO_13_MODE,
  .pin              = GPIO_13_PIN,
  .port             = GPIO_13_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_13 */

#ifdef CONFIG_GPIO_14
static struct upds_gpiodev_s g_gpio14 =
{
  .num              = 14,
  .type             = GPIO_14_TYPE,
  .mode             = GPIO_14_MODE,
  .pin              = GPIO_14_PIN,
  .port             = GPIO_14_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_14 */

#ifdef CONFIG_GPIO_15
static struct upds_gpiodev_s g_gpio15 =
{
  .num              = 15,
  .type             = GPIO_15_TYPE,
  .mode             = GPIO_15_MODE,
  .pin              = GPIO_15_PIN,
  .port             = GPIO_15_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_15 */

#ifdef CONFIG_GPIO_16
static struct upds_gpiodev_s g_gpio16 =
{
  .num              = 16,
  .type             = GPIO_16_TYPE,
  .mode             = GPIO_16_MODE,
  .pin              = GPIO_16_PIN,
  .port             = GPIO_16_PORT,
  .ops              = &g_gpioops
};
#endif  /* CONFIG_GPIO_16 */


/****************************************************************************
 * Private Functions
 ****************************************************************************/
/************************************************************************************
 * Name: upds_create_pseudosysfs
 *
 * Description:
 *   Creates directory structure for registering devices
 *
 * Input Parameters:
 *   
 *
 * Returned Value:
 *   
 *
 ************************************************************************************/
uint8_t upds_create_pseudosysfs()
{

}

/****************************************************************************
 * Name: gpio_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int upds_gpio_open(FAR struct file *filep)
{
  filep->f_pos = 0;
  return OK;
}

/****************************************************************************
 * Name: gpio_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int upds_gpio_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: gpio_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t upds_gpio_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{

}

/****************************************************************************
 * Name: gpio_write
 *
 * Description:
 *   Standard character driver write method.
 *
 *   REVISIT:  The read() method obeys the semantics of a file position and
 *   requires re-opening the driver or seeking to address 0.  The write()
 *   method does not.  This is an inconsistency.
 *
 ****************************************************************************/

static ssize_t upds_gpio_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{

}

/****************************************************************************
 * Name: gpio_seek
 *
 * Description:
 *   Reset read flag on seek to 0
 *
 *   REVISIT:  Seeking address zero is required to return addition GPIO
 *   values from read().  This, however, is an un-natural use of a file
 *   position since there is no file position associated with a GPIO.  It
 *   also makes the read() method difficult to use programmatically.
 *
 ****************************************************************************/

static off_t upds_gpio_seek(FAR struct file *filep, off_t offset, int whence)
{

}

/****************************************************************************
 * Name: gpio_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int upds_gpio_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{

}

/************************************************************************************
 * Name: upds_gpioregister
 *
 * Description:
 *   Registers GPIO device in pseudo sysfs dir
 *
 * Input Parameters:
 *   priv   - private GPIO device structure
 *
 * Returned Value:
 *   
 *
 ************************************************************************************/
int upds_gpioregister(FAR struct upds_gpiodev_s *priv)
{
  uint8_t ret = 0;
  char    devname[UPDS_GPIO_DEVNAME_SIZE + 5];


/* Type of GPIO */

  switch (priv->type)
  {
    /* Configure INPUT name */

    case GPIO_INPUT_PIN:
    case GPIO_INPUT_PIN_PULLUP:
    case GPIO_INPUT_PIN_PULLDOWN:
      snprintf(priv->devname, UPDS_GPIO_DEVNAME_SIZE - 1, "gpin%u", priv->num);
      break;
    
    /* Configure OUTPUT name */

    case GPIO_OUTPUT_PIN:
    case GPIO_OUTPUT_PIN_OPENDRAIN:
      snprintf(priv->devname, UPDS_GPIO_DEVNAME_SIZE - 1, "gpout%u", priv->num);
      break;

    /* Configure as IRQ */

    case GPIO_INTERRUPT_PIN:
      snprintf(priv->devname, UPDS_GPIO_DEVNAME_SIZE - 1, "gpirq%u", priv->num);
      break;
    
    default:
      return -EINVAL;
  }

  /* compose the full device name */

  snprintf(devname, UPDS_GPIO_DEVNAME_SIZE + 5, "/dev/%s", priv->devname );

  return register_driver(devname, &g_gpio_fops, 0666, priv);
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: upds_gpio_initialize
 *
 * Description:
 *   Initialize GPIO subsystem
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   
 *
 ************************************************************************************/
uint8_t upds_gpio_initialize(void)
{
  uint8_t   ret = 0;
  uint8_t   numgpios = 0;
  uint8_t   i;
  int       intret = 0;
  struct upds_gpiodev_s *priv;
  struct upds_gpiodev_s *gpiodir[UPDS_GPIO_NUM_MAX];


#if 0
  /* Create general structure for registering devices */

  upds_create_pseudosysfs();
#endif

/* Make sure unused cells are null */

memset(gpiodir, 0, sizeof(gpiodir));

/* Fill GPIO structure directory to allow loop processing */

#ifdef CONFIG_GPIO_1
  gpiodir[numgpios] = &g_gpio1;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_2
  gpiodir[numgpios] = &g_gpio2;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_3
  gpiodir[numgpios] = &g_gpio3;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_4
  gpiodir[numgpios] = &g_gpio4;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_5
  gpiodir[numgpios] = &g_gpio5;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_6
  gpiodir[numgpios] = &g_gpio6;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_7
  gpiodir[numgpios] = &g_gpio7;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_8
  gpiodir[numgpios] = &g_gpio8;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_9
  gpiodir[numgpios] = &g_gpio9;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_10
  gpiodir[numgpios] = &g_gpio10;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_11
  gpiodir[numgpios] = &g_gpio11;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_12
  gpiodir[numgpios] = &g_gpio12;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_13
  gpiodir[numgpios] = &g_gpio13;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_14
  gpiodir[numgpios] = &g_gpio14;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_15
  gpiodir[numgpios] = &g_gpio15;
  numgpios++;
#endif

#ifdef CONFIG_GPIO_16
  gpiodir[numgpios] = &g_gpio16;
  numgpios++;
#endif

/* Scan registered GPIOs */

for(i = 0; i < numgpios; i++)
  {
    /* Load corresponding GPIO struct addr */

    priv = gpiodir[i];

    /* Config pin */

    intret = upds_gpio_pinconfig(priv);

    /* Register GPIO */

    intret = upds_gpioregister(priv);
  }

  return ret;
}

