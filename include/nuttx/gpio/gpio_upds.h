/****************************************************************************
 * include/nuttx/gpio/gpio.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_GPIO_GPIO_H
#define __INCLUDE_NUTTX_GPIO_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/signal.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Max num of available GPIOs
   Increasing this will only increase the array size of gpiodir in 
   upds_gpio_initialize(), necesary to create GPIOs. You also have to
   increase individual GPIO sections at the beginning of upds_gpio_initialize()
   Any GPIO with number equal or bigger than this limit will NOT be created   */
#define UPDS_GPIO_NUM_MAX             16 
#define UPDS_GPIO_DEVNAME_SIZE        10
#define PIN_NUM_MAX                   15


/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Identifies the type of the GPIO pin */

enum upds_gpiotype_e
{
  GPIO_INPUT_PIN = 0, /* float */
  GPIO_INPUT_PIN_PULLUP,
  GPIO_INPUT_PIN_PULLDOWN,
  GPIO_OUTPUT_PIN, /* push-pull */
  GPIO_OUTPUT_PIN_OPENDRAIN,
  GPIO_INTERRUPT_PIN,
  GPIO_INTERRUPT_HIGH_PIN,
  GPIO_INTERRUPT_LOW_PIN,
  GPIO_INTERRUPT_RISING_PIN,
  GPIO_INTERRUPT_FALLING_PIN,
  GPIO_INTERRUPT_BOTH_PIN,
  GPIO_NPINTYPES
};

enum upds_gpiospeed_e
{
  GPIO_VERY_LOW_SPEED = 0,
  GPIO_LOW_SPEED,
  GPIO_MEDIUM_SPEED,
  GPIO_HIGH_SPEED
};

/* Interrupt callback */

struct upds_gpiodev_s;
typedef CODE int (*pin_interrupt_t)(FAR struct upds_gpiodev_s *dev, uint8_t pin);

/* Pin interface vtable definition.  Instances of this vtable are read-only
 * and may reside in FLASH.
 *
 *   - gpio_read.  Required for all all pin types.
 *   - gpio_write.  Required only for the GPIO_OUTPUT_PIN pin type.  Unused
 *     for other pin types may be NULL.
 *   - gpio_attach and gp_eanble.  Required only the GPIO_INTERRUPT_PIN pin
 *     type.  Unused for other pin types may be NULL.
 *   - gpio_setpinytype.  Required for all all pin types.
 */

struct upds_gpiodev_s;
struct upds_gpioops_s
{
  /* Interface methods */

  CODE int (*gpio_config)(uint32_t cfgset);
  CODE int (*gpio_read)(FAR struct upds_gpiodev_s *dev, FAR bool *value);
  CODE int (*gpio_write)(FAR struct upds_gpiodev_s *dev, bool value);
  CODE int (*gpio_attach)(FAR struct upds_gpiodev_s *dev,
                        pin_interrupt_t callback);
  CODE int (*gpio_enable)(FAR struct upds_gpiodev_s *dev, bool enable);
  CODE int (*gpio_setpintype)(FAR struct upds_gpiodev_s *dev,
                            enum upds_gpiotype_e pintype);
};

/* Signal information */

struct upds_gpiosignal_s
{
  struct sigevent gp_event;
  struct sigwork_s gp_work;
  pid_t gp_pid;        /* The task to be signaled */
};

/* GPIO configuration */

struct upds_gpiodev_s
{
  uint8_t   num;                            /* GPIO device number             */
  uint32_t  stdmode;                        /* GPIO standard codification     */
  uint8_t   type;                           /* GPIO device type               */
  uint8_t   mode;                           /* As output, it's speed config   */
  uint8_t   pin;                            /* Pin number                     */
  uint8_t   port;                           /* A=0, B=1, C=2, etc             */
  uint32_t  address;                        /* Port register adress           */
  char      devname[UPDS_GPIO_DEVNAME_SIZE];  /* Node name under /dev         */

  FAR const struct upds_gpioops_s *ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
uint32_t  upds_get_stdconfig(FAR struct upds_gpiodev_s *priv);
int       upds_gpio_pinconfig(FAR struct upds_gpiodev_s *priv);
uint8_t   upds_gpio_initialize(void);



#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif


#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_GPIO_GPIO_H */
