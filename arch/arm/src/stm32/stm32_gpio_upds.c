/****************************************************************************
 * arch/arm/src/stm32/stm32_gpio_upds.c
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
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/gpio/gpio_upds.h>

#include "stm32_gpio.h"


/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: upds_get_stdconfig
 *
 * Description:
 *   Combine parameters to obtain standard config set
 *
 * Input Parameters:
 *   priv   - private GPIO device structure
 *
 * Returned Value:
 *   32bit cfgset
 *
 ************************************************************************************/
uint32_t upds_get_stdconfig(FAR struct upds_gpiodev_s *priv)
{
  uint32_t  ret = 0;
  uint32_t  mode;
  uint32_t  stdconfig;


  /* Verify valid pin */

  if (priv->pin > PIN_NUM_MAX)
    {
      return 0;
    }

  /* Veryfy valid port */

  if (priv->port >= STM32_NGPIO_PORTS)
    {
      return 0;
    }

  /* Start with bit encoding */

  stdconfig = priv->pin << GPIO_PIN_SHIFT;

  /* Port encoding */

  stdconfig |= (priv->port << GPIO_PORT_SHIFT);

  /* Type of GPIO */

  switch (priv->type)
  {
    /* Configure as FLOAT INPUT */

    case GPIO_INPUT_PIN:
      stdconfig |= GPIO_FLOAT;
      break;
    
    /* Configure as PULL UP INPUT */

    case GPIO_INPUT_PIN_PULLUP:
      stdconfig |= GPIO_PULLUP;
      break;
    
    /* Configure as PULL DOWN INPUT */

    case GPIO_INPUT_PIN_PULLDOWN:
      stdconfig |= GPIO_PULLDOWN;
      break;
    
    /* Configure as PUSHPULL OUTPUT */

    case GPIO_OUTPUT_PIN:
      stdconfig |= GPIO_PUSHPULL;
      break;

    /* Configure as OPEN COLLECTOR OUTPUT */

    case GPIO_OUTPUT_PIN_OPENDRAIN:
      stdconfig |= GPIO_OPENDRAIN;
      break;

    /* Configure as IRQ */

    case GPIO_INTERRUPT_PIN:
      break;
    
    default:
      return 0;
  }

  /* If it's any type of output */

  if(priv->type == GPIO_OUTPUT_PIN ||
     priv->type == GPIO_OUTPUT_PIN_OPENDRAIN)
    {

#if defined(CONFIG_STM32_STM32F10XX)

      /* Order the weird F10XX modes */
      if(priv->mode == GPIO_VERY_LOW_SPEED ||
         priv->mode == GPIO_LOW_SPEED)
        {
          mode = GPIO_MODE_2MHz;
        }
      else if(priv->mode == GPIO_MEDIUM_SPEED)
        {
          mode = GPIO_MODE_10MHz;
        }
      else
        {
          mode = GPIO_MODE_50MHz;
        }

#elif defined(CONFIG_STM32_STM32L15XX) || defined(CONFIG_STM32_STM32F20XX) || \
      defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX) || \
      defined(CONFIG_STM32_STM32F37XX) || defined(CONFIG_STM32_STM32F4XXX) || \
      defined(CONFIG_STM32_STM32G4XXX)

      /* Load speed */

      if(priv->mode == GPIO_VERY_LOW_SPEED)
        {
#if defined(CONFIG_STM32_STM32L15XX)
          mode = GPIO_SPEED_400KHz;
#elif defined(CONFIG_STM32_STM32G4XXX)
          mode = GPIO_SPEED_5MHz;
#else
          mode = GPIO_SPEED_2MHz;
#endif
        }
      else if(priv->mode == GPIO_LOW_SPEED)
        {
#if defined(CONFIG_STM32_STM32L15XX)
          mode = GPIO_SPEED_2MHz;
#elif defined(CONFIG_STM32_STM32G4XXX)
          mode = GPIO_SPEED_25MHz;
#else
          mode = GPIO_SPEED_25MHz;
#endif
        }
      else if(priv->mode == GPIO_MEDIUM_SPEED)
        {
#if defined(CONFIG_STM32_STM32L15XX)
          mode = GPIO_SPEED_10MHz;
#elif defined(CONFIG_STM32_STM32G4XXX)
          mode = GPIO_SPEED_50MHz;
#else
          mode = GPIO_SPEED_50MHz;
#endif
        }
      else  /* priv->mode == GPIO_HIGH_SPEED  */
        {
#if defined(CONFIG_STM32_STM32L15XX)
          mode = GPIO_SPEED_40MHz;
#elif defined(CONFIG_STM32_STM32G4XXX)
          mode = GPIO_SPEED_120MHz;
#else
# ifndef CONFIG_STM32_STM32F30XX
          mode = GPIO_SPEED_100MHz;
# else
          mode = GPIO_SPEED_50MHz;  /* For CONFIG_STM32_STM32F30XX, HIGH_SPEED
                                       will be the same as MEDIUM_SPEED       */
# endif  /* CONFIG_STM32_STM32F30XX */
#endif
        }
#endif  /*defined(CONFIG_STM32_STM32L15XX) || defined(CONFIG_STM32_STM32F20XX) ||
          defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX) ||
          defined(CONFIG_STM32_STM32F37XX) || defined(CONFIG_STM32_STM32F4XXX) ||
          defined(CONFIG_STM32_STM32G4XXX)  */

      stdconfig |= mode;                /* Include the selected mode          */
    }

  return stdconfig;
}


/************************************************************************************
 * Name: upds_gpio_pinconfig
 *
 * Description:
 *   Relays configuration of pin according to chip details
 *
 * Input Parameters:
 *   priv = GPIO structure pointer
 *
 * Returned Value:
 *   
 *
 ************************************************************************************/
int upds_gpio_pinconfig(FAR struct upds_gpiodev_s *priv)
{
  uint32_t  stdconfig;


  /* Obtain standard config set */

  stdconfig = upds_get_stdconfig(priv);

  /* 0 means invalid parameters */

  if(!stdconfig)
    {
      return -EINVAL;
    }

  /* Config pin with basic function */

  return stm32_configgpio(stdconfig);
}
