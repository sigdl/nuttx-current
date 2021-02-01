/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_spi_upds.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi_upds.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "chip.h"
#include "stm32.h"

#include "stm32f4discovery.h"


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

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: upds_spi_select
 *
 * Description:
 *   
 * Input Parameters:
 *   
 *   
 *
 * Returned Value:
 *   
 *
 ************************************************************************************/
void upds_spi_select(FAR struct upds_spibus_s *bus, uint32_t devid, bool selected)
{

}


/************************************************************************************
 * Name: upds_spi_status
 *
 * Description:
 *   
 * Input Parameters:
 *   
 *   
 *
 * Returned Value:
 *   
 *
 ************************************************************************************/
uint8_t upds_spi_status(FAR struct upds_spibus_s *bus, uint32_t devid)
{
  uint8_t ret = 0;


  return ret;
}


/************************************************************************************
 * Name: upds_spi_cmddata
 *
 * Description:
 *   
 * Input Parameters:
 *   
 *   
 *
 * Returned Value:
 *   
 *
 ************************************************************************************/
int upds_spi_cmddata(FAR struct upds_spibus_s *bus, uint32_t devid, bool cmd)
{
  int ret = 0;


  return ret;
}
