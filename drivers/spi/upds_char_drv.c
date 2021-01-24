/****************************************************************************
 * drivers/spi/upds_char_drv.c
 *
 *   Copyright (C) 2021 Grr. All rights reserved.
 *   Author: Grr <gebbet00@gmail.com>
  *  Based on the SPI character driver by Gregory Nutt
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi_transfer.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int          upds_spi_open(FAR struct file *filep);
static int          upds_spi_close(FAR struct file *filep);
static ssize_t      upds_spi_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t      upds_spi_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int          upds_spi_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int          upds_spi_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
#ifdef CONFIG_STM32_SPI1
#ifdef CONFIG_UPDS_SPI_CHARDRIVER
static const struct file_operations g_spifops1 =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .open             = upds_spi_open,      /* open */
  .close            = upds_spi_close,     /* close */
#else
  .open             = NULL,               /* open */
  .close            = NULL,               /* close */
#endif
  .read             = upds_spi_read,      /* read */
  .write            = upds_spi_write,     /* write */
  .seek             = NULL,               /* seek */
  .ioctl            = upds_spi_ioctl,     /* ioctl */
  .poll             = NULL                /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  ,.unlink          = upds_spi_unlink     /* unlink */
#endif
};
#endif    /* CONFIG_UPDS_SPI_CHARDRIVER */
#endif    /* CONFIG_STM32_SPI1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: upds_spi_open
 *
 * Description:
 *   Open UPDS character driver file
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int upds_spi_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct upds_spibus_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct upds_spibus_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the SPI driver state structure */

  ret = nxsem_wait(&priv->filesem);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  nxsem_post(&priv->filesem);
  return OK;
}
#endif

/****************************************************************************
 * Name: upds_spi_close
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int upds_spi_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct upds_spibus_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct upds_spibus_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the SPI driver state structure */

  ret = nxsem_wait(&priv->filesem);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then commit Hara-Kiri now.
   */

  if (priv->crefs <= 0 && priv->unlinked)
    {
      nxsem_destroy(&priv->filesem);
      kmm_free(priv);
      return OK;
    }

  nxsem_post(&priv->filesem);
  return OK;
}
#endif

/****************************************************************************
 * Name: upds_spi_read
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/
static ssize_t upds_spi_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: upds_spi_write
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/
static ssize_t upds_spi_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  return len; /* Say that everything was written */
}

/****************************************************************************
 * Name: upds_spi_ioctl
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/
static int upds_spi_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct upds_spibus_s *priv;
  FAR struct spi_sequence_s *seq;
  int ret;

  spiinfo("cmd=%d arg=%lu\n", cmd, arg);

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  priv = (FAR struct upds_spibus_s *)inode->i_private;
  DEBUGASSERT(priv);

  /* Get exclusive access to the SPI driver state structure */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  ret = nxsem_wait(&priv->filesem);
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* Process the IOCTL command */

  switch (cmd)
    {
      /* Command:      SPIIOC_TRANSFER
       * Description:  Perform a sequence of SPI transfers
       * Argument:     A reference to an instance of struct spi_sequence_s.
       * Dependencies: CONFIG_SPI_DRIVER
       */

      case SPIIOC_TRANSFER:
        {
          /* Get the reference to the spi_transfer_s structure */

          seq = (FAR struct spi_sequence_s *)((uintptr_t)arg);
          DEBUGASSERT(seq != NULL);

          /* Perform the transfer */

          ret = spi_transfer(priv, seq);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  nxsem_post(&priv->filesem);
#endif
  return ret;
}

/****************************************************************************
 * Name: upds_spi_unlink
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int upds_spi_unlink(FAR struct inode *inode)
{
  FAR struct upds_spibus_s *priv;
  int ret;

  /* Get our private data structure */

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  priv = (FAR struct upds_spibus_s *)inode->i_private;

  /* Get exclusive access to the SPI driver state structure */

  ret = nxsem_wait(&priv->filesem);
  if (ret < 0)
    {
      return ret;
    }

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      nxsem_destroy(&priv->filesem);
      kmm_free(priv);
      return OK;
    }

  /* No... just mark the driver as unlinked and free the resouces when the
   * last client closes their reference to the driver.
   */

  priv->unlinked = true;
  nxsem_post(&priv->filesem);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_register
 *
 * Description:
 *   Create and register the SPI character driver.
 *
 *   The SPI character driver is a simple character driver that supports SPI
 *   transfers.  The intent of this driver is to support SPI testing.  It is
 *   not suitable for use in any real driver application.
 *
 * Input Parameters:
 *   spi - An instance of the lower half SPI driver
 *   bus - The SPI bus number.  This will be used as the SPI device minor
 *     number.  The SPI character device will be registered as /dev/spiN
 *     where N is the minor number
 *
 * Returned Value:
 *   OK if the driver was successfully register; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/
uint8_t upds_spibus_char_drv(FAR struct upds_spibus_s *bus)
{
  uint8_t ret;


  /* Load file ops struct */

  bus->fops = &g_spifops1;

  /* Configure file ops semaphore */

  nxsem_init(&bus->filesem, 0, 1);

  /* Create character driver */

  ret = (uint8_t)register_driver(bus->devname, bus->fops, 0666, bus);
  if (ret < 0)
    {
      /* Disable character driver */

      bus->cdstate = false;
    }
  else
    {
      /* Enable character driver */

      bus->cdstate = true;
    }
}
