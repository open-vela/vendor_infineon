/****************************************************************************
 *  Copyright (C) 2025 Xiaomi Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ***************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>

#include <arch/chip/chip.h>

#include "tricore_internal.h"
#include "aurix_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of tricore_earlyserialinit(),
 * tricore_serialinit(), and up_putc().
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Serial driver methods */

static int  aurix_setup(struct uart_dev_s *dev);
static void aurix_shutdown(struct uart_dev_s *dev);
static int  aurix_attach(struct uart_dev_s *dev);
static void aurix_detach(struct uart_dev_s *dev);
static int  aurix_interrupt(int irq, void *context, void *arg);
static int  aurix_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  aurix_receive(struct uart_dev_s *dev, unsigned int *status);
static void aurix_rxint(struct uart_dev_s *dev, bool enable);
static bool aurix_rxavailable(struct uart_dev_s *dev);
static void aurix_send(struct uart_dev_s *dev, int ch);
static void aurix_txint(struct uart_dev_s *dev, bool enable);
static bool aurix_txready(struct uart_dev_s *dev);
static bool aurix_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_aurix_uart_ops =
{
  .setup          = aurix_setup,
  .shutdown       = aurix_shutdown,
  .attach         = aurix_attach,
  .detach         = aurix_detach,
  .ioctl          = aurix_ioctl,
  .receive        = aurix_receive,
  .rxint          = aurix_rxint,
  .rxavailable    = aurix_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = aurix_send,
  .txint          = aurix_txint,
  .txready        = aurix_txready,
  .txempty        = aurix_txempty,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: asclin_init
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static void asclin_init(const struct aurix_uart_config_s *priv)
{
  Ifx_ASCLIN *asclin = priv->base;
  const IfxAsclin_Asc_Pins *pins = &priv->pins;

  /* enabling the module */

  IfxAsclin_enableModule(asclin);

  /* disabling the clock */

  IfxAsclin_setClockSource(asclin, IfxAsclin_ClockSource_noClock);

  /* setting the module in Initialise mode */

  IfxAsclin_setFrameMode(asclin, IfxAsclin_FrameMode_initialise);

  /* sets the prescaler */

  IfxAsclin_setPrescaler(asclin, 1);

  /* temporary set the clock source for baudrate configuration */

  IfxAsclin_setClockSource(asclin, IfxAsclin_ClockSource_ascFastClock);

  /* setting the baudrate bit fields to generate the required baudrate */

  IfxAsclin_setBitTiming(asclin, priv->baud,
                         IfxAsclin_OversamplingFactor_16,
                         IfxAsclin_SamplePointPosition_8,
                         IfxAsclin_SamplesPerBit_three);

  /* disabling the clock again */

  IfxAsclin_setClockSource(asclin, IfxAsclin_ClockSource_noClock);

  /* selecting the loopback mode */

  IfxAsclin_enableLoopBackMode(asclin, false);

  /* setting parity enable */

  IfxAsclin_enableParity(asclin, false);

  /* setting parity type (odd/even) */

  IfxAsclin_setParityType(asclin, IfxAsclin_ParityType_even);

  /* setting the stop bit */

  IfxAsclin_setStopBit(asclin, IfxAsclin_StopBit_1);

  /* setting the shift direction */

  IfxAsclin_setShiftDirection(asclin, IfxAsclin_ShiftDirection_lsbFirst);

  /* setting the data length */

  IfxAsclin_setDataLength(asclin, IfxAsclin_DataLength_8);

  /* setting Tx FIFO inlet width */

  IfxAsclin_setTxFifoInletWidth(asclin, IfxAsclin_TxFifoInletWidth_1);

  /* setting Rx FIFO outlet width */

  IfxAsclin_setRxFifoOutletWidth(asclin, IfxAsclin_RxFifoOutletWidth_1);

  /* setting idle delay */

  IfxAsclin_setIdleDelay(asclin, IfxAsclin_IdleDelay_0);

  /* setting Tx FIFO level at which a Tx interrupt will be triggered */

  IfxAsclin_setTxFifoInterruptLevel(asclin,
                                    IfxAsclin_TxFifoInterruptLevel_0);

  /* setting Rx FIFO interrupt level at which a Rx
   * interrupt will be triggered
   */

  IfxAsclin_setRxFifoInterruptLevel(asclin,
                                    IfxAsclin_RxFifoInterruptLevel_1);

  /* setting Tx FIFO interrupt generation mode */

  IfxAsclin_setTxFifoInterruptMode(asclin,
                                   IfxAsclin_FifoInterruptMode_single);

  /* setting Rx FIFO interrupt generation mode */

  IfxAsclin_setRxFifoInterruptMode(asclin,
                                   IfxAsclin_FifoInterruptMode_single);

  /* selecting the frame mode */

  IfxAsclin_setFrameMode(asclin, IfxAsclin_FrameMode_asc);

  /* Pin mapping */

  if (pins != NULL)
    {
      IfxAsclin_Cts_In *cts = pins->cts;

      if (cts != NULL)
        {
          IfxAsclin_initCtsPin(cts, pins->ctsMode, pins->pinDriver);
        }

      IfxAsclin_Rx_In *rx = pins->rx;

      if (rx != NULL)
        {
          IfxAsclin_initRxPin(rx, pins->rxMode, pins->pinDriver);
        }

      IfxAsclin_Rts_Out *rts = pins->rts;

      if (rts != NULL)
        {
          IfxAsclin_initRtsPin(rts, pins->rtsMode, pins->pinDriver);
        }

      IfxAsclin_Tx_Out *tx = pins->tx;
      if (tx != NULL)
        {
          IfxAsclin_initTxPin(tx, pins->txMode, pins->pinDriver);
        }
    }

  /* select the clock source */

  IfxAsclin_setClockSource(asclin, IfxAsclin_ClockSource_ascFastClock);

  /* disable all flags */

  IfxAsclin_disableAllFlags(asclin);

  /* clear all flags */

  IfxAsclin_clearAllFlags(asclin);

  /* HW error flags */

  IfxAsclin_enableParityErrorFlag(asclin, true);
  IfxAsclin_enableFrameErrorFlag(asclin, true);
  IfxAsclin_enableRxFifoOverflowFlag(asclin, true);
  IfxAsclin_enableRxFifoUnderflowFlag(asclin, true);
  IfxAsclin_enableTxFifoOverflowFlag(asclin, true);

  /* enable transfers */

  IfxAsclin_enableRxFifoInlet(asclin, true);
  IfxAsclin_enableTxFifoOutlet(asclin, true);
  IfxAsclin_flushRxFifo(asclin);
  IfxAsclin_flushTxFifo(asclin);
}

/****************************************************************************
 * Name: aurix_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int aurix_setup(struct uart_dev_s *dev)
{
  asclin_init(dev->priv);
  return OK;
}

/****************************************************************************
 * Name: aurix_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void aurix_shutdown(struct uart_dev_s *dev)
{
  const struct aurix_uart_config_s *priv = dev->priv;

  /* Disable interrupts */

  if (!priv->poll)
    {
      up_disable_irq(priv->irq);
    }
}

/****************************************************************************
 * Name: aurix_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode. This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/

static int aurix_attach(struct uart_dev_s *dev)
{
  const struct aurix_uart_config_s *priv = dev->priv;
  int ret = OK;

  if (!priv->poll)
    {
#ifdef CONFIG_AURIX_UART_ISR_WQUEUE
      ret = irq_attach_wqueue(priv->irq, NULL, aurix_interrupt, dev,
                              CONFIG_AURIX_UART_ISR_WQUEUE_PRIORITY);
#else

      /* Initialize interrupt generation on the peripheral */

      ret = irq_attach(priv->irq, aurix_interrupt, dev);
#endif

      if (ret == OK)
        {
          /* Enable the interrupt (RX and TX interrupts are still disabled
           * in the UART
           */

          up_enable_irq(priv->irq);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: aurix_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called. The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void aurix_detach(struct uart_dev_s *dev)
{
  const struct aurix_uart_config_s *priv = dev->priv;

  if (!priv->poll)
    {
      /* Disable interrupts */

      up_disable_irq(priv->irq);

      /* Detach from the interrupt */

#ifdef CONFIG_AURIX_UART_ISR_WQUEUE
      irq_detach_wqueue(priv->irq);
#else
      irq_detach(priv->irq);
#endif
    }
}

/****************************************************************************
 * Name: aurix_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int aurix_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = arg;

  if (aurix_rxavailable(dev))
    {
      uart_recvchars(dev);
    }

  if (aurix_txready(dev))
    {
      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int aurix_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: aurix_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int aurix_receive(struct uart_dev_s *dev, unsigned int *status)
{
  const struct aurix_uart_config_s *priv = dev->priv;

  return IfxAsclin_readRxData(priv->base);
}

/****************************************************************************
 * Name: aurix_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void aurix_rxint(struct uart_dev_s *dev, bool enable)
{
  const struct aurix_uart_config_s *priv = dev->priv;

  irqstate_t flags = enter_critical_section();
  if (!priv->poll)
    {
      IfxAsclin_enableRxFifoFillLevelFlag(priv->base, enable);
      IfxAsclin_enableRxFifoInlet(priv->base, enable);
    }
  else
    {
      if (aurix_rxavailable(dev))
        {
          uart_recvchars(dev);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: aurix_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool aurix_rxavailable(struct uart_dev_s *dev)
{
  const struct aurix_uart_config_s *priv = dev->priv;

  return IfxAsclin_getRxFifoFillLevel(priv->base) > 0;
}

/****************************************************************************
 * Name: aurix_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static struct uart_dev_s *aurix_console(void)
{
  static struct uart_dev_s *console;
  size_t i = 0;

  while (console == NULL && g_uart[i] != NULL && g_uart[i] != DEV_END)
    {
      if (g_uart[i]->isconsole)
        {
          console = g_uart[i];
          break;
        }

      i++;
    }

  return console;
}

static void aurix_send(struct uart_dev_s *dev, int ch)
{
  const struct aurix_uart_config_s *priv = dev->priv;

  /* Wait for FIFO */

  if (dev == aurix_console())
    {
      up_putc(ch);
      return;
    }
  while (IfxAsclin_getTxFifoFillLevel(priv->base) != 0);
  IfxAsclin_clearAllFlags(priv->base);
  IfxAsclin_writeTxData(priv->base, ch);
}

/****************************************************************************
 * Name: aurix_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void aurix_txint(struct uart_dev_s *dev, bool enable)
{
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
      /* Enable the TX interrupt */

      uart_xmitchars(dev);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: aurix_txready
 *
 * Description:
 *   Return true if the tranmsit data register is not full
 *
 ****************************************************************************/

static bool aurix_txready(struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: aurix_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool aurix_txempty(struct uart_dev_s *dev)
{
  const struct aurix_uart_config_s *priv = dev->priv;

  /* Return true if the TX wartermak is pending */

  return IfxAsclin_getTxFifoFillLevel(priv->base) != 0;
}

/****************************************************************************
 * Name: tricore_lowputc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void tricore_lowputc(char ch)
{
  struct uart_dev_s *console = aurix_console();

  if (console != NULL)
    {
      const struct aurix_uart_config_s *priv = console->priv;

      /* Wait for FIFO */

      while (IfxAsclin_getTxFifoFillLevel(priv->base) != 0);
      IfxAsclin_clearAllFlags(priv->base);
      IfxAsclin_writeTxData(priv->base, ch);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int aurix_uart_allinitialize(struct uart_dev_s **dev)
{
  char path[16];
  int i = 0;

  while (dev[i] && dev[i] != DEV_END)
    {
      FAR const struct aurix_uart_config_s *config = dev[i]->priv;

      if (config == NULL)
        {
          i++;
          continue;
        }

      dev[i]->ops = &g_aurix_uart_ops;
      if (dev[i]->isconsole)
        {
          uart_register("/dev/console", dev[i]);
        }

      /* Register all UARTs */

      snprintf(path, 16, "/dev/ttyS%d", config->bus);
      uart_register(path, dev[i]);
      i++;
    }

  return 0;
}

/****************************************************************************
 * Name: tricore_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before tricore_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock
 *   initialization performed in up_clkinitialize().
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void tricore_earlyserialinit(void)
{
  struct uart_dev_s *console = aurix_console();

  /* Configuration whichever one is the console */

  if (console != NULL)
    {
      aurix_setup(console);
    }
}
#endif

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
  tricore_lowputc(ch);
}
