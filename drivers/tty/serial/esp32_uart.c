#include <linux/console.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>

#define DRIVER_NAME	"esp32-uart"
#define DEV_NAME	"ttyS"
#define UART_NR		3

#define UART_FIFO_REG			0x00
#define UART_STATUS_REG			0x1c
#define UART_MEM_CNT_STATUS_REG		0x64

#define DPORT_PRO_INTR_STATUS_REG_1_REG


// get tx fifo character count from status register
#define UART_TXFIFO_STATUS_MASK		0x00ff0000
#define UART_TXFIFO_STATUS_SHIFT	16
#define UART_TXFIFO_CNT_MASK		0x00000038
#define UART_TXFIFO_CNT_SHIFT		5

// get tx fifo character count
#define UART_RXFIFO_STATUS_MASK		0x000000ff
#define UART_RXFIFO_STATUS_SHIFT	0
#define UART_RXFIFO_CNT_MASK		0x00000007
#define UART_RXFIFO_CNT_SHIFT		8

#define USE_TIMER 1
#define USE_IRQ 0

static const struct of_device_id esp32_dt_ids[] = {
	{
		.compatible = "esp,esp32-uart",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, esp32_dt_ids);

struct esp32uart_port {
	struct uart_port port;
	struct timer_list timer;
	u32 id;
};

static struct esp32uart_port *esp32_ports[UART_NR];
static struct uart_port *earlycon_port;

#define to_esp32uart_port(port)	container_of(port, struct esp32uart_port, port)

/* configure/auto-configure the port */
static void esp32_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_ESP32UART;
}

static void esp32_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int esp32_get_mctrl(struct uart_port *port)
{
	return 0;
}

static unsigned int esp32_rx_fifo_cnt(struct uart_port *port)
{
	uint32_t status_reg = readl(port->membase + UART_STATUS_REG);
	uint32_t mem_cnt_reg = readl(port->membase + UART_MEM_CNT_STATUS_REG);
	return ((mem_cnt_reg & UART_RXFIFO_CNT_MASK) << UART_RXFIFO_CNT_SHIFT) |
		((status_reg & UART_RXFIFO_STATUS_MASK) >> UART_RXFIFO_STATUS_SHIFT);
}

static void read_characters_and_flip(struct uart_port *port)
{
	struct tty_port *tport = &port->state->port;
	int i;

	// read characters while there are characters available
	for (i=0; esp32_rx_fifo_cnt(port); i++) {
		// read single character from the fifo(RX)
		int ch = readl(port->membase + UART_FIFO_REG) & 0xff;
		tty_insert_flip_char(tport, ch, TTY_NORMAL);
	}

	// push the buffer if at least one character was received
	if (i>0)
		tty_flip_buffer_push(&port->state->port);
}

#if USE_TIMER
static void esp32_timer(struct timer_list *t)
{
	// This implementation is a hack! It should really use interrupts...
	struct esp32uart_port *uart = from_timer(uart, t, timer);
	struct uart_port *port = &uart->port;

	read_characters_and_flip(port);

	// restart the timer
	mod_timer(&uart->timer, jiffies + uart_poll_timeout(port));
	//mod_timer(&uart->timer, jiffies + 1);
}
#endif

#if USE_IRQ
static irqreturn_t esp32_uart_interrupt(int irq, void *dev_id)
{
	struct esp32uart_port *uart = dev_id;
	struct uart_port *port = &uart->port;

	pr_info("\nXXX UART INTERRUPT\n");

	//spin_lock(&port->lock);

	read_characters_and_flip(port);

	//spin_unlock(&port->lock);

	return IRQ_HANDLED;
}
#endif

static int esp32_startup(struct uart_port *port)
{
	struct esp32uart_port *uart = to_esp32uart_port(port);

#if USE_IRQ
	int retval;

	/* Allocate the IRQ */
	pr_info("Trying to get IRQ: %d", port->irq);
	retval = request_irq(port->irq, esp32_uart_interrupt, 0, "esp32_uart", uart);
	if (retval)
		return retval;
#endif


#if USE_TIMER


	timer_setup(&uart->timer, esp32_timer, 0);
	mod_timer(&uart->timer, jiffies + uart_poll_timeout(port));
	//mod_timer(&uart->timer, jiffies + 1);
#endif

	//spin_lock_init(&port->lock);

	return 0;
}

static void esp32_shutdown(struct uart_port *port)
{
}

static const char *esp32_type(struct uart_port *port)
{
	return "ESP32 UART";
}

static int esp32_request_port(struct uart_port *port)
{
	return 0;
}

static void esp32_release_port(struct uart_port *port)
{
}

static void esp32_set_termios(struct uart_port *port,
			      struct ktermios *termios,
			      struct ktermios *old)
{
}

static void esp32_break_ctl(struct uart_port *port, int break_state)
{
}

static void esp32_stop_rx(struct uart_port *port)
{
}

static unsigned int esp32_tx_fifo_cnt(struct uart_port *port)
{
	uint32_t status_reg, mem_cnt_reg;

	status_reg = readl(port->membase + UART_STATUS_REG);
	mem_cnt_reg = readl(port->membase + UART_MEM_CNT_STATUS_REG);
	return ((mem_cnt_reg & UART_TXFIFO_CNT_MASK) << UART_TXFIFO_CNT_SHIFT) |
		((status_reg & UART_TXFIFO_STATUS_MASK) >> UART_TXFIFO_STATUS_SHIFT);
}

/* return TIOCSER_TEMT when transmitter is not busy */
static unsigned int esp32_tx_empty(struct uart_port *port)
{
	unsigned long status;

	status = esp32_tx_fifo_cnt(port);

	return status ? TIOCSER_TEMT : 0;
}

static void esp32_stop_tx(struct uart_port *port)
{
	//unsigned long ier;

	//ier = readl(port->membase + LINIER);
	//ier &= ~(LINFLEXD_LINIER_DTIE);
	//writel(ier, port->membase + LINIER);
}

static void esp32_put_char(struct uart_port *sport, unsigned char c)
{
	while (esp32_tx_fifo_cnt(sport) > 126)
		barrier();
	writel(c, sport->membase + UART_FIFO_REG);
}

static inline void esp32_transmit_buffer(struct uart_port *sport)
{
	struct circ_buf *xmit = &sport->state->xmit;

	while (!uart_circ_empty(xmit)) {
		esp32_put_char(sport, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(sport);

	if (uart_circ_empty(xmit))
		esp32_stop_tx(sport);
}

static void esp32_start_tx(struct uart_port *port)
{
	esp32_transmit_buffer(port);
}

static const struct uart_ops esp32_pops = {
	.config_port	= esp32_config_port,
	.set_mctrl	= esp32_set_mctrl,
	.get_mctrl	= esp32_get_mctrl,
	.tx_empty	= esp32_tx_empty,
	.stop_tx	= esp32_stop_tx,
	.start_tx	= esp32_start_tx,
	.startup	= esp32_startup,
	.shutdown	= esp32_shutdown,
	.type		= esp32_type,
	.request_port	= esp32_request_port,
	.release_port	= esp32_release_port,
	.set_termios	= esp32_set_termios,
	.break_ctl	= esp32_break_ctl,
	.stop_rx	= esp32_stop_rx,
};

static void esp32_console_putchar(struct uart_port *port, unsigned char ch)
{
	esp32_put_char(port, ch);
}

static void esp32_string_write(struct uart_port *sport, const char *s,
			       unsigned int count)
{
	uart_console_write(sport, s, count, esp32_console_putchar);
}

static void
esp32_console_write(struct console *co, const char *s, unsigned int count)
{
	struct esp32uart_port *eport = esp32_ports[co->index];
	struct uart_port *sport = &eport->port;
	unsigned long flags;
	int locked = 1;

	if (sport->sysrq)
		locked = 0;
	else if (oops_in_progress)
		locked = spin_trylock_irqsave(&sport->lock, flags);
	else
		spin_lock_irqsave(&sport->lock, flags);

	esp32_string_write(sport, s, count);

	if (locked)
		spin_unlock_irqrestore(&sport->lock, flags);
}

static int __init esp32_console_setup(struct console *co, char *options)
{
	struct esp32uart_port *eport;
	struct uart_port *sport;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;
	/*
	 * check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index == -1 || co->index >= ARRAY_SIZE(esp32_ports))
		co->index = 0;

	eport = esp32_ports[co->index];

	if (!eport)
		return -ENODEV;

	sport = &eport->port;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	ret = uart_set_options(sport, co, baud, parity, bits, flow);

	return ret;
}

static struct uart_driver esp32_reg;
static struct console esp32_console = {
	.name		= DEV_NAME,
	.write		= esp32_console_write,
	.device		= uart_console_device,
	.setup		= esp32_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &esp32_reg,
};

static void esp32_earlycon_putchar(struct uart_port *port, unsigned char ch)
{
	esp32_put_char(port, ch);
}

static void esp32_earlycon_write(struct console *con, const char *s,
				 unsigned int n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, esp32_earlycon_putchar);
}

static int __init esp32_early_console_setup(struct earlycon_device *device,
					    const char *options)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = esp32_earlycon_write;
	earlycon_port = &device->port;

	return 0;
}

OF_EARLYCON_DECLARE(esp32uart, "esp,esp32-uart",
		    esp32_early_console_setup);

static struct uart_driver esp32_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= DEV_NAME,
	.nr		= ARRAY_SIZE(esp32_ports),
	.cons		= &esp32_console,
};

static int esp32_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct esp32uart_port *uart;
	struct uart_port *port;
	struct resource *res;
	int ret;

	uart = devm_kzalloc(&pdev->dev, sizeof(*uart), GFP_KERNEL);
	if (!uart)
		return -ENOMEM;

	port = &uart->port;

	ret = of_alias_get_id(np, "serial");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", ret);
		return ret;
	}
	if (ret >= UART_NR) {
		dev_err(&pdev->dev, "driver limited to %d serial ports\n",
			UART_NR);
		return -ENOMEM;
	}

	port->line = ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		return -ENODEV;
	}

	port->mapbase = res->start;
	port->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(port->membase)) {
		return PTR_ERR(port->membase);
	}

	port->dev = &pdev->dev;
	port->type = PORT_ESP32UART;
	port->iotype = UPIO_MEM;
	port->irq = platform_get_irq(pdev, 0);
	port->ops = &esp32_pops;
	port->flags = UPF_BOOT_AUTOCONF;

	esp32_ports[port->line] = uart;

	platform_set_drvdata(pdev, uart);

	ret = uart_add_one_port(&esp32_reg, port);
	if (ret) {
		return ret;
	}

	pr_info(KERN_INFO "esp32_uart at 0x%llx, irq %d\n",
	       (unsigned long long) port->mapbase, port->irq);

	return 0;
}

static int esp32_remove(struct platform_device *pdev)
{
	struct esp32uart_port *eport = platform_get_drvdata(pdev);
	struct uart_port *sport = &eport->port;

	uart_remove_one_port(&esp32_reg, sport);

	return 0;
}


static struct platform_driver esp32_driver = {
	.probe		= esp32_probe,
	.remove		= esp32_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table	= esp32_dt_ids,
		//.pm	= &esp32_pm_ops,
	},
};

static int __init esp32_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&esp32_reg);
	if (ret)
		return ret;

	ret = platform_driver_register(&esp32_driver);
	if (ret)
		uart_unregister_driver(&esp32_reg);

	return ret;
}

static void __exit esp32_serial_exit(void)
{
	platform_driver_unregister(&esp32_driver);
	uart_unregister_driver(&esp32_reg);
}

module_init(esp32_serial_init);
module_exit(esp32_serial_exit);

MODULE_DESCRIPTION("Espressif ESP32 UART driver.");
MODULE_LICENSE("GPL v2");
