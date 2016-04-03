/*
 *  cir.c - Driver for the Consumer IR 
 *
 * Copyright (C) 2016 by Soukthavy Sopha
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  $Date: 2016-04-03 10:10:39 -0700 (Sun, 03 Apr 2016) $
 *  $Rev: 70 $
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/serial_reg.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/bitops.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeirq.h>
#include <media/rc-core.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define DRVNAME "cir"


#define RXFIFOE     1   /* RX FIFO EMPTY */

#define RHRIT       1   /* RHR interrupt active */
#define RXOEIT      8

/* Misc */
#define CIR_NAME	"CIR"
#define CIR_ID_FAMILY          0xF1 /* Family ID for the WPCD376I	*/
#define	CIR_ID_CHIP            0x04 /* Chip ID for the WPCD376I	*/
#define INVALID_SCANCODE   0x7FFFFFFF /* Invalid with all protos	*/
#define WAKEUP_IOMEM_LEN         0x10 /* Wake-Up I/O Reg Len		*/
#define EHFUNC_IOMEM_LEN         0x10 /* Enhanced Func I/O Reg Len	*/
#define SP_IOMEM_LEN             0x1000 /* Serial Port 3 (IR) Reg Len	*/

#define RX_FIFO_CLEAR       2

#define PRESCALER_36KHZ       111   /* Phillip RC5, RC6 */
#define PRESCALER_38KHZ       105   /* NEC */
#define PRESCALER_40KHZ       400   /* Sony */

#define UART4_BASE	0x481a8000

#define UART_OMAP_TCR	6
#define UART_TLR	7
#define UART_TXFLL	10
#define UART_TXFLH	11
#define UART1_OMAP_SCR	16

#define UART_OMAP_ACREG 0xf
#define UART_OMAP_CFPS	0x18
#define UART_OMAP_MDR3	0x20

#define DEFAULT_CLK_SPEED 48000000 /* 192MHZ/4 is default PLLS feed*/

#define REG_STRUCT_INIT(r) {#r, r}
/* The name and the corresponding reg number */
#define lcr UART_LCR
#define efr UART_EFR
#define ier UART_IER
#define mcr UART_MCR
#define tlr UART_TLR
#define mdr1 UART_OMAP_MDR1
#define mdr2 UART_OMAP_MDR2
#define txfll UART_TXFLL
#define txflh UART_TXFLH
#define mdr3 UART_OMAP_MDR3
#define cfps UART_OMAP_CFPS
#define acreg UART_OMAP_ACREG
#define fcr UART_FCR
#define eblr UART_OMAP_EBLR
#define sysc UART_OMAP_SYSC
#define syss UART_OMAP_SYSS
#define wer UART_OMAP_WER

#define rhr UART_RX

struct reg {
	char *name;
	unsigned char num;
};
static const struct reg regs[] = {
	REG_STRUCT_INIT(rhr),
	REG_STRUCT_INIT(acreg),
	REG_STRUCT_INIT(lcr),
	REG_STRUCT_INIT(efr),
	REG_STRUCT_INIT(ier),
	REG_STRUCT_INIT(mcr),
	REG_STRUCT_INIT(tlr),
	REG_STRUCT_INIT(mdr1),
	REG_STRUCT_INIT(mdr2),
	REG_STRUCT_INIT(mdr3),
	REG_STRUCT_INIT(cfps),
	REG_STRUCT_INIT(txfll),
	REG_STRUCT_INIT(txflh),
	REG_STRUCT_INIT(fcr),
	REG_STRUCT_INIT(eblr),
	REG_STRUCT_INIT(sysc),
	REG_STRUCT_INIT(syss),
	REG_STRUCT_INIT(wer),
};
struct proc_data {
	struct proc_dir_entry *entry;
	int release_buffer;
	int readlen;
	char *rbuffer;
	int writelen;
	int maxwritelen;
	char *wbuffer;
	void (*on_close) (struct inode *, struct file *);
};

struct ir_data {
	struct list_head list;
	u8 *buf;
	int len;
};
/* Per-device data */
struct cir_dev {
	spinlock_t spinlock;

	void __iomem *sbase;        /* Serial Port Baseaddr	*/
	unsigned int  irq;          /* Serial Port IRQ		*/
	int wakeirq;
	unsigned int uartclk;
	resource_size_t		mapbase;		/* for ioremap */
	resource_size_t		mapsize;

	struct input_dev *inputdev;
	struct rc_dev *irprops;
	struct work_struct bh;
	struct   device *dev;
	struct ir_data head;

	/* RX irdata state */
	bool irdata_active;
	bool irdata_error, open;
	u32 pulse_duration;
	u32	carrier_freq_hz;
	u32	clk_freq_hz;
	struct clk *clk;
	struct proc_dir_entry *proc_entry;
	struct proc_data *procdata;
};
static struct proc_dir_entry *cir_procdir_entry; /* root entry */
static struct cir_dev *cirdev = NULL;
static unsigned int dll,dlh;
static int protocol = RC_TYPE_NEC;
module_param(protocol, uint, 0444);
MODULE_PARM_DESC(protocol, "IR protocol to use "
		 "(1 = RC5, 2 = NEC, 4 = RC6A, 16=SONY, default)");

static bool invert; /* default = 0 */
module_param(invert, bool, S_IRUGO);
MODULE_PARM_DESC(invert, "Invert the signal from the IR receiver");

struct config_data {
	u32 offset;
	u16 data;
};
#define IOSHIFT 2

static struct config_data _configure_data[] = {
	/* The configuration data of UART registers for CIR mode.*/
	{UART_LCR << IOSHIFT , 0xbf}, /* disable interrupt */
	{UART_EFR << IOSHIFT, 0x10},
	{UART_LCR << IOSHIFT, 0},
	{UART_IER << IOSHIFT, 0},

	{UART_MCR << IOSHIFT, 0},
	{UART_LCR << IOSHIFT, 0xbf},
	{UART_EFR << IOSHIFT, 0},
	{UART_LCR << IOSHIFT, 0},
	{UART1_OMAP_SCR << IOSHIFT, 0x1},
	{UART_LCR << IOSHIFT, 0xbf},
	{UART_EFR << IOSHIFT, 0x10},
	{UART_LCR << IOSHIFT, 0},
	{UART_MCR << IOSHIFT, 0x40},
	{UART_OMAP_TCR << IOSHIFT, 0x8a},
	{UART_TLR << IOSHIFT, 0x10},
	{UART_MCR << IOSHIFT, 0},
	{UART_LCR << IOSHIFT, 0xbf},
	{UART_EFR << IOSHIFT, 0},
	{UART_LCR << IOSHIFT, 0},
	{UART_OMAP_MDR1 << IOSHIFT, 0x7},
	{UART_FCR << IOSHIFT, 0x57},
	{UART_LCR << IOSHIFT, 0x2},
	{UART_MCR << IOSHIFT, 0x3f},
	{UART_LCR << IOSHIFT, 0xbf},
	{UART_EFR << IOSHIFT, 0},
	{UART_LCR << IOSHIFT, 0},
	{UART_OMAP_MDR1 << IOSHIFT, 0x27},
	{UART_OMAP_MDR2 << IOSHIFT, 0x10},
	{UART_LCR << IOSHIFT, 0xbf},
	{UART_TXFLL << IOSHIFT, 0x4},
	{UART_TXFLH << IOSHIFT, 0},
};
#define ITE_BAUDRATE_DIVISOR		1

static void *seq_reg_start(struct seq_file *seq, loff_t *pos)
{
	return *pos < ARRAY_SIZE(regs) ? (void *)&regs[*pos]: NULL;

}
static void *seq_reg_next(struct seq_file *seq, void *p, loff_t *pos)
{
	++*pos;
	return *pos < ARRAY_SIZE(regs) ? (void *)&regs[*pos]: NULL;
}
static int seq_reg_show(struct seq_file *seq, void *p)
{
	struct reg *r = p;

	seq_printf(seq, "%-25s0x%02x\n", r->name,readw(cirdev->sbase + (r->num << IOSHIFT)));
	return 0;
}
static void seq_reg_stop(struct seq_file *seq, void *p) 
{
}

/* decode raw bytes as received by the hardware, and push them to the rc-core
 * layer */

static void irevent_bh(struct work_struct *bhptr)
{
	struct cir_dev *cirdev = container_of(bhptr, struct cir_dev,bh);
	u8 pulse;
	int i,j;
	struct list_head *list = (struct list_head *)&cirdev->head;
	struct ir_data *dlist;
	u8 *buf;

	DEFINE_IR_RAW_EVENT(ev);

	spin_lock(&cirdev->spinlock);
	while ( list->next != (struct list_head *)&cirdev->head ) {
		dlist = (struct ir_data *)list->next;
		buf = dlist->buf;

		for (i =0; i< dlist->len; i++) {
			pulse = *buf++;

			if ( (pulse == 0) || (pulse == 0xff) ) {
				ev.pulse = pulse ? 1 : 0 ;
				ev.duration = (protocol == RC_TYPE_SONY12 ) ? 600000 * 8: 562500 * 8 ;
				ir_raw_event_store_with_filter(cirdev->irprops, &ev);
				continue;
			}

			for (j = 0; j < 8; j++) {
				ev.pulse = pulse & 1;
				pulse = pulse >> 1;
				if (protocol == RC_TYPE_SONY12) {
					ev.duration = 600000 ; //duration * 1000; //in ns
				}
				else
					ev.duration = 562500; //duration * 1000; //in ns
				ir_raw_event_store_with_filter(cirdev->irprops, &ev);
			}
		}
		ir_raw_event_handle(cirdev->irprops);
		kfree(dlist->buf);
		list_del(&dlist->list);
		kfree(dlist);
		cirdev->irdata_active = false;
		continue;
	}
	ir_raw_event_set_idle(cirdev->irprops,true);
	spin_unlock(&cirdev->spinlock);
}

static irqreturn_t cir_irq_handler(int irqno, void *red_data)
{
	struct cir_dev *cirdev = (struct cir_dev *)red_data;
	unsigned long flags;
	u16 status;
	int i = 0;
	struct ir_data *plist;

	spin_lock_irqsave(&cirdev->spinlock, flags);

	status = readw(cirdev->sbase + (UART_IIR << IOSHIFT));

	if (!(status & (0x2f))) {
		spin_unlock_irqrestore(&cirdev->spinlock, flags);
		return IRQ_NONE;
	}
	writew(0,cirdev->sbase + (UART_IER << IOSHIFT));

	/* Check for rx overrun */
	if (status & RXOEIT) {
		cirdev->irdata_error = true;
        	printk(KERN_INFO "RX overrun\n");
		ir_raw_event_reset(cirdev->irprops);
	}

	if (!cirdev->irdata_active) {
		cirdev->irdata_active = true;
	}
	/* allocat buffer */
	if ( (plist = (struct ir_data *)kmalloc(sizeof(struct ir_data),GFP_KERNEL)) ) {
		if ( (plist->buf = (u8 *)kmalloc(64,GFP_KERNEL)) ) {
			u8 *buf = plist->buf;
			i = 0;

			while( !(readw(cirdev->sbase + (UART_LSR << IOSHIFT)) & RXFIFOE) ) {
				*buf++ = (u8 )readw(cirdev->sbase + (UART_RX << IOSHIFT) ) & 0xff;
				i++;
			}
			plist->len = i;

			list_add_tail(&plist->list,(struct list_head *)&cirdev->head);
		}
	}

	writew(0xc,cirdev->sbase + (UART_IER << IOSHIFT));

	spin_unlock_irqrestore(&cirdev->spinlock, flags);
	schedule_work(&cirdev->bh);
	return IRQ_HANDLED;
}

static void  cir_init_hw(struct cir_dev *cirdev)
{
	int i;
	u32 divisor = 1667;

	if (cirdev->wakeirq) {
		if ( (i = dev_pm_set_dedicated_wake_irq(cirdev->dev,cirdev->wakeirq)) )
			return ;
	}

	/* Disable interrupts */
	writew(0,cirdev->sbase + (UART_IER << IOSHIFT));

	switch (protocol) {
	case RC_TYPE_RC5:
		divisor  = 1778;
		break;
	case RC_TYPE_NEC:
		divisor  = 1778;
		writew(PRESCALER_38KHZ,cirdev->sbase + (UART_OMAP_CFPS << IOSHIFT));
		break;
	case RC_TYPE_SONY12:
		/* formula:
		   DLL,DLH = (Fclk/16)/Tfreq = (48/16)/Tfreq, all in Mhz
		   For the require T = 600us -> Tfreq = 1/600us = 1666.67.
		   DLL,DLH = 3Mhz/1666.67 =1800 = 0x708. 
		dll = 8; //0x90;
		dlh = 7; //0x6; //0x708 base on 40Khz
		*/
		divisor  = 1667;
		writew(PRESCALER_40KHZ,cirdev->sbase + (UART_OMAP_CFPS << IOSHIFT));
		break;
	}

	divisor = (cirdev->clk_freq_hz >> 4)/divisor;
	dll = divisor & 0xff;
	dlh = (divisor >> 8) & 0xff;

	/* configure CIR mode */
	for (i = 0; i < sizeof(_configure_data)/sizeof(struct config_data) ; i++ ) {
		writew(_configure_data[i].data,cirdev->sbase + _configure_data[i].offset);
	}
	writew(dll,cirdev->sbase + (UART_DLL << IOSHIFT));
	writew(dlh,cirdev->sbase + (UART_DLM << IOSHIFT));
	writew(0,cirdev->sbase + (UART_LCR << IOSHIFT));
	writew(9*4,cirdev->sbase + (UART_OMAP_EBLR << IOSHIFT));
	writew(0x10,cirdev->sbase + (UART_OMAP_ACREG << IOSHIFT));
	writew(1,cirdev->sbase + (UART_OMAP_MDR3 << IOSHIFT));    /* disable modulation. nothing received if enable !*/
	writew(0x26,cirdev->sbase + (UART_OMAP_MDR1 << IOSHIFT)); /* CIR mode */
	writew(0x57,cirdev->sbase + (UART_FCR << IOSHIFT));

	/* Clear IR decoding state */
	cirdev->irdata_active = false;
	
	cirdev->irdata_error = false;
	ir_raw_event_set_idle(cirdev->irprops,true);

	/* Enable interrupts */
	writew(0xbf, cirdev->sbase + (UART_LCR << IOSHIFT));
	writew(0x10, cirdev->sbase + (UART_EFR << IOSHIFT));
	writew(0, cirdev->sbase + (UART_LCR << IOSHIFT));
	writew(0xc, cirdev->sbase + (UART_IER << IOSHIFT));

}
static void cir_set_idle(struct rc_dev *rcdev, bool idle)
{
}
static int cir_open(struct rc_dev *rcdev)
{
	struct cir_dev *cirdev = rcdev->priv;

	cir_init_hw(cirdev);
	return 0;
}
static void cir_close(struct rc_dev *rcdev)
{
}
#ifdef DEBUG_ON
static void  get_uart_clock(struct cir_dev *cirdev)
{
    void __iomem *port;

    port = (void __iomem *)ioremap_nocache(0x44e00000,0x100); /* clksrc for uart3-5*/
    printk("%s, CM_PER_L4LS_CLKCTRL=0x%x\n", __FUNCTION__,readl(port ));
    printk("%s, CM_PER_UART4_CLKCTRL=0x%x\n", __FUNCTION__,readl(port+0x78));
    printk("%s: MDR1 reg = 0x%x, CFPS 0x%x\n",__FUNCTION__,readw(cirdev->sbase + 0x20 ),readw(cirdev->sbase+0x60));
    iounmap((void __iomem *)port);
}
#endif
static const struct of_device_id cir_dt_ids[] = {
	{ .compatible = "cir-uart" },
	{},
};

MODULE_DEVICE_TABLE(of, cir_dt_ids);

static struct seq_operations proc_seq_fops = {
	.start = seq_reg_start,
	.stop  = seq_reg_stop,
	.next = seq_reg_next,
	.show = seq_reg_show,
};

static int proc_reg_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offp)
{
	return 0;
}
static int proc_seq_open(struct inode *inode, struct file *filp)
{
	struct proc_data *data;
	int res = 0;
	struct seq_file *seq;

	if ( (data = (struct proc_data *)kzalloc(sizeof(struct proc_data), GFP_KERNEL)) == NULL )
		return -ENOMEM;
	cirdev->procdata = data;

	res = seq_open(filp,&proc_seq_fops);
	if (res == 0) {
		seq = filp->private_data;
		seq->private = cirdev;
	}

	return res;
}

struct file_operations proc_regs_fops = {
	.open =  proc_seq_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = proc_reg_write,
	.owner = THIS_MODULE,

};


static int cir_probe(struct platform_device *pdev)
{
	int err=0;
	struct rc_dev *ir_props;
	struct resource *regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct resource *irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	struct clk *clk;
	struct proc_dir_entry *entry;
#ifdef DEBUG_ON
	printk("%s: entering with regs start 0x%x, size 0x%x, irq %d\n",__FUNCTION__,regs->start,resource_size(regs),irq->start);
#endif	
	if (!regs || !irq) {
		dev_err(&pdev->dev, "missing registers or irq\n");
		return -EINVAL;
	}
	cirdev = devm_kzalloc(&pdev->dev, sizeof(struct cir_dev), GFP_KERNEL);
	if (!cirdev) {
		err = -ENOMEM;
		goto exit;
	}
	if ( !(cirdev->sbase = devm_ioremap_nocache(&pdev->dev, regs->start,
				       resource_size(regs))) )
	{
		kfree(cirdev);
		return -ENODEV;
	}
	cirdev->mapbase = regs->start;
	cirdev->irq = irq->start;
	cirdev->dev = &pdev->dev;

	if (pdev->dev.of_node) {
		const struct of_device_id *id;

		err = of_alias_get_id(pdev->dev.of_node, "serial");

		of_property_read_u32(pdev->dev.of_node, "clock-frequency",
				     &cirdev->uartclk);
		cirdev->wakeirq = irq_of_parse_and_map(pdev->dev.of_node, 1);

		id = of_match_device(of_match_ptr(cir_dt_ids), &pdev->dev);
#ifdef DEBUG_ON
		printk("%s:uartclk %d, wakeirq %d, id %p, sbase 0x%lx, mapbase %lx\n",
				__FUNCTION__,cirdev->uartclk,cirdev->wakeirq,
				id->data,(unsigned long)cirdev->sbase,(unsigned long)cirdev->mapbase);
#endif		
	} else {
		err = pdev->id;
	}
	if (err < 0) {
		printk("%s: failed to get alias/pdev \n",__FUNCTION__);
		dev_err(&pdev->dev, "failed to get alias/pdev id\n");
		iounmap((void __iomem *)cirdev->sbase);
		kfree(cirdev);
		return err;
	}
	if (!cirdev->uartclk) {
		cirdev->uartclk = DEFAULT_CLK_SPEED;
		dev_warn(&pdev->dev,
			 "No clock speed specified: using default: %d\n",
			 DEFAULT_CLK_SPEED);
	}

	platform_set_drvdata(pdev,cirdev);

	/* use uart4_fck */
	clk = clk_get(&pdev->dev,"dpll_per_m2_div4_ck"); //expect 48MHZ clock feed 
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to fetch clock block");
		iounmap(cirdev->sbase);
		kfree(cirdev);
		return PTR_ERR(clk);
	}
	device_init_wakeup(&pdev->dev, true);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev,-1);
	pm_runtime_irq_safe(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	pm_runtime_get_sync(&pdev->dev);

	cirdev->clk = clk;
	clk_prepare_enable(clk);
#ifdef DEBUG_ON
	printk("%s: fck rate %ld\n",__FUNCTION__,clk_get_rate(clk));
#endif
	if (!(cir_procdir_entry = proc_mkdir("cir",NULL))) {
		goto exit_free_data;
	}
	cirdev->proc_entry = cir_procdir_entry;
	if (!(entry = proc_create_data("regs",S_IFREG | S_IRUGO | S_IWUSR,
								   cirdev->proc_entry,
								   &proc_regs_fops,
								   NULL)) ) {
		printk("%s:failed at %d\n",__FUNCTION__,__LINE__);
		remove_proc_entry("regs",cirdev->proc_entry);
		goto exit_free_data;
	}
	spin_lock_init(&cirdev->spinlock);
#ifdef DEBUG_ON
	get_uart_clock(cirdev);
#endif
	if (!request_mem_region((u32)cirdev->sbase, SP_IOMEM_LEN, DRVNAME)) {
		printk(KERN_INFO "Region %p to %p already in use!\n",
			cirdev->sbase, cirdev->sbase + SP_IOMEM_LEN - 1);
		err = -EBUSY;
		remove_proc_entry("regs",cirdev->proc_entry);
		goto exit_free_data;
	}

	cirdev->clk_freq_hz = (cirdev->uartclk ? cirdev->uartclk: 48000000);

	err = request_irq(cirdev->irq, cir_irq_handler,
			  IRQF_SHARED, DRVNAME, (void *)cirdev);
	if (err) {
		printk(KERN_INFO "Failed to claim IRQ %u\n", cirdev->irq);
		err = -EBUSY;
		goto exit_release_sbase;
	}
	writew(2, cirdev->sbase + (UART_OMAP_SYSC << IOSHIFT)); //soft reset
	udelay(10);


	switch (protocol) {
	case RC_TYPE_SONY12:
		cirdev->carrier_freq_hz = 40000;
		break;
	case RC_TYPE_NEC:
	default:
		cirdev->carrier_freq_hz = 38000;
		break;
	}

	ir_props = rc_allocate_device();
	if (ir_props == NULL) {
		printk(KERN_INFO "Failed to allocate rc_dev !\n");
		err = -ENOMEM;
		goto exit_free_input;
	}

	ir_props->driver_type = RC_DRIVER_IR_RAW;
	ir_props->allowed_protocols = RC_BIT_ALL;
	ir_props->priv = (void *)cirdev;
 	ir_props->s_idle = cir_set_idle;
	ir_props->open = cir_open;
	ir_props->close = cir_close;

	ir_props->map_name = RC_MAP_RC6_MCE;
	ir_props->input_name = "CIR Infrared Remote Receiver";
	cirdev->irprops = ir_props;
	
	INIT_WORK((struct work_struct *)&cirdev->bh,irevent_bh);
	INIT_LIST_HEAD((struct list_head *)&cirdev->head);

	err = rc_register_device(cirdev->irprops);
	if (err)
		goto exit_free_ir;

	/* omit for simplicity */
	//pm_runtime_mark_last_busy(&pdev->dev);
	//pm_runtime_put_autosuspend(&pdev->dev);

	return 0;
exit_free_ir:
	kfree(ir_props);
exit_free_input:
	free_irq(cirdev->irq, cirdev);
exit_release_sbase:
	release_mem_region((u32)cirdev->sbase,SP_IOMEM_LEN);
exit_free_data:
	clk_disable_unprepare(clk);
	iounmap((void __iomem *)cirdev->sbase);
	kfree(cirdev);
    
exit:
	return err;
}

static int  cir_remove(struct platform_device *pdev)
{
	unsigned long flags;
	struct cir_dev *cirdev = platform_get_drvdata(pdev);

	spin_lock_irqsave(&cirdev->spinlock, flags);
	writew(0,cirdev->sbase + (UART_IER  << IOSHIFT));
	/* Disable interrupts */
	writew(0xbf, cirdev->sbase + (UART_LCR << IOSHIFT));
	writew(0x10, cirdev->sbase + (UART_EFR << IOSHIFT));
	writew(0, cirdev->sbase + (UART_LCR << IOSHIFT));
	writew(0, cirdev->sbase + (UART_IER << IOSHIFT));
	spin_unlock_irqrestore(&cirdev->spinlock, flags);

	pm_runtime_put_sync(cirdev->dev);
	pm_runtime_disable(cirdev->dev);

	flush_scheduled_work();
	cancel_work_sync(&cirdev->bh);
#ifdef DEBGUG_ON
	printk("%s freeing irq and mem. idev %p\n",__FUNCTION__,cirdev->inputdev);
#endif	
	rc_unregister_device(cirdev->irprops);
	device_init_wakeup(&pdev->dev, false);
	clk_disable_unprepare(cirdev->clk);
	free_irq(cirdev->irq, cirdev);
	devm_iounmap(&pdev->dev,cirdev->sbase);
	release_mem_region((u32)cirdev->sbase, SP_IOMEM_LEN);
	remove_proc_entry("regs",cirdev->proc_entry);
	remove_proc_entry("cir",NULL);
	kfree(cirdev->procdata);
	kfree(cirdev->irprops);
	kfree(cirdev);
	return 0;
}

static struct platform_driver cir_platform_driver = {
	.driver = {
		.name		= "cir-uart",
		.of_match_table = cir_dt_ids,
	},
	.probe			= cir_probe,
	.remove			= cir_remove,
};


static int __init cir_init(void) 
{
	return platform_driver_register(&cir_platform_driver);
}

static void __exit cir_exit(void)
{
	platform_driver_unregister(&cir_platform_driver);
}
module_init(cir_init);
module_exit(cir_exit);

MODULE_AUTHOR("soukthavy.sopha@yahoo.com");
MODULE_DESCRIPTION("Consumer IR Driver");
MODULE_LICENSE("GPL");
