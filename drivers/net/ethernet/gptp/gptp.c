#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#define DRIVER_AUTHOR "Indumathi Duraipandian"
#define DRIVER_DESC   "IEEE 802.1AS gPTP driver"

#define G_PTP_TEST_WQ "gPTPTestWQ"

static int __init gptp_init(void);
static void __exit gptp_exit(void);
static void testTaskFn(struct work_struct *work);

static struct workqueue_struct* pTestWQ;
static DECLARE_DELAYED_WORK(testTask, testTaskFn);

static int __init gptp_init(void)
{
	printk(KERN_INFO "gPTP Init");

	pTestWQ = create_workqueue(G_PTP_TEST_WQ);

	INIT_DELAYED_WORK(&testTask, testTaskFn);
	queue_delayed_work(pTestWQ, &testTask, 1000);

	return 0;
}

static void testTaskFn(struct work_struct *work)
{
	printk(KERN_INFO "gPTP test Timer Tick");
	//dmHandleEvent(1);
	queue_delayed_work(pTestWQ, &testTask, 1000);
}

static void __exit gptp_exit(void)
{
	cancel_delayed_work(&testTask);
	printk(KERN_INFO "gPTP Removed");
}

module_init(gptp_init);
module_exit(gptp_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
