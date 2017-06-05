#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/init.h>

#include "delaymsr.h"

#define DRIVER_AUTHOR "Indumathi Duraipandian"
#define DRIVER_DESC "IEEE 802.1AS gPTP driver"

#define G_PTP_TEST_WQ "gPTPTestWQ"

static int __init gptp_init(void);
static void __exit gptp_exit(void);
static void testTaskFn(struct work_struct *work);

static struct workqueue_struct* pTestWQ;
static DECLARE_DELAYED_WORK(testTask, testTaskFn);

static struct dmst dm;

static int __init gptp_init(void)
{
	printk(KERN_INFO "gPTP Init \n");

	initDM(&dm);

	pTestWQ = create_workqueue(G_PTP_TEST_WQ);

	INIT_DELAYED_WORK(&testTask, testTaskFn);
	queue_delayed_work(pTestWQ, &testTask, 1000);

	return 0;
}

static void testTaskFn(struct work_struct *work)
{
	printk(KERN_INFO "gPTP test Timer Tick \n");
	dmHandleEvent(&dm, GPTP_EVT_ONE_SEC_TICK);
	queue_delayed_work(pTestWQ, &testTask, 1000);
}

static void __exit gptp_exit(void)
{
	cancel_delayed_work(&testTask);
	unintDM(&dm);
	printk(KERN_INFO "gPTP Removed \n");
}

module_init(gptp_init);
module_exit(gptp_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
