/*
 *  AVB soundcard
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/info.h>
#include <sound/initval.h>

#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/in.h>
#include <linux/delay.h>
#include <linux/un.h>
#include <linux/unistd.h>
#include <linux/ctype.h>

#include <asm/unistd.h>

#include <net/sock.h>
#include <net/tcp.h>
#include <net/inet_connection_sock.h>
#include <net/request_sock.h>

#include "avb_types.h"

MODULE_AUTHOR("Indumathi Duraipandian <indu9086@gmail.com>");
MODULE_DESCRIPTION("AVB soundcard");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,AVB soundcard}}");

static int index[SND_AVB_NUM_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SND_AVB_NUM_CARDS] = SNDRV_DEFAULT_STR;
static bool enable[SND_AVB_NUM_CARDS] = {1, [1 ... (SND_AVB_NUM_CARDS - 1)] = 0};
static int pcm_substreams[SND_AVB_NUM_CARDS] = {1};
static int pcm_notify[SND_AVB_NUM_CARDS];

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for avb soundcard.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for avb soundcard.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable this avb soundcard.");
module_param_array(pcm_substreams, int, NULL, 0444);
MODULE_PARM_DESC(pcm_substreams, "PCM substreams # (1-8) for avb driver.");
module_param_array(pcm_notify, int, NULL, 0444);
MODULE_PARM_DESC(pcm_notify, "Break capture when PCM format/rate/channels changes.");

static struct avbdevice avbdevice;
static int numcards = 0;
static struct platform_device *avbdevices[SND_AVB_NUM_CARDS];

static SIMPLE_DEV_PM_OPS(avb_pm, avb_suspend, avb_resume);
static struct platform_driver avb_driver = {
	.probe		= avb_probe,
	.remove		= avb_remove,
	.driver		= {
		.name	= SND_AVB_DRIVER,
		.pm	= AVB_PM_OPS,
	},
};

static struct snd_pcm_ops avb_playback_ops = {
	.open =		avb_open,
	.close =	avb_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	avb_hw_params,
	.hw_free =	avb_hw_free,
	.prepare =	avb_prepare,
	.trigger =	avb_trigger,
	.pointer =	avb_pointer,
	.page =		snd_pcm_lib_get_vmalloc_page,
	.mmap =		snd_pcm_lib_mmap_vmalloc,
};

static struct snd_pcm_ops avb_capture_ops = {
	.open =		avb_open,
	.close =	avb_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	avb_hw_params,
	.hw_free =	avb_hw_free,
	.prepare =	avb_prepare,
	.trigger =	avb_trigger,
	.pointer =	avb_pointer,
	.page =		snd_pcm_lib_get_vmalloc_page,
	.mmap =		snd_pcm_lib_mmap_vmalloc,
};


static int avb_open(struct snd_pcm_substream *substream)
{
	return 0;
}

static int avb_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static int avb_trigger(struct snd_pcm_substream *substream, int cmd)
{
	return 0;
}

static snd_pcm_uframes_t avb_pointer(struct snd_pcm_substream *substream)
{
	return 0;
}

static int avb_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	return 0;
}

static int avb_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}

static int avb_close(struct snd_pcm_substream *substream)
{
	return 0;
}

static bool avb_msrp_init(struct msrp* msrp)
{
	printk(AVB_KERN_INFO "avb_msrp_init");

	if (sock_create(AF_PACKET, SOCK_RAW, htons(ETH_MSRP), &msrp->sock) < 0) {
		printk(AVB_KERN_ERR "avb_msrp_init Socket creation fails \n");
		return false;
	}

	memset(&msrp->if_idx, 0, sizeof(struct ifreq));
	strncpy(msrp->if_idx.ifr_name, "eth0", sizeof(msrp->if_idx.ifr_name) - 1);
	if (msrp->sock->ops->ioctl(msrp->sock, SIOCGIFINDEX, (unsigned int)&msrp->if_idx) < 0) {
		printk(AVB_KERN_ERR "avb_msrp_init SIOCGIFINDEX err");
		return false;
	}

	memset(&msrp->if_mac, 0, sizeof(struct ifreq));
	strncpy(msrp->if_mac.ifr_name, "eth0", sizeof(msrp->if_mac.ifr_name) - 1);
	if (msrp->sock->ops->ioctl(msrp->sock, SIOCGIFHWADDR, (unsigned int)&msrp->if_mac) < 0) {
		printk(AVB_KERN_ERR "avb_msrp_init SIOCGIFHWADDR err");
		return false;
	}

	/* Index of the network device */
	msrp->txSockAddress.sll_family = AF_PACKET;
	msrp->txSockAddress.sll_protocol = htons(ETH_MSRP);
	msrp->txSockAddress.sll_ifindex = msrp->if_idx.ifr_ifindex;
	/* Address length*/
	msrp->txSockAddress.sll_halen = ETH_ALEN;
	/* Destination MAC */
	msrp->txSockAddress.sll_addr[0] = 0x01;
	msrp->txSockAddress.sll_addr[1] = 0x80;
	msrp->txSockAddress.sll_addr[2] = 0xC2;
	msrp->txSockAddress.sll_addr[3] = 0x00;
	msrp->txSockAddress.sll_addr[4] = 0x00;
	msrp->txSockAddress.sll_addr[5] = 0x0E;

	/* Set the message header */
	msrp->txMsgHdr.msg_control=NULL;
	msrp->txMsgHdr.msg_controllen=0;
	msrp->txMsgHdr.msg_flags=0;
	msrp->txMsgHdr.msg_name=&msrp->txSockAddress;
	msrp->txMsgHdr.msg_namelen=sizeof(struct sockaddr_ll);
	msrp->txMsgHdr.msg_iocb = NULL;

	/* Index of the network device */
	msrp->rxSockAddress.sll_family = AF_PACKET;
	msrp->rxSockAddress.sll_protocol = htons(ETH_MSRP);
	msrp->rxSockAddress.sll_ifindex = msrp->if_idx.ifr_ifindex;
	/* Address length*/
	msrp->rxSockAddress.sll_halen = ETH_ALEN;
	/* Destination MAC */
	msrp->rxSockAddress.sll_addr[0] = 0x01;
	msrp->rxSockAddress.sll_addr[1] = 0x80;
	msrp->rxSockAddress.sll_addr[2] = 0xC2;
	msrp->rxSockAddress.sll_addr[3] = 0x00;
	msrp->rxSockAddress.sll_addr[4] = 0x00;
	msrp->rxSockAddress.sll_addr[5] = 0x0E;

	/* Set the message header */
	msrp->rxMsgHdr.msg_control=NULL;
	msrp->rxMsgHdr.msg_controllen=0;
	msrp->rxMsgHdr.msg_flags=0;
	msrp->rxMsgHdr.msg_name=&msrp->rxSockAddress;
	msrp->rxMsgHdr.msg_namelen=sizeof(struct sockaddr_ll);
	msrp->rxMsgHdr.msg_iocb = NULL;
	msrp->rxiov.iov_base = msrp->rxBuf;
	msrp->rxiov.iov_len = AVB_MSRP_ETH_FRAME_SIZE;
	iov_iter_init(&msrp->rxMsgHdr.msg_iter, READ | ITER_KVEC, &msrp->rxiov, 1, AVB_MSRP_ETH_FRAME_SIZE);

	return true;
}

static void avb_msrp_join(struct msrp* msrp)
{
	int txSize = 0;
	int err = 0;
	struct ethhdr *eh = (struct ethhdr *)&msrp->txBuf[0];
	struct msrpdu *pdu = (struct msrpdu*)&msrp->txBuf[sizeof(struct ethhdr)];

	printk(AVB_KERN_INFO "avb_msrp_join");

	/* Initialize it */
	memset(msrp->txBuf, 0, AVB_MSRP_ETH_FRAME_SIZE);

	/* Fill in the Ethernet header */
	eh->h_dest[0] = 0x01;
	eh->h_dest[1] = 0x80;
	eh->h_dest[2] = 0xC2;
	eh->h_dest[3] = 0x00;
	eh->h_dest[4] = 0x00;
	eh->h_dest[5] = 0x0E;
	eh->h_source[0] = ((u8 *)&msrp->if_mac.ifr_hwaddr.sa_data)[0];
	eh->h_source[1] = ((u8 *)&msrp->if_mac.ifr_hwaddr.sa_data)[1];
	eh->h_source[2] = ((u8 *)&msrp->if_mac.ifr_hwaddr.sa_data)[2];
	eh->h_source[3] = ((u8 *)&msrp->if_mac.ifr_hwaddr.sa_data)[3];
	eh->h_source[4] = ((u8 *)&msrp->if_mac.ifr_hwaddr.sa_data)[4];
	eh->h_source[5] = ((u8 *)&msrp->if_mac.ifr_hwaddr.sa_data)[5];

	/* Fill in Ethertype field */
	eh->h_proto = htons(ETH_P_1588);

	pdu->protocolversion = 0;
	pdu->msg.attributetype = MSRP_ATTRIBUTE_TYPE_TALKER_ADVERTISE_VECTOR;
	pdu->msg.attributelen  = MSRP_ATTRIBUTE_LEN_TALKER_ADVERTISE_VECTOR;
	pdu->msg.attributelistlen = sizeof(struct vectorattribute);

	pdu->msg.attibutelist.hdr.numberofvalues = 1;
	pdu->msg.attibutelist.val.streamid[0] = ((u8 *)&msrp->if_mac.ifr_hwaddr.sa_data)[0];
	pdu->msg.attibutelist.val.streamid[1] = ((u8 *)&msrp->if_mac.ifr_hwaddr.sa_data)[1];
	pdu->msg.attibutelist.val.streamid[2] = ((u8 *)&msrp->if_mac.ifr_hwaddr.sa_data)[2];
	pdu->msg.attibutelist.val.streamid[3] = ((u8 *)&msrp->if_mac.ifr_hwaddr.sa_data)[3];
	pdu->msg.attibutelist.val.streamid[4] = ((u8 *)&msrp->if_mac.ifr_hwaddr.sa_data)[4];
	pdu->msg.attibutelist.val.streamid[5] = ((u8 *)&msrp->if_mac.ifr_hwaddr.sa_data)[5];
	pdu->msg.attibutelist.val.streamid[6] = 0;
	pdu->msg.attibutelist.val.streamid[7] = 1;

	pdu->msg.endmarker = 0;

	txSize = sizeof(struct ethhdr) + sizeof(struct msrpdu);

	msrp->txiov.iov_base = msrp->txBuf;
	msrp->txiov.iov_len = txSize;
	iov_iter_init(&msrp->txMsgHdr.msg_iter, WRITE | ITER_KVEC, &msrp->txiov, 1, txSize);

	if ((err = sock_sendmsg(msrp->sock, &msrp->txMsgHdr)) <= 0) {
		printk(KERN_WARNING "avb_msrp_join Socket transmission fails %d \n", err);
		return;
	}
}

static void avb_msrp_listen(struct msrp* msrp)
{
	int err = 0;

	printk(AVB_KERN_INFO "avb_msrp_listen");

	if ((err = sock_recvmsg(msrp->sock, &msrp->rxMsgHdr, AVB_MSRP_ETH_FRAME_SIZE, 0)) > 0) {
	}
}

static void avbWqFn(struct work_struct *work)
{
	struct workdata* wd = (struct workdata*)work;

	printk(AVB_KERN_INFO "avbWqFn");

	if(wd->msrp->initialized == false) {
		wd->msrp->initialized = avb_msrp_init(wd->msrp);

		if(wd->msrp->initialized == false) {
			queue_delayed_work(avbdevice.wq, (struct delayed_work*)avbdevice.wd, 10000);
		}
	} else {
		avb_msrp_join(wd->msrp);
		avb_msrp_listen(wd->msrp);	
	}
}


static int avb_pcm_new(struct avbcard *avbc, int device, int substreams)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(avbc->card, "AVB PCM", device,
			  substreams, substreams, &pcm);
	if (err < 0)
		return err;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &avb_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &avb_capture_ops);

	pcm->private_data = avbc;
	pcm->info_flags = 0;
	strcpy(pcm->name, "AVB PCM");

	avbc->pcm[device] = pcm;
	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int avb_suspend(struct device *pdev)
{
	printk(AVB_KERN_INFO "avb_suspend");
	return 0;
}
	
static int avb_resume(struct device *pdev)
{
	printk(AVB_KERN_INFO "avb_resume");
	return 0;
}

#endif

static int avb_probe(struct platform_device *devptr)
{
	struct snd_card *card;
	struct avbcard *avbcard;
	int dev = devptr->id;
	int err;

	printk(AVB_KERN_INFO "avb_probe");

	err = snd_card_new(&devptr->dev, index[dev], id[dev], THIS_MODULE,
			   sizeof(struct avbcard), &card);

	if (err < 0) {
		printk(AVB_KERN_INFO "avb_probe card new err: %d", err);
		return err;
	}

	avbcard = card->private_data;
	avbcard->card = card;

	err = avb_pcm_new(avbcard, 0, pcm_substreams[dev]);
	if (err < 0) {
		printk(AVB_KERN_INFO "avb_probe card pcm new err: %d", err);
		goto __nodev;
	}

	strcpy(card->driver, "avb");
	strcpy(card->shortname, "avb");
	sprintf(card->longname, "avb %i", dev + 1);
	err = snd_card_register(card);
	if (!err) {
		platform_set_drvdata(devptr, card);
		return 0;
	}

	printk(AVB_KERN_INFO "avb_probe card reg err: %d", err);

__nodev:
	snd_card_free(card);

	return err;
}

static int avb_remove(struct platform_device *devptr)
{
	printk(AVB_KERN_INFO "avb_remove");
	snd_card_free(platform_get_drvdata(devptr));
	return 0;
}

static void avb_remove_all(void) {
	int i = 0;

	printk(AVB_KERN_INFO "avb_remove_all");

	for(i=0; i < numcards; i++)
		platform_device_unregister(avbdevices[i]);
}											

static int __init alsa_avb_init(void)
{
	int i, err;
	struct platform_device *dev;
	printk(AVB_KERN_INFO "alsa_avb_init");

	err = platform_driver_register(&avb_driver);
	if (err < 0) {
		printk(AVB_KERN_ERR "alsa_avb_init reg err %d", err);
		return err;
	}

	for(i=0; i < SND_AVB_NUM_CARDS; i++) {
		if(!enable[i])
			continue;

		dev = platform_device_register_simple(SND_AVB_DRIVER, i, NULL, 0);

		if (IS_ERR(dev)) {		
			printk(AVB_KERN_ERR "alsa_avb_init regsimple err");
			continue;
		}

		if (!platform_get_drvdata(dev)) {
			printk(AVB_KERN_ERR "alsa_avb_init getdrvdata err");
			platform_device_unregister(dev);
			continue;
		}

		avbdevices[i] = dev;
		numcards++;
	}

	if(!numcards) {
		avb_remove_all();
	} else {
		memset(&avbdevice, 0, sizeof(struct avbdevice));

		avbdevice.wq = create_workqueue(AVB_WQ);
		if(avbdevice.wq == NULL) {
			printk(AVB_KERN_ERR "alsa_avb_init workqueue creation failed");
			return -1;
		}

		avbdevice.wd = (struct workdata*)kmalloc(sizeof(struct workdata), GFP_KERNEL);
		if(avbdevice.wd == NULL) {
			printk(AVB_KERN_ERR "alsa_avb_init workq allocation failed");
			return -1;
		}

		avbdevice.wd->msrp = &avbdevice.msrp;
		INIT_DELAYED_WORK((struct delayed_work*)avbdevice.wd, avbWqFn);
				
		queue_delayed_work(avbdevice.wq, (struct delayed_work*)avbdevice.wd, 100);

		printk(AVB_KERN_INFO "alsa_avb_init done err: %d, numcards: %d", err, numcards);	
	}

	return 0;
}

static void __exit alsa_avb_exit(void)
{
	printk(AVB_KERN_INFO "alsa_avb_exit");
	
	if(avbdevice.wd != NULL) {
		cancel_delayed_work((struct delayed_work*)avbdevice.wd);
		kfree(avbdevice.wd);
	}

	if(avbdevice.wq != NULL) {
		flush_workqueue(avbdevice.wq);
		destroy_workqueue(avbdevice.wq);
	}

	avb_remove_all();

	platform_driver_unregister(&avb_driver);
	
	printk(AVB_KERN_INFO "alsa_avb_exit done");
}

module_init(alsa_avb_init)
module_exit(alsa_avb_exit)
