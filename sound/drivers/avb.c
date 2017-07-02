/*
 *  AVB soundcard
 *
 */

#include <linux/init.h>
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

MODULE_AUTHOR("Indumathi Duraipandian <indu9086@gmail.com>");
MODULE_DESCRIPTION("AVB soundcard");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,AVB soundcard}}");

#define SND_AVB_DRIVER		"snd_avb"

#if 1
#define SND_AVB_NUM_CARDS	(SNDRV_CARDS)
#else
#define SND_AVB_NUM_CARDS	(1)
#endif

#ifdef CONFIG_PM_SLEEP
#define AVB_PM_OPS	&avb_pm
#else
#define AVB_PM_OPS	NULL
#endif

#define AVB_KERN_EMERG KERN_EMERG	/* system is unusable */
#define AVB_KERN_ALERT KERN_ALERT	/* action must be taken immediately */
#define AVB_KERN_CRIT  KERN_CRIT	/* critical conditions */
#define AVB_KERN_ERR   KERN_ERR		/* error conditions */
#define AVB_KERN_WARN  KERN_WARNING	/* warning conditions */
#define AVB_KERN_NOT   KERN_NOTICE	/* normal but significant condition */
#define AVB_KERN_INFO  KERN_INFO	/* informational */
#define AVB_KERN_DEBUG KERN_DEBUG	/* debug-level messages */

struct avb {
	struct snd_card *card;
	struct snd_pcm *pcm[1];
};

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

static int numcards = 0;
static struct platform_device *avbdevice[SND_AVB_NUM_CARDS];

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

static int avb_pcm_new(struct avb *avb,
			    int device, int substreams)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(avb->card, "AVB PCM", device,
			  substreams, substreams, &pcm);
	if (err < 0)
		return err;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &avb_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &avb_capture_ops);

	pcm->private_data = avb;
	pcm->info_flags = 0;
	strcpy(pcm->name, "AVB PCM");

	avb->pcm[device] = pcm;
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

static SIMPLE_DEV_PM_OPS(avb_pm, avb_suspend, avb_resume);

#endif

static int avb_probe(struct platform_device *devptr)
{
	struct snd_card *card;
	struct avb *avb;
	int dev = devptr->id;
	int err;

	printk(AVB_KERN_INFO "avb_probe");

	err = snd_card_new(&devptr->dev, index[dev], id[dev], THIS_MODULE,
			   sizeof(struct avb), &card);

	if (err < 0) {
		printk(AVB_KERN_INFO "avb_probe card new err: %d", err);
		return err;
	}

	avb = card->private_data;
	avb->card = card;

	err = avb_pcm_new(avb, 0, pcm_substreams[dev]);
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

static struct platform_driver avb_driver = {
	.probe		= avb_probe,
	.remove		= avb_remove,
	.driver		= {
		.name	= SND_AVB_DRIVER,
		.pm	= AVB_PM_OPS,
	},
};

static void avb_remove_all(void) {
	int i;

	printk(AVB_KERN_INFO "avb_remove_all");
	for(i=0; i < numcards; i++)
		platform_device_unregister(avbdevice[i]);
	platform_driver_unregister(&avb_driver);
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

		avbdevice[i] = dev;
		numcards++;
	}

	if(!numcards)
		avb_remove_all();

	return 0;
}

static void __exit alsa_avb_exit(void)
{
	printk(AVB_KERN_INFO "alsa_avb_exit");
	avb_remove_all();
	printk(AVB_KERN_INFO "alsa_avb_exit done");
}

module_init(alsa_avb_init)
module_exit(alsa_avb_exit)
