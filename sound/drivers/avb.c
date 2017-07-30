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
	.open =		avb_playback_open,
	.close =	avb_playback_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	avb_playback_hw_params,
	.hw_free =	avb_playback_hw_free,
	.prepare =	avb_playback_prepare,
	.trigger =	avb_playback_trigger,
	.pointer =	avb_playback_pointer,
	.copy = 	avb_playback_copy
};

static struct snd_pcm_ops avb_capture_ops = {
	.open =		avb_capture_open,
	.close =	avb_capture_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	avb_capture_hw_params,
	.hw_free =	avb_capture_hw_free,
	.prepare =	avb_capture_prepare,
	.trigger =	avb_capture_trigger,
	.pointer =	avb_capture_pointer,
	.copy = 	avb_capture_copy
};

static struct snd_pcm_hardware avb_playback_hw = {
        .info = (SNDRV_PCM_INFO_INTERLEAVED |
                 SNDRV_PCM_INFO_BLOCK_TRANSFER),
        .formats =          SNDRV_PCM_FMTBIT_S16_LE,
        .rates =            SNDRV_PCM_RATE_8000_48000,
        .rate_min =         8000,
        .rate_max =         48000,
        .channels_min =     2,
        .channels_max =     2,
        .buffer_bytes_max = 32768,
        .period_bytes_min = 4096,
        .period_bytes_max = 32768,
        .periods_min =      1,
        .periods_max =      1024,
};

static struct snd_pcm_hardware avb_capture_hw = {
        .info = (SNDRV_PCM_INFO_INTERLEAVED |
                 SNDRV_PCM_INFO_BLOCK_TRANSFER),
        .formats =          SNDRV_PCM_FMTBIT_S16_LE,
        .rates =            SNDRV_PCM_RATE_8000_48000,
        .rate_min =         8000,
        .rate_max =         48000,
        .channels_min =     2,
        .channels_max =     2,
        .buffer_bytes_max = 32768,
        .period_bytes_min = 4096,
        .period_bytes_max = 32768,
        .periods_min =      1,
        .periods_max =      1024,
};

static bool avb_socket_init(struct socketdata* sd, int rxTimeout)
{
	int err = 0;
	struct timeval tsOpts;      
	tsOpts.tv_sec = (rxTimeout / 1000);
	tsOpts.tv_usec = (rxTimeout % 1000);

	printk(AVB_KERN_INFO "avb_socket_init");

	if ((err = sock_create(AF_PACKET, SOCK_RAW, htons(sd->type), &sd->sock)) != 0) {
		printk(AVB_KERN_ERR "avb_socket_init Socket creation fails %d \n", err);
		return false;
	}

	memset(&sd->if_idx, 0, sizeof(struct ifreq));
	/*strncpy(sd->if_idx.ifr_name, "eth0", sizeof(sd->if_idx.ifr_name) - 1);
	if ((err = kernel_sock_ioctl(sd->sock, SIOCGIFINDEX, (unsigned int)&sd->if_idx)) != 0) {
		printk(AVB_KERN_ERR "avb_msrp_init SIOCGIFINDEX err: %d \n", err);
		return false;
	}*/

	memset(&sd->if_mac, 0, sizeof(struct ifreq));
	/*strncpy(sd->if_mac.ifr_name, "eth0", sizeof(sd->if_idx.ifr_name) - 1);
	if ((err = kernel_sock_ioctl(sd->sock, SIOCGIFHWADDR, (unsigned int)&sd->if_mac)) != 0) {
		printk(AVB_KERN_ERR "avb_msrp_init SIOCGIFHWADDR err:%d \n", err);
		return false;
	}*/

	if ((err = kernel_setsockopt(sd->sock, SOL_SOCKET, SO_RCVTIMEO, (void *) &tsOpts, sizeof(tsOpts))) != 0) {
		printk(KERN_WARNING "avb_msrp_init set rx timeout fails %d\n", err);
		return false;
	}

	/* Index of the network device */
	sd->txSockAddress.sll_family = AF_PACKET;
	sd->txSockAddress.sll_protocol = htons(sd->type);
	sd->txSockAddress.sll_ifindex = 2;
	/* Address length*/
	sd->txSockAddress.sll_halen = ETH_ALEN;
	/* Destination MAC */
	sd->txSockAddress.sll_addr[0] = sd->destmac[0];
	sd->txSockAddress.sll_addr[1] = sd->destmac[1];
	sd->txSockAddress.sll_addr[2] = sd->destmac[2];
	sd->txSockAddress.sll_addr[3] = sd->destmac[3];
	sd->txSockAddress.sll_addr[4] = sd->destmac[4];
	sd->txSockAddress.sll_addr[5] = sd->destmac[5];

	/* Set the message header */
	sd->txMsgHdr.msg_control=NULL;
	sd->txMsgHdr.msg_controllen=0;
	sd->txMsgHdr.msg_flags=0;
	sd->txMsgHdr.msg_name=&sd->txSockAddress;
	sd->txMsgHdr.msg_namelen=sizeof(struct sockaddr_ll);
	sd->txMsgHdr.msg_iocb = NULL;

	/* Index of the network device */
	sd->rxSockAddress.sll_family = AF_PACKET;
	sd->rxSockAddress.sll_protocol = htons(sd->type);
	sd->rxSockAddress.sll_ifindex = 2;
	/* Address length*/
	sd->rxSockAddress.sll_halen = ETH_ALEN;
	/* Destination MAC */
	sd->rxSockAddress.sll_addr[0] = sd->destmac[0];
	sd->rxSockAddress.sll_addr[1] = sd->destmac[1];
	sd->rxSockAddress.sll_addr[2] = sd->destmac[2];
	sd->rxSockAddress.sll_addr[3] = sd->destmac[3];
	sd->rxSockAddress.sll_addr[4] = sd->destmac[4];
	sd->rxSockAddress.sll_addr[5] = sd->destmac[5];

	/* Set the message header */
	sd->rxMsgHdr.msg_control=NULL;
	sd->rxMsgHdr.msg_controllen=0;
	sd->rxMsgHdr.msg_flags=0;
	sd->rxMsgHdr.msg_name=&sd->rxSockAddress;
	sd->rxMsgHdr.msg_namelen=sizeof(struct sockaddr_ll);
	sd->rxMsgHdr.msg_iocb = NULL;
	sd->rxiov.iov_base = sd->rxBuf;
	sd->rxiov.iov_len = AVB_MSRP_ETH_FRAME_SIZE;
	iov_iter_init(&sd->rxMsgHdr.msg_iter, READ | ITER_KVEC, &sd->rxiov, 1, AVB_MSRP_ETH_FRAME_SIZE);

	return true;
}

static int avb_get_avtp_aaf_format(int rtformat)
{
	int format = AVB_AVTP_AAF_FORMAT_USER_SP;

	if((rtformat == SNDRV_PCM_FORMAT_FLOAT_LE) ||
	   (rtformat == SNDRV_PCM_FORMAT_FLOAT_BE))
		format = AVB_AVTP_AAF_FORMAT_32_BIT_FLOAT;
	else if((rtformat == SNDRV_PCM_FORMAT_S32_LE) ||
		(rtformat == SNDRV_PCM_FORMAT_S32_BE) ||
		(rtformat == SNDRV_PCM_FORMAT_U32_LE) ||
		(rtformat == SNDRV_PCM_FORMAT_U32_BE))
		format = AVB_AVTP_AAF_FORMAT_32_BIT_INT;
	else if((rtformat == SNDRV_PCM_FORMAT_S24_LE) ||
		(rtformat == SNDRV_PCM_FORMAT_S24_BE) ||
		(rtformat == SNDRV_PCM_FORMAT_U24_LE) ||
		(rtformat == SNDRV_PCM_FORMAT_U24_BE))
		format = AVB_AVTP_AAF_FORMAT_24_bit_INT;
	else if((rtformat == SNDRV_PCM_FORMAT_S16_LE) ||
		(rtformat == SNDRV_PCM_FORMAT_S16_BE) ||
		(rtformat == SNDRV_PCM_FORMAT_U16_LE) ||
		(rtformat == SNDRV_PCM_FORMAT_U16_BE))
		format = AVB_AVTP_AAF_FORMAT_16_BIT_INT;
	else
		format = AVB_AVTP_AAF_FORMAT_USER_SP;

	return format;
}

static int avb_get_avtp_aaf_nsr(int sampleRate)
{
	int nsr = AVB_AVTP_AAF_NSR_USER_SP;

	if(sampleRate == 8000)
		nsr = AVB_AVTP_AAF_NSR_8_KHZ;
	else if(sampleRate == 16000)
		nsr = AVB_AVTP_AAF_NSR_16_KHZ;
	else if(sampleRate == 32000)
		nsr = AVB_AVTP_AAF_NSR_32_KHZ;
	else if(sampleRate == 44100)
		nsr = AVB_AVTP_AAF_NSR_44_1_KHZ;
	else if(sampleRate == 48000)
		nsr = AVB_AVTP_AAF_NSR_48_KHZ;
	else if(sampleRate == 88200)
		nsr = AVB_AVTP_AAF_NSR_88_2_KHZ;
	else if(sampleRate == 96000)
		nsr = AVB_AVTP_AAF_NSR_96_KHZ;
	else if(sampleRate == 176400)
		nsr = AVB_AVTP_AAF_NSR_176_4_KHZ;
	else if(sampleRate == 192000)
		nsr = AVB_AVTP_AAF_NSR_192_KHZ;
	else if(sampleRate == 24000)
		nsr = AVB_AVTP_AAF_NSR_24_KHZ;
	else
		nsr = AVB_AVTP_AAF_NSR_USER_SP;

	return nsr;
}

static void avb_avtp_aaf_header_init(char* buf, struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);
	struct ethhdr *eh = (struct ethhdr *)&buf[0];
	struct avtPduAafPcmHdr* hdr = (struct avtPduAafPcmHdr*)&buf[sizeof(struct ethhdr)];

	printk(AVB_KERN_INFO "avb_avtp_aaf_header_init");

	memset(buf, 0, AVB_MSRP_ETH_FRAME_SIZE);

	eh->h_dest[0] = avbcard->sd.destmac[0];
	eh->h_dest[1] = avbcard->sd.destmac[1];
	eh->h_dest[2] = avbcard->sd.destmac[2];
	eh->h_dest[3] = avbcard->sd.destmac[3];
	eh->h_dest[4] = avbcard->sd.destmac[4];
	eh->h_dest[5] = avbcard->sd.destmac[5];
	eh->h_source[0] = ((u8 *)&avbcard->sd.if_mac.ifr_hwaddr.sa_data)[0];
	eh->h_source[1] = ((u8 *)&avbcard->sd.if_mac.ifr_hwaddr.sa_data)[1];
	eh->h_source[2] = ((u8 *)&avbcard->sd.if_mac.ifr_hwaddr.sa_data)[2];
	eh->h_source[3] = ((u8 *)&avbcard->sd.if_mac.ifr_hwaddr.sa_data)[3];
	eh->h_source[4] = ((u8 *)&avbcard->sd.if_mac.ifr_hwaddr.sa_data)[4];
	eh->h_source[5] = ((u8 *)&avbcard->sd.if_mac.ifr_hwaddr.sa_data)[5];

	eh->h_proto = htons(avbcard->sd.type);

	hdr->h.f.subType = AVB_AVTP_SUBTYPE_AAF;
	AVB_AVTP_AAF_HDR_SET_SV(hdr, 1);
	AVB_AVTP_AAF_HDR_SET_VER(hdr, AVB_AVTP_AAF_VERSION);
	AVB_AVTP_AAF_HDR_SET_MR(hdr, 0);
	AVB_AVTP_AAF_HDR_SET_TSV(hdr, 1);
	hdr->h.f.seqNo = 0;
	AVB_AVTP_AAF_HDR_SET_TU(hdr, 0);
	hdr->h.f.streamId = 0;
	hdr->h.f.avtpTS = 0;
	hdr->h.f.format = avb_get_avtp_aaf_format(substream->runtime->format);
	AVB_AVTP_AAF_HDR_SET_NSR(hdr, avb_get_avtp_aaf_nsr(params_rate(hw_params)));
	AVB_AVTP_AAF_HDR_SET_CPF(hdr, params_channels(hw_params));
	hdr->h.f.bitDepth = substream->runtime->sample_bits;
	hdr->h.f.streamDataLen = AVTP_PDU_COMMON_STREAM_HEADER_LENGTH;
	AVB_AVTP_AAF_HDR_SET_SP(hdr, 0);
	AVB_AVTP_AAF_HDR_SET_EVT(hdr, 0);
}

static int avb_playback_open(struct snd_pcm_substream *substream)
{
	printk(AVB_KERN_INFO "avb_playback_open");

        substream->runtime->hw = avb_playback_hw;

	return 0;
}

static int avb_playback_close(struct snd_pcm_substream *substream)
{
	printk(AVB_KERN_INFO "avb_playback_close");

	return 0;
}

static int avb_playback_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	printk(AVB_KERN_INFO "avb_playback_hw_params numbytes:%d", params_buffer_bytes(hw_params));

	avbcard->hwIdx = 0;
	avbcard->numBytesConsumed = 0;

	avbcard->sd.type = ETH_P_TSN;
	avbcard->sd.destmac[0] = 0x01;
	avbcard->sd.destmac[1] = 0x80;
	avbcard->sd.destmac[2] = 0xC2;
	avbcard->sd.destmac[3] = 0x00;
	avbcard->sd.destmac[4] = 0x00;
	avbcard->sd.destmac[5] = 0x0E;

	if(!avb_socket_init(&avbcard->sd, 100)) {
		printk(AVB_KERN_INFO "avb_playback_hw_params socket init failed");
		return -1;	
	}

	avb_avtp_aaf_header_init(&avbcard->sd.txBuf[0], substream, hw_params);

	return snd_pcm_lib_alloc_vmalloc_buffer(substream, params_buffer_bytes(hw_params));
}

static int avb_playback_hw_free(struct snd_pcm_substream *substream)
{
	printk(AVB_KERN_INFO "avb_playback_hw_free");

	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int avb_playback_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static int avb_playback_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

        switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
                printk(AVB_KERN_INFO "avb_playback_trigger: Start @ %lu", jiffies);
		avbcard->startts = jiffies;
                break;
        case SNDRV_PCM_TRIGGER_STOP:
                printk(AVB_KERN_INFO "avb_playback_trigger: Stop");
                break;
        default:
		printk(AVB_KERN_INFO "avb_playback_trigger: Unknown");
                ret = -EINVAL;
        }

        return ret;
}

static snd_pcm_uframes_t avb_playback_pointer(struct snd_pcm_substream *substream)
{
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	printk(AVB_KERN_INFO "avb_playback_pointer @ %lu, hwIdx:%lu numBytes:%lu, time: %u us",
		jiffies, avbcard->hwIdx, avbcard->numBytesConsumed, jiffies_to_usecs(jiffies - avbcard->startts));

	return avbcard->hwIdx;
}

static int avb_playback_copy(struct snd_pcm_substream *substream,
                       int channel, snd_pcm_uframes_t pos,
                       void __user *dst,
                       snd_pcm_uframes_t count)
{
	int err = 0;
	int txSize = 0;
	int bytesToCopy = 0;
	int copiedBytes = 0;
	snd_pcm_uframes_t copidFrames = 0;
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);
	struct avtPduAafPcmHdr* hdr = (struct avtPduAafPcmHdr*)&avbcard->sd.txBuf[sizeof(struct ethhdr)];

	printk(AVB_KERN_INFO "avb_playback_copy: ch:%d, pos: %ld, count: %ld", channel, pos, count);

	do {
		hdr->h.f.seqNo++;

		txSize = sizeof(struct ethhdr) + sizeof(struct avtPduAafPcmHdr);
		bytesToCopy = (AVB_AVTP_AAF_SAMPLES_PER_PACKET * (substream->runtime->frame_bits / 8));

		if((err = copy_from_user(&avbcard->sd.txBuf[txSize], (&((u8*)dst)[copiedBytes]), bytesToCopy)) == 0) {
			copiedBytes += bytesToCopy;		
		} else {
			printk(KERN_WARNING "avb_playback_copy copy from user fails: %d \n", err);
			return -1;
		}
		txSize += bytesToCopy;
		copidFrames += AVB_AVTP_AAF_SAMPLES_PER_PACKET;

		printk(AVB_KERN_INFO "avb_playback_copy: bytesToCopy:%d, copiedBytes: %d, copiedFrames: %ld", bytesToCopy, copiedBytes, copidFrames);

		avbcard->sd.txiov.iov_base = avbcard->sd.txBuf;
		avbcard->sd.txiov.iov_len  = txSize;
		iov_iter_init(&avbcard->sd.txMsgHdr.msg_iter, WRITE | ITER_KVEC, &avbcard->sd.txiov, 1, txSize);

		if ((err = sock_sendmsg(avbcard->sd.sock, &avbcard->sd.txMsgHdr)) <= 0) {
			printk(KERN_WARNING "avb_playback_copy Socket transmission fails %d \n", err);
			return -1;
		}
	} while(copidFrames < count);

	avbcard->hwIdx = (pos + count - 1);
	avbcard->numBytesConsumed += (count * (substream->runtime->frame_bits / 8));

	return count;
}

static int avb_capture_open(struct snd_pcm_substream *substream)
{
	printk(AVB_KERN_INFO "avb_capture_open");

        substream->runtime->hw = avb_capture_hw;

	return 0;
}

static int avb_capture_close(struct snd_pcm_substream *substream)
{
	printk(AVB_KERN_INFO "avb_capture_close");

	return 0;
}

static int avb_capture_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	avbcard->hwIdx = 0;
	avbcard->fillsize = 0;
	avbcard->prevHwIdx = 0;
	avbcard->numBytesConsumed = 0;
	avbcard->periodsize = params_period_size(hw_params);
	avbcard->buffersize = params_buffer_bytes(hw_params);
	avbcard->framecount = params_buffer_size(hw_params);
	avbcard->framesize  = params_buffer_bytes(hw_params) / params_buffer_size(hw_params);

	printk(AVB_KERN_INFO "avb_capture_hw_params buffersize:%d framesize:%d", avbcard->buffersize, avbcard->framesize);

	avbdevice.avtpwd = (struct workdata*)kmalloc(sizeof(struct workdata), GFP_KERNEL);
	if(avbdevice.avtpwd == NULL) {
		printk(AVB_KERN_ERR "avb_capture_hw_params avtp workdata allocation failed");
		return -1;
	}

	avbdevice.avtpwd->dw.card = avbcard;
	avbdevice.avtpwd->delayedWorkId = AVB_DELAY_WORK_AVTP;
	avbdevice.avtpwd->substream = substream;
	INIT_DELAYED_WORK((struct delayed_work*)avbdevice.avtpwd, avbWqFn);
			
	queue_delayed_work(avbdevice.wq, (struct delayed_work*)avbdevice.avtpwd, 1);

	return snd_pcm_lib_alloc_vmalloc_buffer(substream, avbcard->buffersize);
}

static int avb_capture_hw_free(struct snd_pcm_substream *substream)
{
	printk(AVB_KERN_INFO "avb_capture_hw_free");

	if(avbdevice.avtpwd != NULL) {
		cancel_delayed_work((struct delayed_work*)avbdevice.avtpwd);
		kfree(avbdevice.avtpwd);
	}

	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int avb_capture_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static int avb_capture_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

        switch (cmd) {
        case SNDRV_PCM_TRIGGER_START:
                printk(AVB_KERN_INFO "avb_capture_trigger: Start @ %lu", jiffies);
		avbcard->startts = jiffies;
                break;
        case SNDRV_PCM_TRIGGER_STOP:
                printk(AVB_KERN_INFO "avb_capture_trigger: Stop");
                break;
        default:
		printk(AVB_KERN_INFO "avb_capture_trigger: Unknown");
                ret = -EINVAL;
        }

        return ret;
}

static snd_pcm_uframes_t avb_capture_pointer(struct snd_pcm_substream *substream)
{
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	printk(AVB_KERN_INFO "avb_capture_pointer @ %lu, hwIdx:%lu numBytes:%lu time: %u us",
		jiffies, avbcard->hwIdx, avbcard->numBytesConsumed, jiffies_to_usecs(jiffies - avbcard->startts));

	return avbcard->hwIdx;
}

static int avb_capture_copy(struct snd_pcm_substream *substream,
                       int channel, snd_pcm_uframes_t pos,
                       void __user *dst,
                       snd_pcm_uframes_t count)
{
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	printk(AVB_KERN_INFO "avb_capture_copy: ch:%d, pos: %ld, count: %ld", channel, pos, count);

	avbcard->numBytesConsumed += (count * avbcard->framesize);

	return count;
}

static bool avb_msrp_init(struct msrp* msrp)
{
	printk(AVB_KERN_INFO "avb_msrp_init");

	msrp->talkerState   = MSRP_DECLARATION_STATE_NONE;
	msrp->listenerState = MSRP_DECLARATION_STATE_NONE;

	msrp->sd.type = ETH_MSRP;
	msrp->sd.destmac[0] = 0x01;
	msrp->sd.destmac[1] = 0x80;
	msrp->sd.destmac[2] = 0xC2;
	msrp->sd.destmac[3] = 0x00;
	msrp->sd.destmac[4] = 0x00;
	msrp->sd.destmac[5] = 0x0E;

	return avb_socket_init(&msrp->sd, 1000);
}

static void avb_msrp_talkerdeclarations(struct msrp* msrp, bool join)
{
	int txSize = 0;
	int err = 0;
	struct ethhdr *eh = (struct ethhdr *)&msrp->sd.txBuf[0];
	struct talkermsrpdu *pdu = (struct talkermsrpdu*)&msrp->sd.txBuf[sizeof(struct ethhdr)];

	printk(AVB_KERN_INFO "avb_msrp_join");

	/* Initialize it */
	memset(msrp->sd.txBuf, 0, AVB_MSRP_ETH_FRAME_SIZE);

	/* Fill in the Ethernet header */
	eh->h_dest[0] = 0x01;
	eh->h_dest[1] = 0x80;
	eh->h_dest[2] = 0xC2;
	eh->h_dest[3] = 0x00;
	eh->h_dest[4] = 0x00;
	eh->h_dest[5] = 0x0E;
	eh->h_source[0] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[0];
	eh->h_source[1] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[1];
	eh->h_source[2] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[2];
	eh->h_source[3] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[3];
	eh->h_source[4] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[4];
	eh->h_source[5] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[5];

	/* Fill in Ethertype field */
	eh->h_proto = htons(msrp->sd.type);

	pdu->protocolversion = 0;
	pdu->msg.attributetype = MSRP_ATTRIBUTE_TYPE_TALKER_ADVERTISE_VECTOR;
	pdu->msg.attributelen  = MSRP_ATTRIBUTE_LEN_TALKER_ADVERTISE_VECTOR;
	pdu->msg.attributelistlen = sizeof(struct talkervectorattribute);

	pdu->msg.attibutelist.hdr.numberofvalues = 1;
	pdu->msg.attibutelist.val.streamid[0] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[0];
	pdu->msg.attibutelist.val.streamid[1] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[1];
	pdu->msg.attibutelist.val.streamid[2] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[2];
	pdu->msg.attibutelist.val.streamid[3] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[3];
	pdu->msg.attibutelist.val.streamid[4] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[4];
	pdu->msg.attibutelist.val.streamid[5] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[5];
	pdu->msg.attibutelist.val.streamid[6] = 0;
	pdu->msg.attibutelist.val.streamid[7] = 1;

	pdu->msg.attibutelist.val.dataframeparams[0] = 0;
	pdu->msg.attibutelist.val.dataframeparams[1] = 0;
	pdu->msg.attibutelist.val.dataframeparams[2] = 0;
	pdu->msg.attibutelist.val.dataframeparams[3] = 0;
	pdu->msg.attibutelist.val.dataframeparams[4] = 0;
	pdu->msg.attibutelist.val.dataframeparams[5] = 0;
	pdu->msg.attibutelist.val.dataframeparams[6] = 0;
	pdu->msg.attibutelist.val.dataframeparams[7] = 0;

	pdu->msg.attibutelist.val.maxFrameSize = MSRP_MAX_FRAME_SIZE_48KHZ_AUDIO;
	pdu->msg.attibutelist.val.maxintervalframes = MSRP_MAX_INTERVAL_FRAME_48KHZ_AUDIO;
	pdu->msg.attibutelist.val.priorityandrank = 0;
	pdu->msg.attibutelist.val.accumalatedlatency = 0;

	pdu->msg.attibutelist.vector[0] = MSRP_THREE_PACK(((join == true)?(MSRP_ATTRIBUTE_EVENT_JOININ):(MSRP_ATTRIBUTE_EVENT_LEAVE)), 0, 0);

	pdu->msg.endmarker = 0;
	pdu->endmarker = 0;

	txSize = sizeof(struct ethhdr) + sizeof(struct talkermsrpdu);

	msrp->sd.txiov.iov_base = msrp->sd.txBuf;
	msrp->sd.txiov.iov_len = txSize;
	iov_iter_init(&msrp->sd.txMsgHdr.msg_iter, WRITE | ITER_KVEC, &msrp->sd.txiov, 1, txSize);

	if ((err = sock_sendmsg(msrp->sd.sock, &msrp->sd.txMsgHdr)) <= 0) {
		printk(KERN_WARNING "avb_msrp_talkerdeclarations Socket transmission fails %d \n", err);
		return;
	}
}

static void avb_msrp_listenerdeclarations(struct msrp* msrp, bool join, int state)
{
	int txSize = 0;
	int err = 0;
	struct ethhdr *eh = (struct ethhdr *)&msrp->sd.txBuf[0];
	struct listnermsrpdu *pdu = (struct listnermsrpdu*)&msrp->sd.txBuf[sizeof(struct ethhdr)];

	printk(AVB_KERN_INFO "avb_msrp_join");

	/* Initialize it */
	memset(msrp->sd.txBuf, 0, AVB_MSRP_ETH_FRAME_SIZE);

	/* Fill in the Ethernet header */
	eh->h_dest[0] = 0x01;
	eh->h_dest[1] = 0x80;
	eh->h_dest[2] = 0xC2;
	eh->h_dest[3] = 0x00;
	eh->h_dest[4] = 0x00;
	eh->h_dest[5] = 0x0E;
	eh->h_source[0] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[0];
	eh->h_source[1] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[1];
	eh->h_source[2] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[2];
	eh->h_source[3] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[3];
	eh->h_source[4] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[4];
	eh->h_source[5] = ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[5];

	/* Fill in Ethertype field */
	eh->h_proto = htons(msrp->sd.type);

	pdu->protocolversion = 0;
	pdu->msg.attributetype = MSRP_ATTRIBUTE_TYPE_LISTENER_VECTOR;
	pdu->msg.attributelen  = MSRP_ATTRIBUTE_LEN_LISTENER_VECTOR;
	pdu->msg.attributelistlen = sizeof(struct listnervectorattribute);

	pdu->msg.attibutelist.hdr.numberofvalues = 1;
	pdu->msg.attibutelist.val.streamid[0] = msrp->streamid[0];
	pdu->msg.attibutelist.val.streamid[1] = msrp->streamid[1];
	pdu->msg.attibutelist.val.streamid[2] = msrp->streamid[2];
	pdu->msg.attibutelist.val.streamid[3] = msrp->streamid[3];
	pdu->msg.attibutelist.val.streamid[4] = msrp->streamid[4];
	pdu->msg.attibutelist.val.streamid[5] = msrp->streamid[5];
	pdu->msg.attibutelist.val.streamid[6] = msrp->streamid[6];
	pdu->msg.attibutelist.val.streamid[7] = msrp->streamid[7];

	pdu->msg.attibutelist.vector[0] = MSRP_THREE_PACK(((join == true)?(MSRP_ATTRIBUTE_EVENT_JOININ):(MSRP_ATTRIBUTE_EVENT_LEAVE)), 0, 0);
	pdu->msg.attibutelist.vector[1] = MSRP_FOUR_PACK(state, 0, 0, 0);

	pdu->msg.endmarker = 0;
	pdu->endmarker = 0;

	txSize = sizeof(struct ethhdr) + sizeof(struct listnermsrpdu);

	msrp->sd.txiov.iov_base = msrp->sd.txBuf;
	msrp->sd.txiov.iov_len = txSize;
	iov_iter_init(&msrp->sd.txMsgHdr.msg_iter, WRITE | ITER_KVEC, &msrp->sd.txiov, 1, txSize);

	if ((err = sock_sendmsg(msrp->sd.sock, &msrp->sd.txMsgHdr)) <= 0) {
		printk(KERN_WARNING "avb_msrp_listenerdeclarations Socket transmission fails %d \n", err);
		return;
	}
}

static int avb_msrp_evaluateTalkerAdvertisement(struct msrp* msrp)
{
	int rxState = MSRP_DECLARATION_STATE_ASKING_FAILED;
	struct talkermsrpdu *tpdu = (struct talkermsrpdu*)&msrp->sd.rxBuf[sizeof(struct ethhdr)];

	if(tpdu->msg.attributetype == MSRP_ATTRIBUTE_TYPE_TALKER_ADVERTISE_VECTOR) {
		rxState = MSRP_DECLARATION_STATE_READY;
	} else {
		rxState = MSRP_DECLARATION_STATE_ASKING_FAILED;	
	}

	memcpy(&msrp->streamid[0], &tpdu->msg.attibutelist.val.streamid[0], 8);

	return rxState;
}

static void avb_msrp_evaluateListnerAdvertisement(struct msrp* msrp)
{
	struct listnermsrpdu *pdu = (struct listnermsrpdu*)&msrp->sd.txBuf[sizeof(struct ethhdr)];

	if((pdu->msg.attibutelist.val.streamid[0] == ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[0]) &&
	   (pdu->msg.attibutelist.val.streamid[1] == ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[1]) && 
	   (pdu->msg.attibutelist.val.streamid[2] == ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[2]) && 
	   (pdu->msg.attibutelist.val.streamid[3] == ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[3]) && 
	   (pdu->msg.attibutelist.val.streamid[4] == ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[4]) && 
	   (pdu->msg.attibutelist.val.streamid[5] == ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[5]) &&
	   (pdu->msg.attibutelist.val.streamid[6] == ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[6]) && 
	   (pdu->msg.attibutelist.val.streamid[7] == ((u8 *)&msrp->sd.if_mac.ifr_hwaddr.sa_data)[7])) {
	} 
}

static void avb_msrp_listen(struct msrp* msrp)
{
	int err = 0;
	int rxState = MSRP_DECLARATION_STATE_ASKING_FAILED;
	struct listnermsrpdu *tpdu = (struct listnermsrpdu*)&msrp->sd.rxBuf[sizeof(struct ethhdr)];

	printk(AVB_KERN_INFO "avb_msrp_listen");

	if ((err = sock_recvmsg(msrp->sd.sock, &msrp->sd.rxMsgHdr, AVB_MSRP_ETH_FRAME_SIZE, 0)) > 0) {
		if(tpdu->protocolversion != 0) {
			printk(KERN_WARNING "avb_msrp_listen unknown protocolversion %d \n", tpdu->protocolversion);
			return;
		} else {
			if((tpdu->msg.attributetype == MSRP_ATTRIBUTE_TYPE_TALKER_ADVERTISE_VECTOR) ||
			   (tpdu->msg.attributetype == MSRP_ATTRIBUTE_TYPE_TALKER_FAILED_VECTOR)) {
				rxState = avb_msrp_evaluateTalkerAdvertisement(msrp);
				avb_msrp_listenerdeclarations(msrp, true, rxState);
			} else if(tpdu->msg.attributetype == MSRP_ATTRIBUTE_TYPE_LISTENER_VECTOR) {
				avb_msrp_evaluateListnerAdvertisement(msrp);
			} else if(tpdu->msg.attributetype == MSRP_ATTRIBUTE_TYPE_DOMAIN_VECTOR) {
			} else {
				printk(KERN_WARNING "avb_msrp_listen unknown attribute type %d \n", tpdu->msg.attributetype);
				return;
			}		
		}
	} else {
		printk(KERN_WARNING "avb_msrp_listen Socket reception fails %d \n", err);
		return;
	}
}

static void avbWqFn(struct work_struct *work)
{
	bool initDone = true;
	int fillsize = 0;
	struct workdata* wd = (struct workdata*)work;

	if(wd->delayedWorkId == AVB_DELAY_WORK_MSRP) {
		printk(AVB_KERN_INFO "avbWqFn: MSRP");

		if(wd->dw.msrp->initialized == false) {
			initDone = false;
			wd->dw.msrp->initialized = avb_msrp_init(wd->dw.msrp);
		}

		if(wd->dw.msrp->initialized == false) {
			queue_delayed_work(avbdevice.wq, (struct delayed_work*)avbdevice.msrpwd, 10000);
		} else {
			if(initDone == false) {
				avb_msrp_talkerdeclarations(wd->dw.msrp, true);
			}
			avb_msrp_listen(wd->dw.msrp);
			queue_delayed_work(avbdevice.wq, (struct delayed_work*)avbdevice.msrpwd, 2000);
		}
	} else if(wd->delayedWorkId == AVB_DELAY_WORK_AVTP) {

		wd->dw.card->hwIdx += AVB_AVTP_AAF_SAMPLES_PER_PACKET;
		wd->dw.card->hwIdx %= wd->dw.card->framecount;

		if (wd->dw.card->hwIdx < wd->dw.card->prevHwIdx)
                        fillsize = wd->dw.card->framecount + wd->dw.card->prevHwIdx - wd->dw.card->hwIdx;
                else
                        fillsize = wd->dw.card->hwIdx - wd->dw.card->prevHwIdx;

		wd->dw.card->prevHwIdx = wd->dw.card->hwIdx;
		wd->dw.card->fillsize += fillsize;

		printk(AVB_KERN_INFO "avbWqFn: AVTP hwIdx:%ld fillSize: %d", wd->dw.card->hwIdx, wd->dw.card->fillsize);
		
		if(wd->dw.card->fillsize >= wd->dw.card->periodsize) {
			wd->dw.card->fillsize %= wd->dw.card->periodsize;
			snd_pcm_period_elapsed(wd->substream);
		}

		queue_delayed_work(avbdevice.wq, (struct delayed_work*)avbdevice.avtpwd, 1);
	} else {
		printk(AVB_KERN_INFO "avbWqFn: Unknown: %d", wd->delayedWorkId);
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

		avbdevice.msrpwd = (struct workdata*)kmalloc(sizeof(struct workdata), GFP_KERNEL);
		if(avbdevice.msrpwd == NULL) {
			printk(AVB_KERN_ERR "alsa_avb_init msrp workdata allocation failed");
			return -1;
		}

		avbdevice.msrpwd->dw.msrp = &avbdevice.msrp;
		avbdevice.msrpwd->delayedWorkId = AVB_DELAY_WORK_MSRP;
		INIT_DELAYED_WORK((struct delayed_work*)avbdevice.msrpwd, avbWqFn);
				
		queue_delayed_work(avbdevice.wq, (struct delayed_work*)avbdevice.msrpwd, 100);

		printk(AVB_KERN_INFO "alsa_avb_init done err: %d, numcards: %d", err, numcards);	
	}

	return 0;
}

static void __exit alsa_avb_exit(void)
{
	printk(AVB_KERN_INFO "alsa_avb_exit");
	
	if(avbdevice.msrpwd != NULL) {
		cancel_delayed_work((struct delayed_work*)avbdevice.msrpwd);
		kfree(avbdevice.msrpwd);
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
