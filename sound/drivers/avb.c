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
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/info.h>
#include <sound/initval.h>
#include <sound/hwdep.h>

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

#define AVB_DEBUG

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
        .rates =            SNDRV_PCM_RATE_8000_192000,
        .rate_min =         8000,
        .rate_max =         192000,
        .channels_min =     1,
        .channels_max =     8,
        .buffer_bytes_max = 32768,
        .period_bytes_min = 512,
        .period_bytes_max = 8192,
        .periods_min =      1,
        .periods_max =      4,
};

static struct snd_pcm_hardware avb_capture_hw = {
        .info = (SNDRV_PCM_INFO_INTERLEAVED |
                 SNDRV_PCM_INFO_BLOCK_TRANSFER),
        .formats =          SNDRV_PCM_FMTBIT_S16_LE,
        .rates =            SNDRV_PCM_RATE_8000_192000,
        .rate_min =         8000,
        .rate_max =         192000,
        .channels_min =     1,
        .channels_max =     8,
        .buffer_bytes_max = 32768,
        .period_bytes_min = 512,
        .period_bytes_max = 8192,
        .periods_min =      1,
        .periods_max =      4,
};

static void avb_log(int level, char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);

	switch(level) {
		case AVB_KERN_EMERG:
			vprintk(fmt, args);
			break;
		case AVB_KERN_ALERT:
			vprintk(fmt, args);
			break;
		case AVB_KERN_CRIT:
			vprintk(fmt, args);
			break;
		case AVB_KERN_ERR:
			vprintk(fmt, args);
			break;
		case AVB_KERN_WARN:
			vprintk(fmt, args);
			break;
		case AVB_KERN_NOT:
			vprintk(fmt, args);
			break;
#ifdef AVB_DEBUG
		case AVB_KERN_INFO:
			vprintk(fmt, args);
			break;
		case AVB_KERN_DEBUG:
			vprintk(fmt, args);
			break;
#else
		default:
			break;
#endif
	}

	va_end(args);
}

static bool avb_socket_init(struct socketdata* sd, int rxTimeout)
{
	int err = 0;
	struct net_device *dev = NULL;
	struct net *net;
	struct timeval tsOpts;      
	tsOpts.tv_sec = (rxTimeout / 1000);
	tsOpts.tv_usec = (rxTimeout % 1000);

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_socket_init");

	if ((err = sock_create(AF_PACKET, SOCK_RAW, htons(sd->type), &sd->sock)) != 0) {
		avb_log(AVB_KERN_ERR, KERN_ERR "avb_socket_init Socket creation fails %d \n", err);
		return false;
	}

	net = sock_net(sd->sock->sk);
	dev = dev_get_by_name_rcu(net, "eth0");

	memcpy(&sd->srcmac[0], dev->dev_addr, 6);
	sd->ifidx = dev->ifindex;

	rtnl_lock();
	dev_set_promiscuity(dev, 1);
	rtnl_unlock();

	if ((err = kernel_setsockopt(sd->sock, SOL_SOCKET, SO_RCVTIMEO, (void *) &tsOpts, sizeof(tsOpts))) != 0) {
		avb_log(AVB_KERN_WARN, KERN_WARNING "avb_msrp_init set rx timeout fails %d\n", err);
		return false;
	}

	/* Index of the network device */
	sd->txSockAddress.sll_family = AF_PACKET;
	sd->txSockAddress.sll_protocol = htons(sd->type);
	sd->txSockAddress.sll_ifindex = sd->ifidx;
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
	sd->rxSockAddress.sll_ifindex = sd->ifidx;
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

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_avtp_aaf_header_init");

	memset(buf, 0, AVB_MSRP_ETH_FRAME_SIZE);

	eh->h_dest[0] = avbcard->sd.destmac[0];
	eh->h_dest[1] = avbcard->sd.destmac[1];
	eh->h_dest[2] = avbcard->sd.destmac[2];
	eh->h_dest[3] = avbcard->sd.destmac[3];
	eh->h_dest[4] = avbcard->sd.destmac[4];
	eh->h_dest[5] = avbcard->sd.destmac[5];
	eh->h_source[0] = avbcard->sd.srcmac[0];
	eh->h_source[1] = avbcard->sd.srcmac[1];
	eh->h_source[2] = avbcard->sd.srcmac[2];
	eh->h_source[3] = avbcard->sd.srcmac[3];
	eh->h_source[4] = avbcard->sd.srcmac[4];
	eh->h_source[5] = avbcard->sd.srcmac[5];

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
	hdr->h.f.streamDataLen = 0;
	AVB_AVTP_AAF_HDR_SET_SP(hdr, 1);
	AVB_AVTP_AAF_HDR_SET_EVT(hdr, 0);
}

static int avb_playback_open(struct snd_pcm_substream *substream)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_playback_open");

        substream->runtime->hw = avb_playback_hw;

	return 0;
}

static int avb_playback_close(struct snd_pcm_substream *substream)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_playback_close");

	return 0;
}

static int avb_playback_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_playback_hw_params numbytes:%d sr:%d", params_buffer_bytes(hw_params), params_rate(hw_params));

	avbcard->tx.substream = substream;
	avbcard->tx.sr = params_rate(hw_params);
	avbcard->tx.hwIdx = 0;
	avbcard->tx.seqNo = 0;
	avbcard->tx.hwnwIdx = 0;
	avbcard->tx.fillsize = 0;
	avbcard->tx.lastTimerTs = jiffies;
	avbcard->tx.socketCount = 0;
	avbcard->tx.pendingTxFrames = 0;
	avbcard->tx.numBytesConsumed = 0;
	avbcard->tx.periodsize = params_period_size(hw_params);
	avbcard->tx.buffersize = params_buffer_bytes(hw_params);
	avbcard->tx.framecount = params_buffer_size(hw_params);
	avbcard->tx.framesize  = params_buffer_bytes(hw_params) / params_buffer_size(hw_params);

	avb_avtp_aaf_header_init(&avbcard->sd.txBuf[0], substream, hw_params);

	init_timer(&avbdevice.txTimer);

	memset(&avbdevice.txts[0], 0, (sizeof(int) * AVB_MAX_TS_SLOTS));
	avbdevice.txIdx = 0;
	avbdevice.txTimer.data     = (unsigned long)avbcard;
	avbdevice.txTimer.function = avb_avtp_timer;
	avbdevice.txTimer.expires  = jiffies + 1;
	add_timer(&avbdevice.txTimer);

	avbcard->tx.tmpbuf = kmalloc(avbcard->tx.buffersize, GFP_KERNEL);

	return 0;
}

static int avb_playback_hw_free(struct snd_pcm_substream *substream)
{
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_playback_hw_free");

	del_timer(&avbdevice.txTimer);
	kfree(avbcard->tx.tmpbuf);

	return 0;
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
                avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_playback_trigger: Start @ %lu", jiffies);
		avbcard->tx.startts = jiffies;
                break;
        case SNDRV_PCM_TRIGGER_STOP:
                avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_playback_trigger: Stop @ %lu", jiffies);
                break;
        default:
		avb_log(AVB_KERN_WARN, KERN_WARNING "avb_playback_trigger: Unknown");
                ret = -EINVAL;
        }

        return ret;
}

static snd_pcm_uframes_t avb_playback_pointer(struct snd_pcm_substream *substream)
{
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_playback_pointer hwIdx:%lu numBytes:%lu, time: %u us",
		avbcard->tx.hwIdx, avbcard->tx.numBytesConsumed, jiffies_to_usecs(jiffies - avbcard->tx.startts));

	return avbcard->tx.hwIdx;
}

static int avb_playback_copy(struct snd_pcm_substream *substream,
                       int channel, snd_pcm_uframes_t pos,
                       void __user *dst,
                       snd_pcm_uframes_t count)
{
	int err = 0;
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_playback_copy: ch:%d, pos: %ld, count: %lu", channel, pos, count);

	if((err = copy_from_user(&avbcard->tx.tmpbuf[(pos * avbcard->tx.framesize)], dst, (count * avbcard->tx.framesize))) != 0) {
		avb_log(AVB_KERN_WARN, KERN_WARNING "avb_playback_copy copy from user fails: %d \n", err);
		return -1;
	}

	avbcard->tx.pendingTxFrames  += count;
	avbcard->tx.numBytesConsumed += (count * (substream->runtime->frame_bits / 8));

	avb_avtp_timer((unsigned long)avbcard);

	if((avbcard->tx.pendingTxFrames > 0) && (!timer_pending(&avbdevice.txTimer))) {
		avbdevice.txTimer.expires  = jiffies + 1;
		add_timer(&avbdevice.txTimer);
	}

	return 0;
}

static void avb_avtp_timer(unsigned long arg)
{
	int i = 0;
	int err = 0;
	int txSize = 0;
	snd_pcm_uframes_t bytesAvai = 0;
	snd_pcm_uframes_t bytesToCopy  = 0;
	snd_pcm_uframes_t framesToCopy = 0;
	snd_pcm_uframes_t avtpFramesPerPacket = 0;
	snd_pcm_uframes_t avtpMaxFramesPerPacket = 0;
	struct avbcard *avbcard = (struct avbcard *)arg;
	unsigned long int numJiffies = ((jiffies > avbcard->tx.lastTimerTs)?(jiffies - avbcard->tx.lastTimerTs):(1));
	snd_pcm_uframes_t frameCount = ((avbcard->tx.sr * numJiffies) / HZ);
	struct avtPduAafPcmHdr* hdr = (struct avtPduAafPcmHdr*)&avbcard->sd.txBuf[sizeof(struct ethhdr)];

	avbcard->tx.accumframecount += frameCount;

	avtpMaxFramesPerPacket = ((ETH_DATA_LEN - sizeof(struct avtPduAafPcmHdr)) / avbcard->tx.framesize);
	avtpFramesPerPacket = (avbcard->tx.sr / HZ);
	avtpFramesPerPacket = ((avtpFramesPerPacket > avtpMaxFramesPerPacket)?(avtpMaxFramesPerPacket):(avtpFramesPerPacket));

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_avtp_timer ct: %lu, mfppk: %lu, fppk: %lu, frSz: %lu, sr: %lu, HZ: %lu",
		frameCount, avtpMaxFramesPerPacket, avtpFramesPerPacket, avbcard->tx.framesize, avbcard->tx.sr, HZ);

	while(((avbcard->tx.accumframecount >= avtpFramesPerPacket) ||
	       (avbcard->tx.pendingTxFrames <= avtpFramesPerPacket)) &&
	      ((avbcard->tx.pendingTxFrames > 0) && (i < 32))) { 

		i++; /* Just as a failsafe to quit loop */
		avbcard->tx.seqNo++;
		hdr->h.f.seqNo = avbcard->tx.seqNo;

		txSize = sizeof(struct ethhdr) + sizeof(struct avtPduAafPcmHdr);
		framesToCopy = ((avbcard->tx.pendingTxFrames > avtpFramesPerPacket)?(avtpFramesPerPacket):(avbcard->tx.pendingTxFrames));
		bytesToCopy  = (framesToCopy * avbcard->tx.framesize);

		bytesAvai = ((avbcard->tx.framecount - avbcard->tx.hwIdx) * avbcard->tx.framesize);
		bytesAvai = ((bytesAvai >= bytesToCopy)?(bytesToCopy):(bytesAvai));

		memcpy(&avbcard->sd.txBuf[txSize], &avbcard->tx.tmpbuf[(avbcard->tx.hwIdx * avbcard->tx.framesize)], bytesAvai);

		if(bytesAvai < bytesToCopy) {
			memcpy(&avbcard->sd.txBuf[txSize+bytesAvai], &avbcard->tx.tmpbuf[0], (bytesToCopy - bytesAvai));
		}

		hdr->h.f.avtpTS = avbdevice.txts[((avbcard->tx.hwnwIdx / avbcard->tx.periodsize) % AVB_MAX_TS_SLOTS)];
		hdr->h.f.streamDataLen = bytesToCopy;
		txSize += bytesToCopy;

		avbcard->sd.txiov.iov_base = avbcard->sd.txBuf;
		avbcard->sd.txiov.iov_len  = txSize;
		iov_iter_init(&avbcard->sd.txMsgHdr.msg_iter, WRITE | ITER_KVEC, &avbcard->sd.txiov, 1, txSize);

		if ((err = sock_sendmsg(avbcard->sd.sock, &avbcard->sd.txMsgHdr)) <= 0) {
			avb_log(AVB_KERN_WARN, KERN_WARNING "avb_avtp_timer Socket transmission fails %d \n", err);
			return;
		}

		avbcard->tx.accumframecount = ((avbcard->tx.accumframecount > framesToCopy)?(avbcard->tx.accumframecount - framesToCopy):(0));

		avbcard->tx.hwIdx += framesToCopy;
		avbcard->tx.hwnwIdx += framesToCopy;
		avbcard->tx.fillsize += framesToCopy;
		avbcard->tx.hwIdx = ((avbcard->tx.hwIdx < avbcard->tx.framecount)?(avbcard->tx.hwIdx):(avbcard->tx.hwIdx % avbcard->tx.framecount));
		avbcard->tx.pendingTxFrames -= framesToCopy;

		avb_log(AVB_KERN_INFO, KERN_INFO "avb_avtp_timer seqNo:%d, hwIdx: %lu, afrCt: %lu, penFrs:%lu, filSz:%lu",
			hdr->h.f.seqNo, avbcard->tx.hwIdx, avbcard->tx.accumframecount, avbcard->tx.pendingTxFrames, avbcard->tx.fillsize);

		if(avbcard->tx.fillsize >= avbcard->tx.periodsize) {
			avbcard->tx.fillsize %= avbcard->tx.periodsize;
			snd_pcm_period_elapsed(avbcard->tx.substream);
		}
	}

	if((avbcard->tx.pendingTxFrames > 0) && (!timer_pending(&avbdevice.txTimer))) {
		avbdevice.txTimer.expires  = jiffies + 1;
		add_timer(&avbdevice.txTimer);
	}

	avbcard->tx.lastTimerTs = jiffies;
}

static int avb_capture_open(struct snd_pcm_substream *substream)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_capture_open");

        substream->runtime->hw = avb_capture_hw;

	return 0;
}

static int avb_capture_close(struct snd_pcm_substream *substream)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_capture_close");

	return 0;
}

static int avb_capture_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	avbcard->rx.substream = substream;
	avbcard->rx.hwIdx = 0;
	avbcard->rx.seqNo = 0;
	avbcard->rx.hwnwIdx = 0;
	avbcard->rx.fillsize = 0;
	avbcard->rx.prevHwIdx = 0;
	avbcard->rx.socketCount = 0;
	avbcard->rx.numBytesConsumed = 0;
	avbcard->rx.periodsize = params_period_size(hw_params);
	avbcard->rx.buffersize = params_buffer_bytes(hw_params);
	avbcard->rx.framecount = params_buffer_size(hw_params);
	avbcard->rx.framesize  = params_buffer_bytes(hw_params) / params_buffer_size(hw_params);

	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_capture_hw_params buffersize:%lu framesize:%lu", avbcard->rx.buffersize, avbcard->rx.framesize);

	avbdevice.avtpwd = (struct workdata*)kmalloc(sizeof(struct workdata), GFP_KERNEL);
	if(avbdevice.avtpwd == NULL) {
		avb_log(AVB_KERN_ERR, KERN_ERR "avb_capture_hw_params avtp workdata allocation failed");
		return -1;
	}

	memset(&avbdevice.rxts[0], 0, (sizeof(int) * AVB_MAX_TS_SLOTS));
	avbdevice.rxIdx = 0;
	avbdevice.avtpwd->dw.card = avbcard;
	avbdevice.avtpwd->delayedWorkId = AVB_DELAY_WORK_AVTP;
	INIT_DELAYED_WORK((struct delayed_work*)avbdevice.avtpwd, avbWqFn);

	queue_delayed_work(avbdevice.wq, (struct delayed_work*)avbdevice.avtpwd, 1);

	avbcard->rx.tmpbuf = kmalloc(avbcard->rx.buffersize, GFP_KERNEL);

	return 0;
}

static int avb_capture_hw_free(struct snd_pcm_substream *substream)
{
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_capture_hw_free");

	if(avbdevice.avtpwd != NULL) {
		cancel_delayed_work((struct delayed_work*)avbdevice.avtpwd);
		kfree(avbdevice.avtpwd);
		avbdevice.avtpwd = NULL;
	}

	kfree(avbcard->rx.tmpbuf);

	return 0;
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
                avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_capture_trigger: Start @ %lu", jiffies);
		avbcard->rx.startts = jiffies;
                break;
        case SNDRV_PCM_TRIGGER_STOP:
                avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_capture_trigger: Stop");
                break;
        default:
		avb_log(AVB_KERN_WARN, KERN_WARNING "avb_capture_trigger: Unknown");
                ret = -EINVAL;
        }

        return ret;
}

static snd_pcm_uframes_t avb_capture_pointer(struct snd_pcm_substream *substream)
{
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_capture_pointer hwIdx:%lu numBytes:%lu",
		avbcard->rx.hwIdx, avbcard->rx.numBytesConsumed);

	return avbcard->rx.hwIdx;
}

static int avb_capture_copy(struct snd_pcm_substream *substream,
                       int channel, snd_pcm_uframes_t pos,
                       void __user *dst,
                       snd_pcm_uframes_t count)
{
	char* srcbuf;
	int copyres = 0;
	struct avbcard *avbcard = snd_pcm_substream_chip(substream);

	srcbuf = (char*)&avbcard->rx.tmpbuf[pos * avbcard->rx.framesize];
	
	copyres = copy_to_user(dst, srcbuf, (count * avbcard->rx.framesize));

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_capture_copy: ch:%d, pos: %ld, ct: %ld, res: %d", channel, pos, count, copyres);

	avbcard->rx.numBytesConsumed += (count * avbcard->rx.framesize);

	return count;
}

static int avb_avtp_listen(struct avbcard* avbcard)
{
	int err = 0;
	char* srcbuf;
	char* destbuf;
	int rxOff = 0;
	int rxSize = 0;
	int nrxSize = 0;
	int avaiSize = 0;
	int rxFrames = 0;
	int nrxFrames = 0;
	int nextSeqNo = 0;
	int skippedPackets = 0;
	mm_segment_t oldfs;
	snd_pcm_uframes_t hwIdx = 0;
	struct avtPduAafPcmHdr* hdr = (struct avtPduAafPcmHdr*)&avbcard->sd.rxBuf[sizeof(struct ethhdr)];

	memset(avbcard->sd.rxBuf, 0, AVB_MSRP_ETH_FRAME_SIZE);
	avbcard->sd.rxiov.iov_base = avbcard->sd.rxBuf;
	avbcard->sd.rxiov.iov_len = AVB_MSRP_ETH_FRAME_SIZE;
	iov_iter_init(&avbcard->sd.rxMsgHdr.msg_iter, READ | ITER_KVEC, &avbcard->sd.rxiov, 1, AVB_MSRP_ETH_FRAME_SIZE);

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	err = sock_recvmsg(avbcard->sd.sock, &avbcard->sd.rxMsgHdr, AVB_MSRP_ETH_FRAME_SIZE, 0);
	set_fs(oldfs);

	if (err > 0) {
		avbcard->rx.socketCount++;

		rxOff   = sizeof(struct ethhdr) + sizeof(struct avtPduAafPcmHdr);
		srcbuf  = (char*)&avbcard->sd.rxBuf[rxOff];

		rxSize = hdr->h.f.streamDataLen;
		rxFrames = (rxSize / avbcard->rx.framesize);

		avbdevice.rxts[((avbcard->rx.hwnwIdx / avbcard->rx.periodsize) % AVB_MAX_TS_SLOTS)] = hdr->h.f.avtpTS;

		nextSeqNo = (avbcard->rx.seqNo + 1) % 256;

		if(nextSeqNo != hdr->h.f.seqNo) {
			avb_log(AVB_KERN_INFO, KERN_ERR "avb_listen missing frames from %d to %d \n",
				avbcard->rx.seqNo, hdr->h.f.seqNo);

			skippedPackets = ((hdr->h.f.seqNo >= avbcard->rx.seqNo)? \
						(hdr->h.f.seqNo - avbcard->rx.seqNo): \
						((hdr->h.f.seqNo + 255) - avbcard->rx.seqNo));
			nrxFrames = (skippedPackets * rxFrames);
			nrxSize = nrxFrames * avbcard->rx.framesize;

			avb_log(AVB_KERN_INFO, KERN_INFO "avb_listen idx: %ld nrsz: %d, nrf: %d \n",
				avbcard->rx.hwIdx, nrxSize, nrxFrames);

			avaiSize = ((avbcard->rx.framecount - avbcard->rx.hwIdx) * avbcard->rx.framesize);
			avaiSize = ((avaiSize < nrxSize)?(avaiSize):(nrxSize));
			destbuf  = (char*)&avbcard->rx.tmpbuf[avbcard->rx.hwIdx * avbcard->rx.framesize];
			memset(destbuf, 0, avaiSize);

			if(avaiSize < nrxSize) {
				destbuf  = (char*)&avbcard->rx.tmpbuf[0];
				memset(destbuf, 0, (nrxSize - avaiSize));
			}

			hwIdx = ((avbcard->rx.hwIdx + nrxFrames) % (avbcard->rx.framecount));
			rxFrames = rxFrames + nrxFrames;
		} else {
			hwIdx = avbcard->rx.hwIdx;
		}

		avb_log(AVB_KERN_INFO, KERN_INFO "avb_listen (%d) seq: %d, idx: %ld, sz: %d, ts: %u, rf: %d \n",
			avbcard->rx.socketCount, hdr->h.f.seqNo, hwIdx, rxSize, hdr->h.f.avtpTS, rxFrames);

		avbcard->rx.seqNo = hdr->h.f.seqNo;

		avaiSize = ((avbcard->rx.framecount - hwIdx) * avbcard->rx.framesize);
		avaiSize = ((avaiSize < rxSize)?(avaiSize):(rxSize));
		destbuf  = (char*)&avbcard->rx.tmpbuf[hwIdx * avbcard->rx.framesize];
		memcpy(destbuf, srcbuf, avaiSize);

		if(avaiSize < rxSize) {
			destbuf  = (char*)&avbcard->rx.tmpbuf[0];
			memcpy(destbuf, &srcbuf[avaiSize], (rxSize - avaiSize));
		}
	} else {
		if(err != -11)
			avb_log(AVB_KERN_WARN, KERN_WARNING "avb_avtp_listen Socket reception fails %d \n", err);
		return 0;
	}

	return rxFrames;
}

static bool avb_msrp_init(struct msrp* msrp)
{
	avb_log(AVB_KERN_INFO, KERN_INFO "avb_msrp_init");

	msrp->rxState = MSRP_DECLARATION_STATE_NONE;
	msrp->txState = MSRP_DECLARATION_STATE_NONE;

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

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_msrp_talkerdeclarations");

	/* Initialize it */
	memset(msrp->sd.txBuf, 0, AVB_MSRP_ETH_FRAME_SIZE);

	/* Fill in the Ethernet header */
	eh->h_dest[0] = 0x01;
	eh->h_dest[1] = 0x80;
	eh->h_dest[2] = 0xC2;
	eh->h_dest[3] = 0x00;
	eh->h_dest[4] = 0x00;
	eh->h_dest[5] = 0x0E;
	eh->h_source[0] = msrp->sd.srcmac[0];
	eh->h_source[1] = msrp->sd.srcmac[1];
	eh->h_source[2] = msrp->sd.srcmac[2];
	eh->h_source[3] = msrp->sd.srcmac[3];
	eh->h_source[4] = msrp->sd.srcmac[4];
	eh->h_source[5] = msrp->sd.srcmac[5];

	/* Fill in Ethertype field */
	eh->h_proto = htons(msrp->sd.type);

	pdu->protocolversion = 0;
	pdu->msg.attributetype = MSRP_ATTRIBUTE_TYPE_TALKER_ADVERTISE_VECTOR;
	pdu->msg.attributelen  = MSRP_ATTRIBUTE_LEN_TALKER_ADVERTISE_VECTOR;
	pdu->msg.attributelistlen = sizeof(struct talkervectorattribute);

	pdu->msg.attibutelist.hdr.numberofvalues = 1;
	pdu->msg.attibutelist.val.streamid[0] = msrp->sd.srcmac[0];
	pdu->msg.attibutelist.val.streamid[1] = msrp->sd.srcmac[1];
	pdu->msg.attibutelist.val.streamid[2] = msrp->sd.srcmac[2];
	pdu->msg.attibutelist.val.streamid[3] = msrp->sd.srcmac[3];
	pdu->msg.attibutelist.val.streamid[4] = msrp->sd.srcmac[4];
	pdu->msg.attibutelist.val.streamid[5] = msrp->sd.srcmac[5];
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
		avb_log(AVB_KERN_WARN, KERN_WARNING "avb_msrp_talkerdeclarations Socket transmission fails %d \n", err);
		return;
	}
}

static void avb_msrp_listenerdeclarations(struct msrp* msrp, bool join, int state)
{
	int txSize = 0;
	int err = 0;
	struct ethhdr *eh = (struct ethhdr *)&msrp->sd.txBuf[0];
	struct listnermsrpdu *pdu = (struct listnermsrpdu*)&msrp->sd.txBuf[sizeof(struct ethhdr)];

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_msrp_listenerdeclarations");

	/* Initialize it */
	memset(msrp->sd.txBuf, 0, AVB_MSRP_ETH_FRAME_SIZE);

	/* Fill in the Ethernet header */
	eh->h_dest[0] = 0x01;
	eh->h_dest[1] = 0x80;
	eh->h_dest[2] = 0xC2;
	eh->h_dest[3] = 0x00;
	eh->h_dest[4] = 0x00;
	eh->h_dest[5] = 0x0E;
	eh->h_source[0] = msrp->sd.srcmac[0];
	eh->h_source[1] = msrp->sd.srcmac[1];
	eh->h_source[2] = msrp->sd.srcmac[2];
	eh->h_source[3] = msrp->sd.srcmac[3];
	eh->h_source[4] = msrp->sd.srcmac[4];
	eh->h_source[5] = msrp->sd.srcmac[5];

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
		avb_log(AVB_KERN_WARN, KERN_WARNING "avb_msrp_listenerdeclarations Socket transmission fails %d \n", err);
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

static int avb_msrp_evaluateListnerAdvertisement(struct msrp* msrp)
{
	int txState = MSRP_DECLARATION_STATE_ASKING_FAILED;
	struct listnermsrpdu *pdu = (struct listnermsrpdu*)&msrp->sd.rxBuf[sizeof(struct ethhdr)];

	if((pdu->msg.attibutelist.val.streamid[0] == msrp->sd.srcmac[0]) &&
	   (pdu->msg.attibutelist.val.streamid[1] == msrp->sd.srcmac[1]) && 
	   (pdu->msg.attibutelist.val.streamid[2] == msrp->sd.srcmac[2]) && 
	   (pdu->msg.attibutelist.val.streamid[3] == msrp->sd.srcmac[3]) && 
	   (pdu->msg.attibutelist.val.streamid[4] == msrp->sd.srcmac[4]) && 
	   (pdu->msg.attibutelist.val.streamid[5] == msrp->sd.srcmac[5]) &&
	   (pdu->msg.attibutelist.val.streamid[6] == 0) && 
	   (pdu->msg.attibutelist.val.streamid[7] == 1)) {
		txState = MSRP_DECLARATION_STATE_READY;
	} 

	return txState;
}

static void avb_msrp_listen(struct msrp* msrp)
{
	int err = 0;
	mm_segment_t oldfs;
	struct listnermsrpdu *tpdu = (struct listnermsrpdu*)&msrp->sd.rxBuf[sizeof(struct ethhdr)];

	memset(msrp->sd.rxBuf, 0, AVB_MSRP_ETH_FRAME_SIZE);
	msrp->sd.rxiov.iov_base = msrp->sd.rxBuf;
	msrp->sd.rxiov.iov_len = AVB_MSRP_ETH_FRAME_SIZE;
	iov_iter_init(&msrp->sd.rxMsgHdr.msg_iter, READ | ITER_KVEC, &msrp->sd.rxiov, 1, AVB_MSRP_ETH_FRAME_SIZE);

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	err = sock_recvmsg(msrp->sd.sock, &msrp->sd.rxMsgHdr, AVB_MSRP_ETH_FRAME_SIZE, 0);
	set_fs(oldfs);
	

	if (err <= 0) {
		if(err != -11)
			avb_log(AVB_KERN_WARN, KERN_WARNING "avb_msrp_listen Socket reception res %d \n", err);
		return;
	} else {
		if(tpdu->protocolversion != 0) {
			avb_log(AVB_KERN_WARN, KERN_WARNING "avb_msrp_listen unknown protocolversion %d \n", tpdu->protocolversion);
		} else {
			if((tpdu->msg.attributetype == MSRP_ATTRIBUTE_TYPE_TALKER_ADVERTISE_VECTOR) ||
			   (tpdu->msg.attributetype == MSRP_ATTRIBUTE_TYPE_TALKER_FAILED_VECTOR)) {
				msrp->rxState = avb_msrp_evaluateTalkerAdvertisement(msrp);
				avb_msrp_listenerdeclarations(msrp, true, msrp->rxState);
			} else if(tpdu->msg.attributetype == MSRP_ATTRIBUTE_TYPE_LISTENER_VECTOR) {
				msrp->txState = avb_msrp_evaluateListnerAdvertisement(msrp);
			} else if(tpdu->msg.attributetype == MSRP_ATTRIBUTE_TYPE_DOMAIN_VECTOR) {
			} else {
				avb_log(AVB_KERN_WARN, KERN_WARNING "avb_msrp_listen unknown attribute type %d \n", tpdu->msg.attributetype);
				return;
			}

			avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_msrp_listen: rxType: %d, rxState: %d, txState: %d", tpdu->msg.attributetype,
				msrp->rxState, msrp->txState);		
		}
	}
}

static void avbWqFn(struct work_struct *work)
{
	bool initDone = true;
	int fillsize = 0;
	int rxFrames = -1;
	int rxLoopCount = 0;
	struct workdata* wd = (struct workdata*)work;

	if(wd->delayedWorkId == AVB_DELAY_WORK_MSRP) {
		avb_log(AVB_KERN_INFO, KERN_INFO "avbWqFn: MSRP @ %lu", jiffies);

		if(wd->dw.msrp->initialized == false) {
			initDone = false;
			wd->dw.msrp->initialized = avb_msrp_init(wd->dw.msrp);
		}

		if(wd->dw.msrp->initialized == false) {
			queue_delayed_work(avbdevice.wq, (struct delayed_work*)avbdevice.msrpwd, 10000);
		} else {
			if(wd->dw.msrp->txState == MSRP_DECLARATION_STATE_NONE) {
				avb_msrp_talkerdeclarations(wd->dw.msrp, true);
			}
			avb_msrp_listen(wd->dw.msrp);

			if((wd->dw.msrp->txState != MSRP_DECLARATION_STATE_READY) ||
			   (wd->dw.msrp->rxState != MSRP_DECLARATION_STATE_READY)) {
				queue_delayed_work(avbdevice.wq, (struct delayed_work*)avbdevice.msrpwd, 2000);
			}
		}
	} else if(wd->delayedWorkId == AVB_DELAY_WORK_AVTP) {

		memcpy(&avbdevice.card, wd->dw.card, sizeof(struct avbcard));

		do {
			rxFrames = avb_avtp_listen(&avbdevice.card);

			if(rxFrames > 0) {
				avbdevice.card.rx.hwIdx += rxFrames;
				avbdevice.card.rx.hwnwIdx += rxFrames;
				avbdevice.card.rx.hwIdx %= avbdevice.card.rx.framecount;

				if (avbdevice.card.rx.hwIdx < avbdevice.card.rx.prevHwIdx)
				        fillsize = avbdevice.card.rx.framecount + avbdevice.card.rx.prevHwIdx - avbdevice.card.rx.hwIdx;
				else
				        fillsize = avbdevice.card.rx.hwIdx - avbdevice.card.rx.prevHwIdx;

				avbdevice.card.rx.prevHwIdx = avbdevice.card.rx.hwIdx;
				avbdevice.card.rx.fillsize += fillsize;

				avb_log(AVB_KERN_INFO, KERN_INFO "avbWqFn: AVTP-%lu @ %lu rxFrms:%d hwIdx:%lu filSz: %lu",
					rxLoopCount++, jiffies, rxFrames, avbdevice.card.rx.hwIdx, avbdevice.card.rx.fillsize);
		
				if(avbdevice.card.rx.fillsize >= avbdevice.card.rx.periodsize) {
					avbdevice.card.rx.fillsize %= avbdevice.card.rx.periodsize;
					snd_pcm_period_elapsed(avbdevice.card.rx.substream);
				}
			} else {
				break;
			}
		

			memcpy(wd->dw.card, &avbdevice.card, sizeof(struct avbcard));

		} while(rxFrames > 0);

		if(avbdevice.avtpwd != NULL) {
			queue_delayed_work(avbdevice.wq, (struct delayed_work*)avbdevice.avtpwd, 1);
		}
	} else {
		avb_log(AVB_KERN_INFO, KERN_INFO "avbWqFn: Unknown: %d", wd->delayedWorkId);
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

static int avb_hwdep_open(struct snd_hwdep * hw, struct file *file)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_hwdep_open");

	return 0;
}

static int avb_hwdep_ioctl(struct snd_hwdep * hw, struct file *file, unsigned int cmd, unsigned long arg)
{
	int res = 0;

	if(cmd == 0) {
		avb_log(AVB_KERN_INFO, KERN_INFO "avb_hwdep_ioctl set ts: %ld @ idx: %d", arg, avbdevice.txIdx);
		avbdevice.txts[avbdevice.txIdx] = arg;
		avbdevice.txIdx++;
		avbdevice.txIdx %= AVB_MAX_TS_SLOTS;
	} else {
		res = copy_to_user((void*)arg, &avbdevice.rxts[avbdevice.rxIdx], sizeof(unsigned long));
		avb_log(AVB_KERN_INFO, KERN_INFO "avb_hwdep_ioctl get ts: %d @ %d, res: %d", avbdevice.rxts[avbdevice.rxIdx], avbdevice.rxIdx, res);
		avbdevice.rxIdx++;
		avbdevice.rxIdx %= AVB_MAX_TS_SLOTS;
	}

	return 0;
}

static int avb_hwdep_release(struct snd_hwdep * hw, struct file *file)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_hwdep_release");

	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int avb_suspend(struct device *pdev)
{
	avb_log(AVB_KERN_INFO, KERN_INFO "avb_suspend");
	return 0;
}
	
static int avb_resume(struct device *pdev)
{
	avb_log(AVB_KERN_INFO, KERN_INFO "avb_resume");
	return 0;
}

#endif

static int avb_probe(struct platform_device *devptr)
{
	struct snd_card *card;
	struct avbcard *avbcard;
	int dev = devptr->id;
	int err;

	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_probe");

	err = snd_card_new(&devptr->dev, index[dev], id[dev], THIS_MODULE,
			   sizeof(struct avbcard), &card);

	if (err < 0) {
		avb_log(AVB_KERN_ERR, KERN_ERR "avb_probe card new err: %d", err);
		return err;
	}

	avbcard = card->private_data;
	avbcard->card = card;

	err = avb_pcm_new(avbcard, 0, pcm_substreams[dev]);
	if (err < 0) {
		avb_log(AVB_KERN_ERR, KERN_ERR "avb_probe card pcm new err: %d", err);
		goto __nodev;
	}

	err = snd_hwdep_new(card, "avbhw", 0, &avbdevice.hwdep);
	if(err < 0) {
		avb_log(AVB_KERN_ERR, KERN_ERR "avb_probe card hwdep new err: %d", err);
		goto __nodev;
	}
	
	avbdevice.hwdep->ops.open    = avb_hwdep_open;
	avbdevice.hwdep->ops.ioctl   = avb_hwdep_ioctl;
	avbdevice.hwdep->ops.release = avb_hwdep_release;

	strcpy(card->driver, "avb");
	strcpy(card->shortname, "avb");
	sprintf(card->longname, "avb %i", dev + 1);
	err = snd_card_register(card);
	if (!err) {
		platform_set_drvdata(devptr, card);
	
		avbcard->sd.type = ETH_P_TSN;
		avbcard->sd.destmac[0] = 0x01;
		avbcard->sd.destmac[1] = 0x80;
		avbcard->sd.destmac[2] = 0xC2;
		avbcard->sd.destmac[3] = 0x00;
		avbcard->sd.destmac[4] = 0x00;
		avbcard->sd.destmac[5] = 0x0E;

		if(!avb_socket_init(&avbcard->sd, 100)) {
			avb_log(AVB_KERN_ERR, KERN_ERR "avb_probe socket init failed");
			err = -1;
			goto __nodev;	
		}

		return 0;
	}

	avb_log(AVB_KERN_INFO, KERN_INFO "avb_probe card reg err: %d", err);

__nodev:
	snd_card_free(card);

	return err;
}

static int avb_remove(struct platform_device *devptr)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_remove");
	snd_card_free(platform_get_drvdata(devptr));
	return 0;
}

static void avb_remove_all(void) {
	int i = 0;

	avb_log(AVB_KERN_NOT, KERN_NOTICE "avb_remove_all");

	for(i=0; i < numcards; i++)
		platform_device_unregister(avbdevices[i]);
}											

static int __init alsa_avb_init(void)
{
	int i, err;
	struct platform_device *dev;
	avb_log(AVB_KERN_NOT, KERN_NOTICE "alsa_avb_init");

	err = platform_driver_register(&avb_driver);
	if (err < 0) {
		avb_log(AVB_KERN_ERR, KERN_ERR "alsa_avb_init reg err %d", err);
		return err;
	}

	for(i=0; i < SND_AVB_NUM_CARDS; i++) {
		if(!enable[i])
			continue;

		dev = platform_device_register_simple(SND_AVB_DRIVER, i, NULL, 0);

		if (IS_ERR(dev)) {		
			avb_log(AVB_KERN_ERR, KERN_ERR "alsa_avb_init regsimple err");
			continue;
		}

		if (!platform_get_drvdata(dev)) {
			avb_log(AVB_KERN_ERR, KERN_ERR "alsa_avb_init getdrvdata err");
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
			avb_log(AVB_KERN_ERR, KERN_ERR "alsa_avb_init workqueue creation failed");
			return -1;
		}

		avbdevice.msrpwd = (struct workdata*)kmalloc(sizeof(struct workdata), GFP_KERNEL);
		if(avbdevice.msrpwd == NULL) {
			avb_log(AVB_KERN_ERR, KERN_ERR "alsa_avb_init msrp workdata allocation failed");
			return -1;
		}

		avbdevice.msrpwd->dw.msrp = &avbdevice.msrp;
		avbdevice.msrpwd->delayedWorkId = AVB_DELAY_WORK_MSRP;
		INIT_DELAYED_WORK((struct delayed_work*)avbdevice.msrpwd, avbWqFn);
				
		queue_delayed_work(avbdevice.wq, (struct delayed_work*)avbdevice.msrpwd, 100);

		avb_log(AVB_KERN_NOT, KERN_NOTICE "alsa_avb_init done err: %d, numcards: %d", err, numcards);	
	}

	return 0;
}

static void __exit alsa_avb_exit(void)
{
	avb_log(AVB_KERN_NOT, KERN_NOTICE "alsa_avb_exit");
	
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
	
	avb_log(AVB_KERN_NOT, KERN_NOTICE "alsa_avb_exit done");
}

module_init(alsa_avb_init)
module_exit(alsa_avb_exit)
