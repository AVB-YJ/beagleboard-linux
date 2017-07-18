
#define SND_AVB_DRIVER		"snd_avb"

#define ETH_MSRP                (0x22EA)

#define SND_AVB_NUM_CARDS	(SNDRV_CARDS)

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

#define AVB_WQ "AVBWQ"

#define AVB_MSRP_ETH_FRAME_SIZE                         (512)

#define MSRP_ATTRIBUTE_TYPE_TALKER_ADVERTISE_VECTOR	(1)
#define MSRP_ATTRIBUTE_TYPE_TALKER_FAILED_VECTOR	(2)
#define MSRP_ATTRIBUTE_TYPE_LISTENER_VECTOR		(3)
#define MSRP_ATTRIBUTE_TYPE_DOMAIN_VECTOR		(4)

#define MSRP_ATTRIBUTE_LEN_TALKER_ADVERTISE_VECTOR	(25)
#define MSRP_ATTRIBUTE_LEN_TALKER_FAILED_VECTOR		(34)
#define MSRP_ATTRIBUTE_LEN_LISTENER_VECTOR		(8)
#define MSRP_ATTRIBUTE_LEN_DOMAIN_VECTOR		(4)

#define MSRP_MAX_FRAME_SIZE_48KHZ_AUDIO                 (80)

#define MSRP_MAX_INTERVAL_FRAME_48KHZ_AUDIO             (1)

#define MSRP_ATTRIBUTE_EVENT_NEW                        (0)
#define MSRP_ATTRIBUTE_EVENT_JOININ                     (1)
#define MSRP_ATTRIBUTE_EVENT_IN                         (2)
#define MSRP_ATTRIBUTE_EVENT_JOINMT                     (3)
#define MSRP_ATTRIBUTE_EVENT_MT                         (4)
#define MSRP_ATTRIBUTE_EVENT_LEAVE                      (5)

#define MSRP_DECLARATION_STATE_NONE                     (0)
#define MSRP_DECLARATION_STATE_IGNORE                   (0)
#define MSRP_DECLARATION_STATE_ASKING_FAILED            (1)
#define MSRP_DECLARATION_STATE_READY                    (2)
#define MSRP_DECLARATION_STATE_READY_FAILED             (3)

#define MSRP_THREE_PACK(a, b, c) (u8)((((a * 6) + b) * 6) + c)
#define MSRP_FOUR_PACK(a, b, c, d) (u8)((a * 64) + (b * 16) + (c * 4) + (d))

#define AVTP_PDU_COMMON_STREAM_HEADER_LENGTH            (24)
#define AVTP_PDU_COMMON_CONTROL_HEADER_LENGTH           (12)

typedef signed long long int s64;
typedef signed int s32;
typedef signed short int s16;
typedef signed char s8;
typedef unsigned long long int u64;
typedef unsigned int u32;
typedef unsigned short int u16;
typedef unsigned char u8;

#pragma pack(push, 1)

struct avtPduAafPcmHdr {
	union th {
		struct tf {
			u8 subType;
			union tb1 {
				u8 sv;		/* 1 bit stream valid indication */
				u8 version;	/* 3 bits version */
				u8 mr;		/* 1 bit media clock restart */
				u8 rsv;		/* 2 bits reserved */
				u8 tsValid;	/* 1 bit timestamp valid */
			} b1;
			u8 seqNo;
			union tb2 {
				u8 rsv;		/* 7 bit reserved data */
				u8 tu;		/* 1 bit timestamp uncertain */	
			} b2;
			u64 streamId;
			u32 avtpTS;
			u8 format;
			union tfsd1 {
				u8 nsr;		/* 4 bits nominal sample rate */
				u8 rsv;         /* 2 bits reserved data */
				u8 cpf;         /* first 2 bits of channels per frame */
			} fsd1;
			u8 cpf;                 /* last 8 bits of channels per frame */
			u8 bitDepth;
			u16 streamDataLen;
			union tfsd2 {
				u8 rsv;		/* 3 bits reserved data */
				u8 sp;		/* 1 bit sparse timestamp */
				u8 evt;		/* 4 bits event data */
			} fsd2;
			u8 rsv;
		} f;
		u8 bytes[AVTP_PDU_COMMON_STREAM_HEADER_LENGTH];
	} h;
};

struct listenermsrpfirstvalue {
	u8 streamid[8];
};

struct talkermsrpfirstvalue {
	u8 streamid[8];
	u8 dataframeparams[8];
	u16 maxFrameSize;
	u16 maxintervalframes;
	u8 priorityandrank;
	u32 accumalatedlatency;
};

struct bridgemsrpfirstvalue {
	u8 streamid[8];
	u8 dataframeparams[8];
	u16 maxFrameSize;
	u16 maxintervalframes;
	u8 priorityandrank;
	u32 accumalatedlatency;
	u8 bridgeid[8];
	u8 failurereason; 
};

struct vectorheader {
	u16 numberofvalues;
};

struct listnervectorattribute {
	struct vectorheader hdr;
	struct listenermsrpfirstvalue val;
	u8 vector[2];
};

struct talkervectorattribute {
	struct vectorheader hdr;
	struct talkermsrpfirstvalue val;
	u8 vector[1];
};

struct listnermrpmsg {
	u8 attributetype;
	u8 attributelen;
	u16 attributelistlen;
	struct listnervectorattribute attibutelist;
	u16 endmarker;
};

struct talkermrpmsg {
	u8 attributetype;
	u8 attributelen;
	u16 attributelistlen;
	struct talkervectorattribute attibutelist;
	u16 endmarker;
};

struct listnermsrpdu {
	u8 protocolversion;
	struct listnermrpmsg msg;
	u16 endmarker;
};

struct talkermsrpdu {
	u8 protocolversion;
	struct talkermrpmsg msg;
	u16 endmarker;
};

#pragma pack(pop)
 
struct msrp {
	bool initialized;
	int  talkerState;
	int  listenerState;
	struct socket* sock;
	struct ifreq if_mac;
	struct ifreq if_idx;
	struct iovec txiov;
	struct iovec rxiov;
	struct msghdr txMsgHdr;
	struct sockaddr_ll txSockAddress;
	struct msghdr rxMsgHdr;
	struct sockaddr_ll rxSockAddress;
	char txBuf[AVB_MSRP_ETH_FRAME_SIZE];
	char rxBuf[AVB_MSRP_ETH_FRAME_SIZE];
	u8 streamid[8];
}; 

struct workdata {
	struct delayed_work work;
	struct msrp* msrp;
};

struct avbdevice {
	struct msrp msrp;
	struct workdata* wd;
	struct workqueue_struct* wq;
};

struct avbcard {
	struct snd_card *card;
	struct snd_pcm *pcm[1];
};


static int avb_open(struct snd_pcm_substream *substream);
static int avb_prepare(struct snd_pcm_substream *substream);
static int avb_trigger(struct snd_pcm_substream *substream, int cmd);
static snd_pcm_uframes_t avb_pointer(struct snd_pcm_substream *substream);
static int avb_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params);
static int avb_hw_free(struct snd_pcm_substream *substream);
static int avb_close(struct snd_pcm_substream *substream);

static bool avb_msrp_init(struct msrp* msrp);
static int avb_msrp_evaluateTalkerAdvertisement(struct msrp* msrp);
static void avb_msrp_evaluateListnerAdvertisement(struct msrp* msrp);
static void avb_msrp_talkerdeclarations(struct msrp* msrp, bool join);
static void avb_msrp_listenerdeclarations(struct msrp* msrp, bool join, int state);
static void avb_msrp_listen(struct msrp* msrp);

static void avbWqFn(struct work_struct *work);

static int avb_pcm_new(struct avbcard *avbc, int device, int substreams);

#ifdef CONFIG_PM_SLEEP

static int avb_suspend(struct device *pdev);
static int avb_resume(struct device *pdev);

#endif

static int avb_probe(struct platform_device *devptr);
static int avb_remove(struct platform_device *devptr);
static void avb_remove_all(void);
static int __init alsa_avb_init(void);
static void __exit alsa_avb_exit(void);


