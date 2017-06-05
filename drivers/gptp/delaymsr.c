
#define DELAY_MSR_MODULE

#include "delaymsr.h"

char txBuf[GPTP_SYNCMSG_ETH_FRAME_SIZE];
char rxBuf[GPTP_SYNCMSG_ETH_FRAME_SIZE];

void initDM(struct dmst* dm)
{
	int tsOpts;
	int errCode;

	memset(dm, 0, sizeof(struct dmst));
	dm->state = DM_STATE_INIT;
	
	/* Create a socket */
	if (sock_create(AF_PACKET, SOCK_RAW, htons(ETH_P_1588), &dm->sock) < 0) {
		printk(KERN_WARNING "gPTP sendSyncMsg Socket creation fails \n");
		goto err;
	}

	/* Set timestamp options */
	tsOpts = SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_TX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE | \
		 SOF_TIMESTAMPING_SOFTWARE | SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_TX_SOFTWARE | \
		 SOF_TIMESTAMPING_OPT_CMSG | SOF_TIMESTAMPING_OPT_ID;
	if ((errCode == dm->sock->ops->setsockopt(dm->sock, SOL_SOCKET, SO_TIMESTAMPING, (void *) &tsOpts, sizeof(tsOpts))) != 0) {
		printk(KERN_WARNING "gPTP sendSyncMsg Set TS option fails %d\n", errCode);
		goto err;	
	}

	/* Index of the network device */
	dm->txSockAddress.sll_family = AF_PACKET;
	dm->txSockAddress.sll_protocol = htons(ETH_P_1588);
	dm->txSockAddress.sll_ifindex = 2;
	/* Address length*/
	dm->txSockAddress.sll_halen = ETH_ALEN;
	/* Destination MAC */
	dm->txSockAddress.sll_addr[0] = 0xFF;
	dm->txSockAddress.sll_addr[1] = 0xFF;
	dm->txSockAddress.sll_addr[2] = 0xFF;
	dm->txSockAddress.sll_addr[3] = 0xFF;
	dm->txSockAddress.sll_addr[4] = 0xFF;
	dm->txSockAddress.sll_addr[5] = 0xFF;

	/* Set the message header */
	dm->txMsgHdr.msg_control=NULL;
	dm->txMsgHdr.msg_controllen=0;
	dm->txMsgHdr.msg_flags=0;
	dm->txMsgHdr.msg_name=&dm->txSockAddress;
	dm->txMsgHdr.msg_namelen=sizeof(struct sockaddr_ll);
	dm->txMsgHdr.msg_iocb = NULL;

	/* Index of the network device */
	dm->rxSockAddress.sll_family = AF_PACKET;
	dm->rxSockAddress.sll_protocol = htons(ETH_P_ALL);
	dm->rxSockAddress.sll_ifindex = 2;
	/* Address length*/
	dm->rxSockAddress.sll_halen = ETH_ALEN;
	/* Destination MAC */
	dm->rxSockAddress.sll_addr[0] = 0x04;
	dm->rxSockAddress.sll_addr[1] = 0xA3;
	dm->rxSockAddress.sll_addr[2] = 0x16;
	dm->rxSockAddress.sll_addr[3] = 0xAD;
	dm->rxSockAddress.sll_addr[4] = 0x3A;
	dm->rxSockAddress.sll_addr[5] = 0x33;

	/* Set the message header */
	dm->rxMsgHdr.msg_control=dm->tsBuf;
	dm->rxMsgHdr.msg_controllen=GPTP_SYNCMSG_ETH_FRAME_SIZE;
	dm->rxMsgHdr.msg_flags=0;
	dm->rxMsgHdr.msg_name=&dm->rxSockAddress;
	dm->rxMsgHdr.msg_namelen=sizeof(struct sockaddr_ll);
	dm->rxMsgHdr.msg_iocb = NULL;

err:
	return;
}

void unintDM(struct dmst* dm)
{
	sock_release(dm->sock);
}

void dmHandleEvent(struct dmst* dm, int evtId)
{
	printk(KERN_INFO "gPTP dmHandleEvent st: %d evt: %d \n", dm->state, evtId);
	
	switch(dm->state) {

		case DM_STATE_INIT:
		case DM_STATE_DELAY_REQ_TS:
	
			switch (evtId) {
				case GPTP_EVT_ONE_SEC_TICK:
					if(sendSyncMsg(dm) >= 0)
						dmHandleStateChange(dm, DM_STATE_DELAY_REQ_TX);
					break;
				default:
					break;
			}

			break;

		case DM_STATE_DELAY_REQ_TX:
	
			switch (evtId) {
				case GPTP_EVT_ONE_SEC_TICK:
					if(getDelayReqTS(dm) >= 0) 
						dmHandleStateChange(dm, DM_STATE_DELAY_REQ_TS);
					break;
				default:
					break;
			}

			break;
	}
}

void dmHandleStateChange(struct dmst* dm, int toState)
{
	dm->state = toState;
}

static int sendSyncMsg(struct dmst* dm)
{
	int res = -1;
	int dataOff;
	int errCode;
	struct ethhdr *eh = (struct ethhdr *)txBuf;
	struct iovec txiov;
	mm_segment_t oldfs;

	/* Initialize it */
	memset(txBuf, 0, GPTP_SYNCMSG_ETH_FRAME_SIZE);

	/* Fill in the Ethernet header */
	eh->h_dest[0] = 0xFF;
	eh->h_dest[1] = 0xFF;
	eh->h_dest[2] = 0xFF;
	eh->h_dest[3] = 0xFF;
	eh->h_dest[4] = 0xFF;
	eh->h_dest[5] = 0xFF;
	eh->h_source[0] = 0x04;
	eh->h_source[1] = 0xA3;
	eh->h_source[2] = 0x16;
	eh->h_source[3] = 0xAD;
	eh->h_source[4] = 0x3A;
	eh->h_source[5] = 0x33;

	/* Fill in Ethertype field */
	eh->h_proto = htons(ETH_P_1588);

	/* Get data offset */
	dataOff = sizeof(struct ethhdr);

	/* Fill payload data */
	txBuf[dataOff++] = 0x33;
	txBuf[dataOff++] = 0x66;
	txBuf[dataOff++] = 0x99;

	/* Set the output buffer */
	txiov.iov_base = txBuf;
	txiov.iov_len = dataOff;
	iov_iter_init(&dm->txMsgHdr.msg_iter, WRITE | ITER_KVEC, &txiov, 1, dataOff);

	oldfs=get_fs();
	set_fs(KERNEL_DS);

	/* Send packet */
	if ((errCode = sock_sendmsg(dm->sock, &dm->txMsgHdr)) <= 0) {
		printk(KERN_WARNING "gPTP sendSyncMsg Socket transmission fails %d \n", errCode);
		goto fserr;
	} else {
		res = 0;
		printk(KERN_WARNING "gPTP sendSyncMsg Socket transmission success \n");
	}
	
fserr:
	set_fs(oldfs);

	return res;		
}

static int getDelayReqTS(struct dmst* dm)
{
	int res = -1;
	int errCode;
	struct iovec rxiov;
	struct cmsghdr *cmsghdr;
	struct cmsghdr *pcmsghdr;
	int rxcount = 0;
	int cmsgcount = 0;

	/* Set the output buffer */
	rxiov.iov_base = rxBuf;
	rxiov.iov_len = GPTP_SYNCMSG_ETH_FRAME_SIZE;
	iov_iter_init(&dm->rxMsgHdr.msg_iter, READ | ITER_KVEC, &rxiov, 1, GPTP_SYNCMSG_ETH_FRAME_SIZE);

	/* Destination MAC */
	dm->rxSockAddress.sll_addr[0] = 0xFF;
	dm->rxSockAddress.sll_addr[1] = 0xFF;
	dm->rxSockAddress.sll_addr[2] = 0xFF;
	dm->rxSockAddress.sll_addr[3] = 0xFF;
	dm->rxSockAddress.sll_addr[4] = 0xFF;
	dm->rxSockAddress.sll_addr[5] = 0xFF;	
	
	/* Receive packet */
	rxcount = 0;
	if (((errCode = sock_recvmsg(dm->sock, &dm->rxMsgHdr, GPTP_SYNCMSG_ETH_FRAME_SIZE, 0)) > 0) && (rxcount < 10)) {
		res = 0;
		rxcount++;
		printk(KERN_WARNING "gPTP getDelayReqTS Socket reception success with msgsize %d cmsgsize %d \n", errCode, dm->rxMsgHdr.msg_controllen);
		cmsgcount = 0;
		cmsghdr = CMSG_FIRSTHDR(&dm->rxMsgHdr);
		while ((cmsghdr) && (cmsgcount < 3)) {
			printk(KERN_WARNING "gPTP getDelayReqTS Cmsg lvl:%d type:%d len:%d", cmsghdr->cmsg_level, cmsghdr->cmsg_type, cmsghdr->cmsg_len);
			cmsgcount++;
			pcmsghdr = cmsghdr;
			cmsghdr = CMSG_NXTHDR(&dm->rxMsgHdr, pcmsghdr);	
		}
	} else {
		printk(KERN_WARNING "gPTP getDelayReqTS Socket reception fails %d \n", errCode);
	}

	/* Receive error packet */
	rxcount = 0;
	if (((errCode = sock_recvmsg(dm->sock, &dm->rxMsgHdr, GPTP_SYNCMSG_ETH_FRAME_SIZE, MSG_ERRQUEUE)) > 0) && (rxcount < 10)) {
		res = 0;
		rxcount++;
		printk(KERN_WARNING "gPTP getDelayReqTS Err Socket reception success with msgsize %d cmsgsize %d \n", errCode, dm->rxMsgHdr.msg_controllen);
		cmsghdr = CMSG_FIRSTHDR(&dm->rxMsgHdr);
		cmsgcount = 0;
		cmsghdr = CMSG_FIRSTHDR(&dm->rxMsgHdr);
		while ((cmsghdr) && (cmsgcount < 3)) {
			printk(KERN_WARNING "gPTP getDelayReqTS Err Cmsg lvl:%d type:%d len:%d", cmsghdr->cmsg_level, cmsghdr->cmsg_type, cmsghdr->cmsg_len);
			cmsgcount++;
			pcmsghdr = cmsghdr;
			cmsghdr = CMSG_NXTHDR(&dm->rxMsgHdr, pcmsghdr);	
		}
	} else {
		printk(KERN_WARNING "gPTP getDelayReqTS Err Socket reception fails %d \n", errCode);
	}

	return res;
}

