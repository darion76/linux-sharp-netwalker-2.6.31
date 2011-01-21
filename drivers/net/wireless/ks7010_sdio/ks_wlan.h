/*
 *
 *   Driver for KeyStream IEEE802.11 b/g wireless LAN cards.
 *   
 *   ks_wlan.h
 *   $Id: ks_wlan.h 822 2009-01-27 10:43:23Z sekine $
 *
 *   Copyright (c) 2006-2008 KeyStream Corp.
 *   All rights reserved.
 *
 *   2011/01/20 ported to kernel 2.6.31 by Andrey Zhornyak darion76@gmail.com
 *
 */

#ifndef _KS_WLAN_H
#define _KS_WLAN_H

#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)) 
#include <linux/config.h>
#endif
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/spinlock.h>	/* spinlock_t					*/
#include <linux/sched.h>	/* wait_queue_head_t				*/
#include <linux/types.h>	/* pid_t					*/
#include <linux/netdevice.h>	/* struct net_device_stats,  struct sk_buff	*/
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <asm/atomic.h> 	/* struct atmic_t				*/
#include <linux/timer.h>	/* struct timer_list */
#include <linux/string.h>
#include <linux/completion.h>   /* struct completion */

#include <asm/io.h>

/* Workqueue / task queue backwards compatibility stuff */
#if ((LINUX_VERSION_CODE > KERNEL_VERSION(2,5,41)) || (defined _MVL31_) || (defined _CELF3_))
#include <linux/workqueue.h>
#else
#include <linux/tqueue.h>
#define work_struct tq_struct
#define INIT_WORK INIT_TQUEUE
#define schedule_work schedule_task
#endif

/* Interrupt handler backwards compatibility stuff */
#ifndef IRQ_NONE
#define IRQ_NONE
#define IRQ_HANDLED
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)
typedef void irqreturn_t;
#endif
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,23)
#define free_netdev(x) kfree(x) 
#define pci_name(x) x->slot_name 
#endif

#if (defined _PCMCIA_)
#include "pcmcia/ks7010_pcmcia.h"
#elif (defined _PCI_)
#include "pci/ks7010_pci.h"
#elif (defined _SDIO_)
#include "sdio/ks7010_sdio.h"
#elif (defined _SPI_)
#include "spi/ks7010_spi.h"
#else
#error not defined bus type !
#endif

struct ks_wlan_parameter {
	uint8_t		operation_mode;	   /* Operation Mode */
	uint8_t		channel;	   /*  Channel */
	uint8_t		tx_rate;	   /*  Transmit Rate */
	struct {
		uint8_t size;
		uint8_t body[16];
	} rate_set;
	uint8_t		bssid[ETH_ALEN];	/* BSSID */
	struct {
		uint8_t size;
		uint8_t body[32+1];
	} ssid;				   /*  SSID */
	uint8_t		preamble;	   /*  Preamble */
	uint8_t		powermgt;	   /*  PowerManagementMode */
	uint32_t	scan_type;         /*  AP List Scan Type */
#define BEACON_LOST_COUNT_MIN 0
#define BEACON_LOST_COUNT_MAX 65535
	uint32_t	beacon_lost_count; /*  Beacon Lost Count */
	uint32_t	rts;		   /*  RTS Threashold */
	uint32_t	fragment;	   /*  Fragmentation Threashold */
	uint32_t	privacy_invoked;
	uint32_t	wep_index;
	struct {
		uint8_t size;
		uint8_t val[13*2+1];
	} wep_key[4];
	uint16_t	authenticate_type;	
	uint16_t	phy_type; /* 11b/11g/11bg mode type*/
	uint16_t	cts_mode; /* for 11g/11bg mode cts mode */
	uint16_t	phy_info_timer; /* phy information timer */
	char		rom_file[256];
};

enum {
	DEVICE_STATE_OFF = 0,	/* this means hw_unavailable is != 0 */
	DEVICE_STATE_PREBOOT,	/* we are in a pre-boot state (empty RAM) */
	DEVICE_STATE_BOOT,	/* boot state (fw upload, run fw) */
	DEVICE_STATE_PREINIT,	/* pre-init state */
	DEVICE_STATE_INIT,	/* init state (restore MIB backup to device) */
	DEVICE_STATE_READY,	/* driver&device are in operational state */
	DEVICE_STATE_SLEEP	/* device in sleep mode */
};

/* SME flag */
#define SME_MODE_SET	    (1<<0)
#define SME_RTS             (1<<1)
#define SME_FRAG            (1<<2)
#define SME_WEP_FLAG        (1<<3)
#define SME_WEP_INDEX       (1<<4)
#define SME_WEP_VAL1        (1<<5)
#define SME_WEP_VAL2        (1<<6)
#define SME_WEP_VAL3        (1<<7)
#define SME_WEP_VAL4        (1<<8)
#define SME_WEP_VAL_MASK    (SME_WEP_VAL1|SME_WEP_VAL2|SME_WEP_VAL3|SME_WEP_VAL4)
#define SME_RSN             (1<<9)
#define SME_RSN_MULTICAST   (1<<10)
#define SME_RSN_UNICAST	    (1<<11)
#define SME_RSN_AUTH	    (1<<12)

#define SME_AP_SCAN         (1<<13)
#define SME_MULTICAST       (1<<14)

/* SME Event */
enum {
	SME_START,

	SME_MULTICAST_REQUEST,
	SME_MACADDRESS_SET_REQUEST,
	SME_BSS_SCAN_REQUEST,
	SME_SET_FLAG,
	SME_SET_TXKEY,
	SME_SET_KEY1,
	SME_SET_KEY2,
	SME_SET_KEY3,
	SME_SET_KEY4,
	SME_SET_PMK_TSC,
	SME_SET_GMK1_TSC,
	SME_SET_GMK2_TSC,
	SME_SET_GMK3_TSC,
	SME_SET_PMKSA,
	SME_POW_MNGMT_REQUEST,
	SME_PHY_INFO_REQUEST,
	SME_MIC_FAILURE_REQUEST,
	SME_GET_MAC_ADDRESS,
	SME_GET_PRODUCT_VERSION,
	SME_STOP_REQUEST,
	SME_RTS_THRESHOLD_REQUEST,
	SME_FRAGMENTATION_THRESHOLD_REQUEST,
	SME_WEP_INDEX_REQUEST,
	SME_WEP_KEY1_REQUEST,
	SME_WEP_KEY2_REQUEST,
	SME_WEP_KEY3_REQUEST,
	SME_WEP_KEY4_REQUEST,
	SME_WEP_FLAG_REQUEST,
	SME_RSN_UCAST_REQUEST,
	SME_RSN_MCAST_REQUEST,
	SME_RSN_AUTH_REQUEST,
	SME_RSN_ENABLED_REQUEST,
	SME_RSN_MODE_REQUEST,

	SME_SET_GAIN,
	SME_GET_GAIN,
	SME_SLEEP_REQUEST,
	SME_MODE_SET_REQUEST,
	SME_START_REQUEST,


	SME_MIC_FAILURE_CONFIRM,
	SME_START_CONFIRM,

	SME_MULTICAST_CONFIRM,
	SME_BSS_SCAN_CONFIRM,
	SME_GET_CURRENT_AP,
	SME_POW_MNGMT_CONFIRM,
	SME_PHY_INFO_CONFIRM,
	SME_STOP_CONFIRM,
	SME_RTS_THRESHOLD_CONFIRM,
	SME_FRAGMENTATION_THRESHOLD_CONFIRM,
	SME_WEP_INDEX_CONFIRM,
	SME_WEP_KEY1_CONFIRM,
	SME_WEP_KEY2_CONFIRM,
	SME_WEP_KEY3_CONFIRM,
	SME_WEP_KEY4_CONFIRM,
	SME_WEP_FLAG_CONFIRM,
	SME_RSN_UCAST_CONFIRM,
	SME_RSN_MCAST_CONFIRM,
	SME_RSN_AUTH_CONFIRM,
	SME_RSN_ENABLED_CONFIRM,
	SME_RSN_MODE_CONFIRM,
	SME_MODE_SET_CONFIRM,
	SME_SLEEP_CONFIRM,

	SME_RSN_SET_CONFIRM,
	SME_WEP_SET_CONFIRM,
	SME_TERMINATE,

	SME_EVENT_SIZE
};

/* SME Status */
enum {
	SME_IDLE,
	SME_SETUP,
	SME_DISCONNECT,
	SME_CONNECT
};

#define	SME_EVENT_BUFF_SIZE	128

struct	sme_info{
	int	sme_status;
	int	event_buff[SME_EVENT_BUFF_SIZE];
	unsigned int	qhead;
	unsigned int	qtail;
#ifdef KS_WLAN_DEBUG
  /* for debug */
	unsigned int max_event_count;
#endif
	spinlock_t    sme_spin;
	unsigned long sme_flag;
};

struct	hostt_t{
	int	buff[SME_EVENT_BUFF_SIZE];
	unsigned int	qhead;
	unsigned int	qtail;
};

#define RSN_IE_BODY_MAX 64
struct rsn_ie_t {
	uint8_t	id; /* 0xdd = WPA or 0x30 = RSN */
	uint8_t	size; /* max ? 255 ? */
	uint8_t	body[RSN_IE_BODY_MAX];
} __attribute__((packed));

struct local_ap_t {
	uint8_t	bssid[6];
	uint8_t	rssi;
	uint8_t	sq;
	struct {
		uint8_t	size;
		uint8_t	body[32];
		uint8_t	ssid_pad;
	} ssid;
	struct {
		uint8_t	size;
		uint8_t	body[16];
		uint8_t	rate_pad;
	} rate_set;
	uint16_t	capability;
	uint8_t	channel;
	uint8_t	noise;
	struct rsn_ie_t wpa_ie;
	struct rsn_ie_t rsn_ie;
};

#define LOCAL_APLIST_MAX 31
#define LOCAL_CURRENT_AP LOCAL_APLIST_MAX
struct local_aplist_t {
	int size;
	struct local_ap_t ap[LOCAL_APLIST_MAX+1];
};

struct local_gain_t{
	uint8_t	TxMode;
	uint8_t	RxMode;
	uint8_t	TxGain;
	uint8_t	RxGain;
};

/* Power Save Status */
enum {
	PS_NONE,
	PS_ACTIVE_SET,
	PS_SAVE_SET,
	PS_CONF_WAIT,
	PS_SNOOZE,
	PS_WAKEUP
};

struct power_save_status_t {
        atomic_t	  status;      	/* initialvalue 0 */
	struct completion wakeup_wait;
	atomic_t	  confirm_wait;
	atomic_t	  snooze_guard;
};

struct sleep_status_t {
        atomic_t	  status;      	/* initialvalue 0 */
	atomic_t	  doze_request;
	atomic_t	  wakeup_request;
};

/* WPA */
struct scan_ext_t {
	unsigned int flag;
	char ssid[IW_ESSID_MAX_SIZE+1];
};

enum {
	CIPHER_NONE,
	CIPHER_WEP40,
	CIPHER_TKIP,
	CIPHER_CCMP,
	CIPHER_WEP104
};

#define CIPHER_ID_WPA_NONE    "\x00\x50\xf2\x00"
#define CIPHER_ID_WPA_WEP40   "\x00\x50\xf2\x01"
#define CIPHER_ID_WPA_TKIP    "\x00\x50\xf2\x02"
#define CIPHER_ID_WPA_CCMP    "\x00\x50\xf2\x04"
#define CIPHER_ID_WPA_WEP104  "\x00\x50\xf2\x05"

#define CIPHER_ID_WPA2_NONE   "\x00\x0f\xac\x00"
#define CIPHER_ID_WPA2_WEP40  "\x00\x0f\xac\x01"
#define CIPHER_ID_WPA2_TKIP   "\x00\x0f\xac\x02"
#define CIPHER_ID_WPA2_CCMP   "\x00\x0f\xac\x04"
#define CIPHER_ID_WPA2_WEP104 "\x00\x0f\xac\x05"

#define CIPHER_ID_LEN    4

enum { 
	KEY_MGMT_802_1X,
	KEY_MGMT_PSK,
	KEY_MGMT_WPANONE,
};

#define KEY_MGMT_ID_WPA_NONE     "\x00\x50\xf2\x00"
#define KEY_MGMT_ID_WPA_1X       "\x00\x50\xf2\x01"
#define KEY_MGMT_ID_WPA_PSK      "\x00\x50\xf2\x02"
#define KEY_MGMT_ID_WPA_WPANONE  "\x00\x50\xf2\xff"

#define KEY_MGMT_ID_WPA2_NONE    "\x00\x0f\xac\x00"
#define KEY_MGMT_ID_WPA2_1X      "\x00\x0f\xac\x01"
#define KEY_MGMT_ID_WPA2_PSK     "\x00\x0f\xac\x02"
#define KEY_MGMT_ID_WPA2_WPANONE "\x00\x0f\xac\xff"

#define KEY_MGMT_ID_LEN  4

#define MIC_KEY_SIZE 8

struct wpa_key_t {
	uint32_t	ext_flags; /* IW_ENCODE_EXT_xxx */
	uint8_t	tx_seq[IW_ENCODE_SEQ_MAX_SIZE]; /* LSB first */
	uint8_t	rx_seq[IW_ENCODE_SEQ_MAX_SIZE]; /* LSB first */
	struct sockaddr	addr; /* ff:ff:ff:ff:ff:ff for broadcast/multicast
			       * (group) keys or unicast address for
			       * individual keys */
	uint16_t	alg;
	uint16_t	key_len; /* WEP: 5 or 13, TKIP: 32, CCMP: 16 */
	uint8_t	key_val[IW_ENCODING_TOKEN_MAX];
	uint8_t	tx_mic_key[MIC_KEY_SIZE];
	uint8_t	rx_mic_key[MIC_KEY_SIZE];
};
#define WPA_KEY_INDEX_MAX 4
#define WPA_RX_SEQ_LEN 6

struct mic_failure_t {
	uint16_t	failure; /* MIC Failure counter 0 or 1 or 2 */
	uint16_t	counter; /* 1sec counter 0-60 */
	uint32_t	last_failure_time;
	int stop; /* stop flag */
};

struct wpa_status_t {
	int wpa_enabled;
	unsigned int rsn_enabled;
	int version;
	int pairwise_suite;	/* unicast cipher */
	int group_suite;	/* multicast cipher */
	int key_mgmt_suite;	/* authentication key management suite */
	int auth_alg;
	int txkey;
	struct wpa_key_t key[WPA_KEY_INDEX_MAX];
	struct scan_ext_t scan_ext;
	struct mic_failure_t mic_failure;
};

#include <linux/list.h>
#define PMK_LIST_MAX 8
struct pmk_list_t {
	uint16_t size;
	struct list_head head;
	struct pmk_t {
		struct list_head list;
		uint8_t	bssid[ETH_ALEN];
		uint8_t	pmkid[IW_PMKID_LEN];
	} pmk[PMK_LIST_MAX];
};


typedef struct ks_wlan_private{

	struct hw_info_t ks_wlan_hw;  /* hardware information */

	struct net_device *net_dev;
	int reg_net; /* register_netdev */
	struct net_device_stats nstats;
	struct iw_statistics wstats;

	struct completion confirm_wait;

        /* trx device & sme */
	struct tx_device tx_dev;
	struct rx_device rx_dev;
	struct sme_info  sme_i;
	u8 *rxp;
	unsigned int  rx_size;
	struct tasklet_struct sme_task;
	struct work_struct ks_wlan_wakeup_task;
	int scan_ind_count;

	unsigned char eth_addr[ETH_ALEN];

	struct local_aplist_t aplist;
	struct local_ap_t current_ap;
	struct power_save_status_t psstatus;
	struct sleep_status_t sleepstatus;
	struct wpa_status_t wpa;
	struct pmk_list_t pmklist;
        /* wireless parameter */
	struct ks_wlan_parameter reg;
	uint8_t current_rate;

	char nick[IW_ESSID_MAX_SIZE+1];
	
        spinlock_t multicast_spin;

	spinlock_t dev_read_lock;
        wait_queue_head_t devread_wait;

	unsigned int need_commit; /* for ioctl */

        /* DeviceIoControl */
	int device_open_status;
	atomic_t event_count;
        atomic_t rec_count;
        int dev_count; 
#define DEVICE_STOCK_COUNT 20
	unsigned char *dev_data[DEVICE_STOCK_COUNT];
	int dev_size[DEVICE_STOCK_COUNT];

        /* ioctl : IOCTL_FIRMWARE_VERSION */
	unsigned char firmware_version[128+1];
	int version_size;

	int mac_address_valid; /* Mac Address Status */

	int dev_state;

	struct sk_buff *skb;
	unsigned int cur_rx;	/* Index into the Rx buffer of next Rx pkt. */
	/* spinlock_t lock; */
#define FORCE_DISCONNECT    0x80000000
#define CONNECT_STATUS_MASK 0x7FFFFFFF
	uint32_t connect_status;    /* connect status */
	int infra_status;  /* Infractructure status */

        uint8_t data_buff[0x1000];

	uint8_t scan_ssid_len;
	uint8_t scan_ssid[IW_ESSID_MAX_SIZE+1];
	struct local_gain_t gain;

 	uint8_t sleep_mode;

	struct hostt_t  hostt;
	struct net_device_ops *netdev_ops;
} ks_wlan_private; 



#endif /* _KS_WLAN_H */
