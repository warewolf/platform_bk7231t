#ifndef _MAIN_NONE_H_
#define _MAIN_NONE_H_

#include "os.h"
#include "eloop.h"

#define CFG_CONFIG_FNAME          "beken_cfg_fname"
#define CFG_BSS_CONFIG            "wangzhilei_config:bss_fname"
#define CFG_AP_IFACE_CONFIG       "bss_config= wlan0"
#define WEP40_KEY_LENGTH           10

struct hapd_global {
	void **drv_priv;
	size_t drv_count;
};

extern char *bss_iface;

void hostapd_thread_start(void);
void hostapd_thread_stop(void);
extern int hostapd_main_entry(int argc, char *argv[]);
extern int hostapd_channel_switch(int new_freq);
extern int supplicant_main_entry(char *oob_ssid);
extern int supplicant_main_exit();
extern void wpa_supplicant_poll(void *param);
int wpa_hostapd_queue_command(wpah_msg_t *msg);
int hostapd_main_exit(void);
int hostapd_main_entry(int argc, char *argv[]);
struct wpa_supplicant *wpa_suppliant_ctrl_get_wpas();
struct hapd_interfaces *hostapd_ctrl_get_interfaces();


#endif // _MAIN_NONE_H_
// eof

