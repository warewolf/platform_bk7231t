#ifndef _SYS_CONFIG_H_
#define _SYS_CONFIG_H_

#define CFG_SUPPORT_BOOTLOADER                     1

/*SUMMARY: macro--1: OPEN; --0:CLOSE*/

/* uart2 for debug, and generally, uart1 is used for communication.
   what is more, uart1 maybe is not bound out*/
#define CFG_USE_UART1                              1
#define CFG_UART2_CLI                              0
#define CFG_JTAG_ENABLE                            0
#define OSMALLOC_STATISTICAL                       0
#define CFG_REUSE_ISTACK_SPACE                     1
#define CFG_BLE_MALLOC_HEAP                        1

/*section 0-----app macro config-----*/
#define CFG_IEEE80211N                             1

/*section 1-----OS macro config-----*/
#define CFG_OS_FREERTOS                            1

#if CFG_OS_FREERTOS
#define THD_APPLICATION_PRIORITY                   3
#define THD_CORE_PRIORITY                          2
#define THD_UMP3_PRIORITY                          4
#define THD_UBG_PRIORITY                           5
#define THD_LWIP_PRIORITY                          4
#define THD_INIT_PRIORITY                          4
#define THD_RECONNECT_PRIORITY                     4
#define THD_MEDIA_PRIORITY						   4
#define THD_WPAS_PRIORITY                          5
#define THD_EXTENDED_APP_PRIORITY                  5
#define THD_HOSTAPD_PRIORITY                       5
#define THDD_KEY_SCAN_PRIORITY					   7
#endif // CFG_OS_FREERTOS

/*section 2-----function macro config-----*/
#define CFG_TX_EVM_TEST                            0
#define CFG_RX_SENSITIVITY_TEST                    0
#define CFG_ROLE_LAUNCH                            1
#define CFG_USE_WPA_29							   0
#define CFG_WPA_CTRL_IFACE						   0
#define CFG_WLAN_FAST_CONNECT                      1
/* PMF */
#define CFG_IEEE80211W							   0
#if CFG_WPA_CTRL_IFACE
#undef CFG_ROLE_LAUNCH
#define CFG_ROLE_LAUNCH							   0
#endif

#define CFG_WPA3								   1
#if CFG_WPA3
/* disable rl, enable ctrl iface, enable wpa29, pmf, sme */
#undef CFG_ROLE_LAUNCH
#define CFG_ROLE_LAUNCH                            0
#undef CFG_WPA_CTRL_IFACE
#define CFG_WPA_CTRL_IFACE						   1
#undef CFG_USE_WPA_29
#define CFG_USE_WPA_29							   1
#undef CFG_IEEE80211W
#define CFG_IEEE80211W							   1
#define CFG_SME								       1
#define CFG_WPA_CRYPTO_MBEDTLS                     1
#define CFG_WRAP_LIBC                              1
#endif

//#define CFG_MESH								   0
#define CFG_WFA_CERT							   0
#define CFG_ENABLE_BUTTON                          0
#define CFG_UDISK_MP3                              0

/*section 3-----driver macro config-----*/
#define CFG_MAC_PHY_BAPASS                         0

#define CFG_SDIO                                   0
#define CFG_SDIO_TRANS                             0
#define CFG_REAL_SDIO                              0
#if CFG_REAL_SDIO
#define FOR_SDIO_BLK_512                           0
#endif

#define CFG_MSDU_RESV_HEAD_LEN                    96
#define CFG_MSDU_RESV_TAIL_LEN                    16

#define CFG_USE_USB_HOST                           0

#define CFG_USB                                    0
#if CFG_USB
#define CFG_SUPPORT_MSD                            1
#define CFG_SUPPORT_HID                            0
#define CFG_SUPPORT_CCD                            0
#define CFG_SUPPORT_UVC                            0
#endif

/*section 4-----DEBUG macro config-----*/
#define CFG_UART_DEBUG                             0
#define CFG_SUPPORT_BKREG                          0
#define CFG_ENABLE_WPA_LOG                         0
#define CFG_IPERF_TEST                             0
#define CFG_ENABLE_DEMO_TEST                       0
#define CFG_PING_COMMAND                           0

/*section 5-----PRODUCT macro config-----*/
#define CFG_RELEASE_FIRMWARE                       0

/*section 6-----for platform*/
#define SOC_PLATFORM                               1
#define FPGA_PLATFORM                              0
#define CFG_RUNNING_PLATFORM                       SOC_PLATFORM

#define SOC_BK7231                                 1
#define SOC_BK7231U                                2
#define SOC_BK7221U                                3
#define CFG_SOC_NAME                               SOC_BK7231U

/*section 7-----calibration*/
#if (CFG_RUNNING_PLATFORM == FPGA_PLATFORM)
#define CFG_SUPPORT_CALIBRATION                    0
#define CFG_SUPPORT_MANUAL_CALI                    0
#else
#define CFG_SUPPORT_CALIBRATION                    1
#define CFG_SUPPORT_MANUAL_CALI                    1
//tpc rf pa map power for bk7231u
#define CFG_SUPPORT_TPC_PA_MAP                     1
#endif

/*section 8-----for netstack*/
#define CFG_USE_LWIP_NETSTACK                      1

/*section 9-----for DHCP servicers and client*/
#define CFG_USE_DHCP                               1

/*section 10-----patch*/
#define CFG_BK7221_MDM_WATCHDOG_PATCH              0

/*section 11-----temperature detect*/
#define CFG_USE_TEMPERATURE_DETECT                 1

/*section 12-----for video transfer*/
#define CFG_USE_APP_DEMO_VIDEO_TRANSFER            0
#define CFG_USE_HSLAVE_SPI                         0
#define CFG_USE_SPIDMA                             0
#define CFG_USE_CAMERA_INTF                        0
#define CFG_USE_IIC1_ONLY                          1

/*section 13-----for GENERRAL DMA */
#define CFG_GENERAL_DMA                            1

/*section 14-----for FTPD UPGRADE*/
#define CFG_USE_FTPD_UPGRADE                       0

/*section 15-----support customer macro*/
#define CFG_SUPPORT_TIANZHIHENG_DRONE              0

/*section 16-----support mcu & deep sleep*/
#define CFG_USE_MCU_PS                             1
#define CFG_USE_DEEP_PS                            1

/*section 17-----support sta power sleep*/
#define CFG_USE_STA_PS                             1

/*section 18-----AP support stas in power save*/
#define CFG_USE_AP_PS                              0

/*section 19-----for SDCARD HOST*/
#define CFG_USE_SDCARD_HOST                        0

/*section 20 ----- support mp3 decoder*/
#define CONFIG_APP_MP3PLAYER 			           0

/*section 21 ----- support ota*/
#define CFG_SUPPORT_OTA_HTTP                       0
#define CFG_SUPPORT_OTA_TFTP                       0

/*section 22 ----- support adc calibrate*/
#define CFG_SARADC_CALIBRATE                       0

/*section 23 ----- support reduce nomal power*/
#define CFG_SYS_REDUCE_NORMAL_POWER                0

/*section 24 ----- less memery in rwnx*/
#define CFG_LESS_MEMERY_IN_RWNX                    0

/*section 25 ----- use audio*/
#define CFG_USE_AUDIO                              0
#define CFG_USE_AUD_DAC                            0
#define CFG_USE_AUD_ADC                            0

#define CFG_RF_USER_BLE                            1
#define CFG_RF_USER_WIFI                           2
#define CFG_DEFAULT_RF_USER                        CFG_RF_USER_WIFI

#define CFG_USE_TUYA_CCA_TEST                      0

#define CFG_AP_MONITOR_COEXIST                     1
#define CFG_SUPPORT_BLE                            1
#define CFG_USE_BLE_PS                             1

#if CFG_UART2_CLI
#define CFG_ENABLE_ATE_FEATURE                     1
#else
#define CFG_ENABLE_ATE_FEATURE                     0
#endif

#define CFG_RWNX_QOS_MSDU						   1
#endif // _SYS_CONFIG_H_
