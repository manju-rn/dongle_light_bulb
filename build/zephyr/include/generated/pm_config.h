/* File generated by E:/nRF/v1.7.1/nrf/scripts/partition_manager_output.py, do not modify */
#ifndef PM_CONFIG_H__
#define PM_CONFIG_H__
#define PM_NRF5_MBR_OFFSET 0x0
#define PM_NRF5_MBR_ADDRESS 0x0
#define PM_NRF5_MBR_END_ADDRESS 0x1000
#define PM_NRF5_MBR_SIZE 0x1000
#define PM_NRF5_MBR_NAME nrf5_mbr
#define PM_NRF5_MBR_ID 0
#define PM_nrf5_mbr_ID PM_NRF5_MBR_ID
#define PM_nrf5_mbr_IS_ENABLED 1
#define PM_0_LABEL NRF5_MBR
#define PM_NRF5_MBR_DEV_NAME "NRF_FLASH_DRV_NAME"
#define PM_APP_OFFSET 0x1000
#define PM_APP_ADDRESS 0x1000
#define PM_APP_END_ADDRESS 0xd5000
#define PM_APP_SIZE 0xd4000
#define PM_APP_NAME app
#define PM_APP_ID 1
#define PM_app_ID PM_APP_ID
#define PM_app_IS_ENABLED 1
#define PM_1_LABEL APP
#define PM_APP_DEV_NAME "NRF_FLASH_DRV_NAME"
#define PM_ZBOSS_NVRAM_OFFSET 0xd5000
#define PM_ZBOSS_NVRAM_ADDRESS 0xd5000
#define PM_ZBOSS_NVRAM_END_ADDRESS 0xdd000
#define PM_ZBOSS_NVRAM_SIZE 0x8000
#define PM_ZBOSS_NVRAM_NAME zboss_nvram
#define PM_ZBOSS_NVRAM_ID 2
#define PM_zboss_nvram_ID PM_ZBOSS_NVRAM_ID
#define PM_zboss_nvram_IS_ENABLED 1
#define PM_2_LABEL ZBOSS_NVRAM
#define PM_ZBOSS_NVRAM_DEV_NAME "NRF_FLASH_DRV_NAME"
#define PM_ZBOSS_PRODUCT_CONFIG_OFFSET 0xdd000
#define PM_ZBOSS_PRODUCT_CONFIG_ADDRESS 0xdd000
#define PM_ZBOSS_PRODUCT_CONFIG_END_ADDRESS 0xde000
#define PM_ZBOSS_PRODUCT_CONFIG_SIZE 0x1000
#define PM_ZBOSS_PRODUCT_CONFIG_NAME zboss_product_config
#define PM_ZBOSS_PRODUCT_CONFIG_ID 3
#define PM_zboss_product_config_ID PM_ZBOSS_PRODUCT_CONFIG_ID
#define PM_zboss_product_config_IS_ENABLED 1
#define PM_3_LABEL ZBOSS_PRODUCT_CONFIG
#define PM_ZBOSS_PRODUCT_CONFIG_DEV_NAME "NRF_FLASH_DRV_NAME"
#define PM_SETTINGS_STORAGE_OFFSET 0xde000
#define PM_SETTINGS_STORAGE_ADDRESS 0xde000
#define PM_SETTINGS_STORAGE_END_ADDRESS 0xe0000
#define PM_SETTINGS_STORAGE_SIZE 0x2000
#define PM_SETTINGS_STORAGE_NAME settings_storage
#define PM_SETTINGS_STORAGE_ID 4
#define PM_settings_storage_ID PM_SETTINGS_STORAGE_ID
#define PM_settings_storage_IS_ENABLED 1
#define PM_4_LABEL SETTINGS_STORAGE
#define PM_SETTINGS_STORAGE_DEV_NAME "NRF_FLASH_DRV_NAME"
#define PM_EMPTY_0_OFFSET 0xe0000
#define PM_EMPTY_0_ADDRESS 0xe0000
#define PM_EMPTY_0_END_ADDRESS 0x100000
#define PM_EMPTY_0_SIZE 0x20000
#define PM_EMPTY_0_NAME EMPTY_0
#define PM_EMPTY_0_ID 5
#define PM_empty_0_ID PM_EMPTY_0_ID
#define PM_empty_0_IS_ENABLED 1
#define PM_5_LABEL EMPTY_0
#define PM_EMPTY_0_DEV_NAME "NRF_FLASH_DRV_NAME"
#define PM_EMPTY_1_OFFSET 0x0
#define PM_EMPTY_1_ADDRESS 0x20000000
#define PM_EMPTY_1_END_ADDRESS 0x20000400
#define PM_EMPTY_1_SIZE 0x400
#define PM_EMPTY_1_NAME EMPTY_1
#define PM_SRAM_PRIMARY_OFFSET 0x400
#define PM_SRAM_PRIMARY_ADDRESS 0x20000400
#define PM_SRAM_PRIMARY_END_ADDRESS 0x20040000
#define PM_SRAM_PRIMARY_SIZE 0x3fc00
#define PM_SRAM_PRIMARY_NAME sram_primary
#define PM_NUM 6
#define PM_ALL_BY_SIZE "EMPTY_1 nrf5_mbr zboss_product_config settings_storage zboss_nvram EMPTY_0 sram_primary app"
#define PM_ADDRESS 0x1000
#define PM_SIZE 0xd4000
#define PM_SRAM_ADDRESS 0x20000400
#define PM_SRAM_SIZE 0x3fc00
#endif /* PM_CONFIG_H__ */