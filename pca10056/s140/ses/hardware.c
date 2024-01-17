/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author: Om Trivedi and Brijesh Vaghasiya
 *
 *
 *                      CONFIDENTIAL INFORMATION
 *                      ------------------------
 *      This Document contains Confidential Information or Trade Secrets,
 *      or both, which are the property of Bacancy System LLP.
 *      This document may not be copied, reproduced, reduced to any
 *      electronic medium or machine readable form or otherwise
 *      duplicated and the information herein may not be used,
 *      disseminated or otherwise disclosed, except with the prior
 *      written consent of Bacancy System LLP.
 *
 */
/*!
 * \file     hardware.c
 */


#include "hardware.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"
#include "BMA400.h"

#include "nrfx_qspi.h"
#include "as7050_agc.h"
#include "common.h"
#include "as7050.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
#define QSPI_PIN_IO1             NRF_GPIO_PIN_MAP(1,8)   
#define QSPI_STD_CMD_WRSR        0x01 /* Write status register */
#define QSPI_STD_CMD_RSTEN       0x66 /* Reset Enable */
#define QSPI_STD_CMD_RST         0x99 /* Reset */
#define QSPI_MANUFACTURE_ID      0x90 /* Manufacture ID Read Address */
#define QSPI_JEDEC_ID            0x9F /* JEDEC ID Read Address */
#define QSPI_ERASE_64KB          0x20
#define QSPI_WRITE_ENABLE        0x06
#define QSPI_FASTREAD_QUADIO     0xEB
#define QSPI_SR_QUAD_ENABLE_BYTE 0x02
#define QSPI_XIP_START_ADDR      0x12000000
#define QSPI_TEST_DATA_SIZE      256
#define QSPI_PACKET_SIZE         88

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
read_flash flashreaddata;

/******************************************************************************
 *                                  LOCALS                                  *
 ******************************************************************************/

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
nrf_qspi_cinstr_conf_t cinstr_cfg = {
  .opcode    = QSPI_STD_CMD_RSTEN,
  .length    = NRF_QSPI_CINSTR_LEN_1B,
  .io2_level = true,
  .io3_level = true,
  .wipwait   = true,
  .wren      = true
  };

static uint32_t volatile m_qspi_base_address = 0;
static uint32_t m_buffer[QSPI_TEST_DATA_SIZE];
static uint32_t m_buffer1[EXPORT_SIZE];
static uint32_t m_buffer2[EXPORT_SIZE];
static uint32_t m_data_buffer[QSPI_TEST_DATA_SIZE/4];
uint32_t ppg_vals[EXPORT_SIZE] = {0};
uint32_t ecg_vals[EXPORT_SIZE] = {0};
int index1 = 0x00000;
int index2 = 0x40000;

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/**
 * @brief: Log Initialization Function
 *
 * @param[in] void
 *
 * @param[out] void
 */
static void log_init(void)
{
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**
 * @brief: I2C Initialization Function
 *
 * @param[in] void
 *
 * @param[out] void
 */
void twi_init(void)
{
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config = {
     .scl                = I2C_SCL_PIN,
     .sda                = I2C_SDA_PIN,
     .frequency          = NRF_TWI_FREQ_400K,
     .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST,
     .clear_bus_init     = false
  };

  err_code = nrf_drv_twi_init(&m_twi,&twi_config,NULL, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief: I2C Uninitialization Function
 *
 * @param[in] void
 *
 * @param[out] void
 */
void twi_deinit(void)
{
  /* Disable I2C Configuration */
  nrf_drv_twi_disable(&m_twi);
  /* De-initialize I2C communication */
  nrf_drv_twi_uninit(&m_twi);
}

/**
 * @brief: I2C Write Function
 *
 * @param[in] unsigned char reg_addr, unsigned char data
 *
 * @param[out] void
 */

void write_reg(unsigned char reg_addr, unsigned char data)
{
  int i;
  ret_code_t err_code;
  uint8_t bytes_to_send[2] = {reg_addr, data};

      err_code = nrf_drv_twi_tx(&m_twi, AS7050_SLAVE_ADD, bytes_to_send, sizeof(bytes_to_send), false);
      if(!err_code)
      {
          //break;
      }
      nrf_delay_ms(1);

  if(err_code)
  {
      printf("Failed to write value 0x%2.2X at Register address 0x%2.2X ! \n", data, reg_addr);
  }
}


/**
 * @brief: I2C read Function
 *
 * @param[in] uint8_t reg_address,uint8_t *data
 *
 * @param[out] bool
 */
bool read_reg_i2c(uint8_t reg_address,uint8_t *data)
{
  int i;
  ret_code_t err_code;
  bool result = false;

  for (i = 0; i < I2C_MAX_RETRIES; i++) 
  {
    err_code = nrf_drv_twi_tx(&m_twi, AS7050_SLAVE_ADD, &reg_address, 1, false);
    if (!err_code) 
    {
      break;
    }
    nrf_delay_ms(1);
  }

  if (!err_code) 
  {
    for (i = 0; i < I2C_MAX_RETRIES; i++) 
    {
      err_code = nrf_drv_twi_rx(&m_twi, AS7050_SLAVE_ADD, data, sizeof(data));
      if (!err_code) 
      {
        result = 0;
        break;
      }
      nrf_delay_ms(1);
    }

    if (err_code) {
      printf("Failed to read value 0x%2.2X at Register address 0x%2.2X !", data, reg_address);
    }
  } else {
    printf("Failed to write Register address 0x%2.2X !", reg_address);
  }
  return result;
}


/**
 * @brief: GPIO Initialization Function
 *
 * @param[in] void
 *
 * @param[out] void
 */
void gpio_init(void)
{
  ret_code_t  err_code;
  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);
  
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_RAW_CONFIG_IN_SENSE_LOTOHI(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;


  nrf_drv_gpiote_in_config_t fall_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  fall_config.pull = NRF_GPIO_PIN_NOPULL;

  nrf_drv_gpiote_in_config_t step_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  step_config.pull = NRF_GPIO_PIN_PULLDOWN;

  nrf_drv_gpiote_in_config_t stat_config = GPIOTE_RAW_CONFIG_IN_SENSE_HITOLO(true);
  stat_config.pull = NRF_GPIO_PIN_PULLUP;

  err_code = nrf_drv_gpiote_in_init(AS7050_INT_PIN, &in_config, (void *)interrupt_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(AS7050_INT_PIN, true);   

  err_code = nrf_drv_gpiote_in_init(ACCEL_INT1, &fall_config,(void *)fall_interrupt_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(ACCEL_INT1, true);  

  err_code = nrf_drv_gpiote_in_init(ACCEL_INT2, &step_config,(void *)step_interrupt_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(ACCEL_INT2, true); 
}

/**
 * @brief: GPIO Uninitialization Function
 *
 * @param[in] void
 *
 * @param[out] void
 */
void gpio_deinit(void)
{
  /* Unintialize GPIO */
  nrf_drv_gpiote_in_event_disable(AS7050_INT_PIN);
  nrf_drv_gpiote_in_event_disable(ACCEL_INT1);
  nrf_drv_gpiote_in_event_disable(ACCEL_INT2);
  nrf_drv_gpiote_uninit();
}

/**
 * @brief: QSPI Communication check function
 *
 * @param[in] void
 *
 * @param[out] void
 */

uint8_t qspi_check(void)
{
  uint32_t err_code;
  flash_power_on();
  qspi_init();
  nrf_delay_ms(100);
  uint8_t m_buffer_tx[QSPI_TEST_DATA_SIZE];
  memset(&m_buffer_tx, 0, sizeof(m_buffer_tx));
  nrf_gpio_pin_clear(NRFX_QSPI_PIN_CSN);
  cinstr_cfg.opcode = QSPI_JEDEC_ID;
  cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_4B;
  err_code = nrfx_qspi_cinstr_xfer(&cinstr_cfg,NULL,m_buffer_tx);
  APP_ERROR_CHECK(err_code);
  nrf_gpio_pin_set(NRFX_QSPI_PIN_CSN);
  flash_power_off();
  if(m_buffer_tx == 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}


/**
 * @brief: QSPI Erase function
 *
 * @param[in] void
 *
 * @param[out] void
 */
void qspi_erase (void)
{
  uint32_t err_code;
  nrf_gpio_pin_clear(NRFX_QSPI_PIN_CSN);

  uint8_t m_buffer_tx[QSPI_TEST_DATA_SIZE];
  m_buffer_tx[0] = 0x02;

  cinstr_cfg.opcode = QSPI_STD_CMD_WRSR;
  cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_4B;
  err_code = nrfx_qspi_cinstr_xfer(&cinstr_cfg, m_buffer_tx, NULL);
  APP_ERROR_CHECK(err_code);
  nrf_gpio_pin_set(NRFX_QSPI_PIN_CSN);


  err_code =nrfx_qspi_erase(QSPI_ERASE_LEN_LEN_All, 0x00);
  APP_ERROR_CHECK(err_code);

  while (nrfx_qspi_mem_busy_check())
  {}
}

/**
 * @brief: QSPI Read function
 *
 * @param[in] uint32_t * m_read_buffer,uint32_t dst_address,uint16_t length
 *
 * @param[out] void
 */

void qspi_read(uint32_t * m_read_buffer,uint32_t dst_address,uint16_t length)
{
  uint32_t err_code;
  uint32_t read_address = m_qspi_base_address + dst_address;

  err_code =nrfx_qspi_read(m_buffer,length, read_address);
  APP_ERROR_CHECK(err_code);
  while (nrfx_qspi_mem_busy_check())
  {}

  memcpy(m_read_buffer,m_buffer,sizeof(m_buffer));
  nrf_delay_ms(10);

  for (int i = 0; i < QSPI_UINT32_T_PAGE_SIZE; i++)
  {
     printf("m_buffer[%d] =  %x\n",i,m_buffer[i]);
     nrf_delay_ms(10);
  }
}

/**
 * @brief: QSPI Write function
 *
 * @param[in] uint32_t * m_write_buffer,uint32_t dst_address,uint16_t length
 *
 * @param[out] void
 */
void qspi_write(uint32_t * m_write_buffer,uint32_t dst_address,uint16_t length)
{
  uint32_t err_code;

  nrf_gpio_pin_clear(NRFX_QSPI_PIN_CSN);

  uint32_t write_address = m_qspi_base_address + dst_address;

  memcpy(m_data_buffer,m_write_buffer,sizeof(m_data_buffer));

  
  err_code = nrfx_qspi_write(m_data_buffer, length,write_address);
  APP_ERROR_CHECK(err_code);
  while (nrfx_qspi_mem_busy_check())
  {}
  
  nrf_gpio_pin_set(NRFX_QSPI_PIN_CSN);
  nrf_delay_ms(50);

  //err_code =nrfx_qspi_read(m_buffer,QSPI_TEST_DATA_SIZE, write_address);
  //APP_ERROR_CHECK(err_code);
  //while (nrfx_qspi_mem_busy_check())
  //{}

  //nrf_delay_ms(10);

  //for (int i = 0; i < QSPI_UINT32_T_PAGE_SIZE; i++)
  //{
  //   printf("m_buffer[%d] =  %x\n",i,m_buffer[i]);
  //   nrf_delay_ms(10);
  //}
}

/**
 * @brief: QSPI Page Write function
 *
 * @param[in] uint32_t* m_write_buffer,uint32_t const dst_address
 *
 * @param[out] void
 */

void qspi_page_write(uint32_t m_write_buffer[256],uint32_t dst_address)
{
  uint32_t err_code;

  nrf_gpio_pin_clear(NRFX_QSPI_PIN_CSN);

  uint32_t write_address = m_qspi_base_address + dst_address;

  memcpy(m_data_buffer,m_write_buffer,sizeof(m_data_buffer));

  
  err_code = nrfx_qspi_write(m_data_buffer, QSPI_TEST_DATA_SIZE,write_address);
  APP_ERROR_CHECK(err_code);
  while (nrfx_qspi_mem_busy_check())
  {}
  
  nrf_gpio_pin_set(NRFX_QSPI_PIN_CSN);
  nrf_delay_ms(50);

  //err_code =nrfx_qspi_read(m_buffer,QSPI_TEST_DATA_SIZE, write_address);
  //APP_ERROR_CHECK(err_code);
  //while (nrfx_qspi_mem_busy_check())
  //{}

  //nrf_delay_ms(10);

  //for (int i = 0; i < QSPI_UINT32_T_PAGE_SIZE; i++)
  //{
  //   printf("m_buffer[%d] =  %x\n",i,m_buffer[i]);
  //   nrf_delay_ms(10);
  //}
}


/**
 * @brief: QSPI Page Read function
 *
 * @param[in] uint32_t const dst_address
 *
 * @param[out] void
 */
void qspi_page_read(uint32_t const dst_address,uint8_t data_flag)
{
  uint32_t err_code;
  uint32_t read_address = m_qspi_base_address + dst_address;
  uint32_t num_raw_vals = EXPORT_SIZE;

  err_code = nrfx_qspi_read(m_buffer1, QSPI_PACKET_SIZE, index1);
  APP_ERROR_CHECK(err_code);
  while (nrfx_qspi_mem_busy_check()) 
  {
  }
  memcpy(ecg_vals, m_buffer1, QSPI_PACKET_SIZE);
  index1 += QSPI_PACKET_SIZE;

  nrf_delay_ms(10);

  err_code = nrfx_qspi_read(m_buffer2, QSPI_PACKET_SIZE, index2);
  APP_ERROR_CHECK(err_code);
  while (nrfx_qspi_mem_busy_check()) 
  {
  }
  memcpy(ppg_vals, m_buffer2, QSPI_PACKET_SIZE);
  index2 += QSPI_PACKET_SIZE;

  nrf_delay_ms(10);
}
/**
 * @brief: QSPI Initialization function
 *
 * @param[in] void
 *
 * @param[out] void
 */
void qspi_init(void)
{
  uint32_t err_code = NRF_SUCCESS;

  /* Set QSPI peripheral with default configuration. */
  nrfx_qspi_config_t config = NRFX_QSPI_DEFAULT_CONFIG(NRFX_QSPI_PIN_SCK, NRFX_QSPI_PIN_CSN, NRFX_QSPI_PIN_IO0,         \
                               NRFX_QSPI_PIN_IO1, NRFX_QSPI_PIN_IO2, NRFX_QSPI_PIN_IO3);

  /* Try to initialize QSPI peripheral in blocking mode. */
  err_code = nrfx_qspi_init(&config, NULL, NULL);
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief: QSPI Uninitialization function
 *
 * @param[in] void
 *
 * @param[out] void
 */
void qspi_deinit(void)
{
  nrfx_qspi_uninit();
  nrf_gpio_cfg_output(CS_FLASH);
  nrf_gpio_pin_set(CS_FLASH);
  nrf_gpio_cfg_output(FLASH_EN);
  nrf_gpio_pin_clear(FLASH_EN);
}

/**
 * @brief: QSPI Command Write function
 *
 * @param[in] uint8_t Data
 *
 * @param[out] uint8_t
 */
uint8_t W25qxx_Spi(uint8_t Data)
{
  uint8_t ret;
  uint32_t err_code;
  cinstr_cfg.opcode = Data;
  err_code = nrfx_qspi_cinstr_xfer(&cinstr_cfg, &Data, &ret);
  APP_ERROR_CHECK(err_code);
  return ret;
}

/**
 * @brief: QSPI Status Register Read function
 *
 * @param[in]uint8_t SelectStatusRegister_1_2_3
 *
 * @param[out] uint8_t
 */
uint8_t W25qxx_ReadStatusRegister(uint8_t SelectStatusRegister_1_2_3)
{
  uint8_t status = 0;
  uint8_t ret = 0;
  nrf_gpio_pin_clear(NRFX_QSPI_PIN_CSN);
  if (SelectStatusRegister_1_2_3 == 1)
  {
      ret = W25qxx_Spi(0x05);
      printf("\nStatusRegister1 = %x\n",ret);
  }
  else if (SelectStatusRegister_1_2_3 == 2)
  {
      ret = W25qxx_Spi(0x35);
      printf("\nStatusRegister2 = %x\n",ret);
  }
  else
  {
      ret = W25qxx_Spi(0x15);
      printf("\nStatusRegister3 = %x\n",ret);
  }
  nrf_gpio_pin_set(NRFX_QSPI_PIN_CSN);
  return status;
}

/**
 * @brief: Function to turn on accelerometer
 *
 * @param[in] void
 *
 * @param[out] void
 */
void accel_power_on(void)
{
  nrf_gpio_cfg_output(ACCEL_EN);

  //nrf_gpio_cfg(
  //  ACCEL_EN,
  //  NRF_GPIO_PIN_DIR_OUTPUT,
  //  NRF_GPIO_PIN_INPUT_DISCONNECT,
  //  NRF_GPIO_PIN_NOPULL,
  //  NRF_GPIO_PIN_D0S1,
  //  NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_set(ACCEL_EN);
    nrf_delay_ms(100);
}

/**
 * @brief: Function to turn off accelerometer
 *
 * @param[in] void
 *
 * @param[out] void
 */
void accel_power_off(void)
{
  nrf_gpio_cfg_output(ACCEL_EN);
  
  //nrf_gpio_cfg(
  //  ACCEL_EN,
  //  NRF_GPIO_PIN_DIR_OUTPUT,
  //  NRF_GPIO_PIN_INPUT_DISCONNECT,
  //  NRF_GPIO_PIN_NOPULL,
  //  NRF_GPIO_PIN_D0S1,
  //  NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_clear(ACCEL_EN);
  nrf_delay_ms(100);
}

/**
 * @brief: Function to turn off flash
 *
 * @param[in] void
 *
 * @param[out] void
 */
void flash_power_off(void)
{
  nrf_gpio_cfg_output(FLASH_EN);
  nrf_gpio_pin_clear(FLASH_EN);
  nrf_gpio_cfg_output(CS_FLASH);
  nrf_gpio_pin_set(CS_FLASH);
  nrf_delay_ms(500);
}

/**
 * @brief: Function to turn on flash
 *
 * @param[in] void
 *
 * @param[out] void
 */
void flash_power_on(void)
{
  nrf_gpio_cfg(FLASH_EN,GPIO_PIN_CNF_DIR_Output,GPIO_PIN_CNF_INPUT_Connect,GPIO_PIN_CNF_PULL_Disabled,GPIO_PIN_CNF_DRIVE_H0H1,GPIO_PIN_CNF_SENSE_Disabled);
  nrf_gpio_pin_set(FLASH_EN);
  nrf_delay_ms(500);
}


/**
 * @brief: Function to turn on temp
 *
 * @param[in] void
 *
 * @param[out] void
 */
void temp_power_on(void)
{
  nrf_gpio_cfg_output(TEMP_EN);
  nrf_gpio_pin_set(TEMP_EN);
  nrf_delay_ms(50);
}

/**
 * @brief: Function to turn off temp
 *
 * @param[in] void
 *
 * @param[out] void
 */
void temp_power_off(void)
{
  nrf_gpio_cfg_output(TEMP_EN);
  nrf_gpio_pin_clear(TEMP_EN);
}

/**
 * @brief: Function to turn off AS7050
 *
 * @param[in] void
 *
 * @param[out] void
 */
void as7050_power_off(void)
{
  nrf_gpio_cfg_output(AS7050_ENABLE_PIN);
  nrf_gpio_pin_clear(AS7050_ENABLE_PIN);
  as7050_power = 0;
  nrf_delay_ms(50);
}

/**
 * @brief: Function to turn on AS7050
 *
 * @param[in] void
 *
 * @param[out] void
 */
void as7050_power_on(void)
{
  nrf_gpio_cfg_output(AS7050_ENABLE_PIN);
  nrf_gpio_pin_set(AS7050_ENABLE_PIN);
  as7050_power = 1;
  nrf_delay_ms(50);
}

/**
 * @brief: Function to enable battery monitoring
 *
 * @param[in] void
 *
 * @param[out] void
 */
void battery_mon_en(void)
{
  nrf_gpio_cfg_output(BAT_MON_ENABLE);
  nrf_gpio_pin_set(BAT_MON_ENABLE);
  nrf_delay_ms(50);
}

/**
 * @brief: Function to disable battery monitoring
 *
 * @param[in] void
 *
 * @param[out] void
 */
void battery_mon_disable(void)
{
  nrf_gpio_cfg_output(BAT_MON_ENABLE);
  nrf_gpio_pin_clear(BAT_MON_ENABLE);
}


void configure_flash_settings(void)
{
  uint32_t err_code;
  uint8_t m_read_buffer[10]={0};


  nrf_gpio_pin_clear(NRFX_QSPI_PIN_CSN);

  m_read_buffer[0] = 0x42;

  cinstr_cfg.opcode = QSPI_STD_CMD_WRSR;
  cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_2B;
  err_code = nrfx_qspi_cinstr_xfer(&cinstr_cfg, m_read_buffer, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_gpio_pin_set(NRFX_QSPI_PIN_CSN); 

  W25qxx_ReadStatusRegister(1);
}

/**
 * @brief: Hardware Peripherals Initialization function
 *
 * @param[in] void
 *
 * @param[out] void
 */

void hardware_init(void)
{
  log_init();
  gpio_init();
  twi_init();
}