/* I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "ads1115.h"

static const char *TAG = "main";

/**
 * TEST CODE BRIEF
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO0 is assigned as the data signal of i2c master port
 *    GPIO2 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of sensor with GPIO0/GPIO2
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the data, if connected.
 */

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

/**
 * @brief Function to initialize ESP in I2C mode and write the bits in the Configuration register on the ADS1115
 *
 * @param i2c_num
 * @return esp_err_t
 */
static esp_err_t i2c_master_ads1115_init(i2c_port_t i2c_num)
{
    vTaskDelay(200 / portTICK_RATE_MS);
    i2c_master_init();
    ESP_LOGI(TAG, "Starting ESP Master Initialization for ADS1115...\n");
	
	ADS1115_CONFIG_FIELDS config_fields;
	/* No effect */
    config_fields.OS = 0x00;   
	/* AINp = A0 & AINn = GND */
    config_fields.MUX = 0x04;       
	/* Gain - 4.096 V */
    config_fields.PGA = 0x01;    
	/* Continuous-conversion mode */
    config_fields.MODE = 0x00;      
	/* 128 SPS */
    config_fields.DR = 0x04;        
	/* Traditional comparator */
    config_fields.COMP_MODE = 0x00; 
	/* Active low */
    config_fields.COMP_POL = 0x00; 
	/* Nonlatching comparator */
    config_fields.COMP_LAT = 0x00;
	/* Assert after 4 cconversions */
    config_fields.COMP_QUE = 0x02;
	
	get_16bit_config(&config_fields);

    ESP_LOGI(TAG, "Config Field: %d\n", (int)config_fields.configuration);

    /* WRITING TO CONFIG REGISTER */
    ESP_ERROR_CHECK(ads1115_write_data(i2c_num, ADS1115_CONFIG_REG, config_fields.configuration));
    return ESP_OK;
}

/**
 * @brief Task Funnction for xTaskCreate FreeRTOS function
 *
 * Initialize the ESP in I2C mode and starts process to continously read the ADS1115
 * @param config_params ->  NULL
 */
static void adc_task(void *config_params)
{
    uint16_t sensor_data;
    esp_err_t ret;

    ret = i2c_master_ads1115_init(I2C_MASTER_NUM);
    if (ret == ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Successfully Initialized ADS1115 and I2C Protocol.\n");
    }
    else
    {
        i2c_driver_delete(I2C_MASTER_NUM);
        ESP_LOGI(TAG, "Error: %s", (char *)ret);
    }
    while (1)
    {
        ret = (ads1115_read_data(I2C_MASTER_NUM, ADS1115_CONV_REG, &sensor_data));
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Success!\n");
            ESP_LOGI(TAG, "Value: %d\n", (int)sensor_data);
            vTaskDelay(3000 / portTICK_PERIOD_MS);
        }
        else
        {
            ESP_LOGI(TAG, "Could not read the ADS1115\nError: %s", (char *)ret);
            vTaskDelete(NULL);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    xTaskCreate(adc_task, "adc_task", 2048, NULL, 5, NULL);
}

static void get_16bit_config(ADS1115_CONFIG_FIELDS *config)
{
    uint16_t data;

    data = (config->OS << 3) | config->MUX;
    data = (data << 3) | config->PGA;
    data = (data << 1) | config->MODE;
    data = (data << 3) | config->DR;
    data = (data << 1) | config->COMP_MODE;
    data = (data << 1) | config->COMP_POL;
    data = (data << 1) | config->COMP_LAT;
    data = (data << 2) | config->COMP_QUE;
    config->configuration = data;
}

static esp_err_t ads1115_write_bits(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, uint16_t data_len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                         
    i2c_master_write_byte(cmd, (ADS1115_ADDR_GND << 1) | WRITE_BIT, ACK_CHECK_EN); 
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);                            
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);                           
    i2c_master_stop(cmd);                                                          
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t ads1115_write_data(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t data)
{
    esp_err_t ret;
    uint8_t write_buff[2];

    write_buff[0] = (data >> 8) & 0xFF;
    write_buff[1] = (data >> 0) & 0xFF;

    ret = ads1115_write_bits(i2c_num, reg_addr, write_buff, 2);

    return ret;
}

static esp_err_t ads1115_read_bits(i2c_port_t i2c_num, uint8_t reg_addr, uint8_t *data, uint16_t data_len)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR_GND << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "Could not read bits from ADS1115!");
        return ret;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR_GND << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    for (int i = 0; i < data_len; i++)
    {
        ESP_LOGI(TAG, "Byte %d: %d", i, data[i]);
    }
    return ret;
}

static esp_err_t ads1115_read_data(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t *data)
{
    esp_err_t ret;
    uint16_t sensor_data = 0;
    uint8_t read_buff[2];

    ret = ads1115_read_bits(i2c_num, reg_addr, read_buff, 2);
    sensor_data = (read_buff[0] << 8) | read_buff[1];
    *data = sensor_data;
    return ret;
}
