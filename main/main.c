#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_system.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <driver/i2c.h>
#include <sdrone_motors_task.h>

static sdrone_motors_state_t motors_state;
static sdrone_motors_state_handle_t sdrone_motors_state_handle = &motors_state;
static TaskHandle_t motors_task_handle;

static esp_err_t my_i2c_init(int i2c_master_port) {
	i2c_config_t conf = {
	    .mode = I2C_MODE_MASTER,
	    .sda_io_num = GPIO_NUM_4,         // select GPIO specific to your project
	    .sda_pullup_en = GPIO_PULLUP_ENABLE,
	    .scl_io_num = GPIO_NUM_0,         // select GPIO specific to your project
	    .scl_pullup_en = GPIO_PULLUP_ENABLE,
	    .master.clk_speed = 400000,  // select frequency specific to your project
	    // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
	};
	ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, 0,0,0));
	return ESP_OK;
}

esp_err_t my_nvs_init(){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
	return err;
}

void app_main(void)
{

	/*
	 * TODO: define component for hw initialization
	 */
	ESP_ERROR_CHECK(my_nvs_init());
	ESP_ERROR_CHECK(my_i2c_init(1));

    TaskHandle_t tsk = xTaskGetCurrentTaskHandle();
    sdrone_motors_state_handle->controller_task_handle = tsk;
    xTaskCreate(sdrone_motors_task, "my_motors_test_task", 4096, sdrone_motors_state_handle, 5, &motors_task_handle);
    float thrust = 0.0f;
    float increment = 0.4f;
    uint16_t counter = 0;
    while(true) {
    	counter++;
    	counter %= 600;
    	if(counter == 0) {
        	if(thrust >= 4.4f) {
        		increment = -0.4f;
        	} else if(thrust < 0.4f) {
        		increment = 0.4f;
        	}
        	thrust += increment;
    	}
    	sdrone_motors_state_handle->input.isCommand = 0;
    	sdrone_motors_state_handle->input.data.thrust = thrust;
    	sdrone_motors_state_handle->input.data.tx_rx_flag = MOTORS_TXRX_TRANSMITTED;
    	vTaskDelay(pdMS_TO_TICKS(1));
    	xTaskNotify(motors_task_handle,pdPASS,eSetValueWithOverwrite);
    }

}

