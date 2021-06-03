/*
 * sdrone_motors_task.c
 *
 *  Created on: 19 apr 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <motors.h>
#include <sdrone_motors_task.h>
#include <driver/i2c.h>
#include <ina3221.h>


void sdrone_motors_controller_init(
		sdrone_motors_state_handle_t sdrone_motors_state_handle) {
	printf("sdrone_motors_controller_init init initial state and motors\n");
	memset(sdrone_motors_state_handle, 0, sizeof(*sdrone_motors_state_handle));
	ESP_ERROR_CHECK(motors_init(&(sdrone_motors_state_handle->motors)));

	sdrone_motors_state_handle->motors_task_handle =
			xTaskGetCurrentTaskHandle();

	// voltage & current sensor (3 channels);
	printf("sdrone_motors_controller_init init ina3221 sensor\n");
	static ina3221_t ina3221;
	ina3221_handle_t ina3221_handle = &ina3221;
	ina3221_handle->i2c_master_port = MOTORS_INA3221_I2C_PORT;
	ESP_ERROR_CHECK(ina3221_init(ina3221_handle));
	ESP_ERROR_CHECK(ina3221_test_connection(ina3221_handle));
	sdrone_motors_state_handle->ina3221_handle = ina3221_handle;
	printf("sdrone_motors_controller_init initial state and motors initialized\n");
}

esp_err_t sdrone_motors_two_horizontal_axis_control(
		sdrone_motors_state_handle_t sdrone_motors_state_handle) {
	return ESP_OK;
}
esp_err_t sdrone_motors_horizontal_hexacopter_control(
		sdrone_motors_state_handle_t sdrone_motors_state_handle) {
	return ESP_OK;
}
void sdrone_motors_controller_cycle(
		sdrone_motors_state_handle_t sdrone_motors_state_handle) {
	motors_handle_t motors_handle = &(sdrone_motors_state_handle->motors);

	printf("sdrone_motors_task_init::Arm motors\n");
	ESP_ERROR_CHECK(motors_arm(motors_handle));
	vTaskDelay(pdMS_TO_TICKS(10));

	while (true) {
		// Take Notify at end of duty_cycle
		if (ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(10))
				== pdPASS) {
			if (sdrone_motors_state_handle->input.isCommand) {
				switch (sdrone_motors_state_handle->input.command.type) {
				case MOTORS_ARM: {
					ESP_ERROR_CHECK(motors_arm(motors_handle));
					break;
				}
				case MOTORS_DISARM: {
					ESP_ERROR_CHECK(motors_disarm(motors_handle));
					break;
				}
				case MOTORS_SWITCHOFF: {
					ESP_ERROR_CHECK(motors_switchoff(motors_handle));
					break;
				}
				case MOTORS_SWITCHON: {
					ESP_ERROR_CHECK(motors_switchon(motors_handle));
					break;
				}
				default: {
					printf(
							"ERROR: sdrone_motors_controller_cycle::command %d unknown for motors",
							sdrone_motors_state_handle->input.command.type);
				}
				}
			} else if (sdrone_motors_state_handle->input.data.tx_rx_flag
					== MOTORS_TXRX_TRANSMITTED) {
				/*
				 * load input data to local var
				 *
				 * TODO: REMOVE and IMPLEMENTS this function. This is for my tests.
				 */
				float nd = 0;
				float duty_prev = motors_handle->motor[1].duty_cycle;
				ESP_ERROR_CHECK(motors_newton_to_duty(sdrone_motors_state_handle->input.data.thrust, &motors_handle->motor[1].duty_cycle));
				ESP_ERROR_CHECK(motors_duty_to_newton(motors_handle->motor[1].duty_cycle, &nd));
				ESP_ERROR_CHECK(ina3321_load_data(sdrone_motors_state_handle->ina3221_handle));
				motors_handle->motor[0].duty_cycle = motors_handle->motor[1].duty_cycle;
				motors_handle->motor[2].duty_cycle = motors_handle->motor[1].duty_cycle;
				motors_handle->motor[3].duty_cycle = motors_handle->motor[1].duty_cycle;
				motors_handle->motor[4].duty_cycle = motors_handle->motor[1].duty_cycle;
				motors_handle->motor[5].duty_cycle = motors_handle->motor[1].duty_cycle;

				printf("%5.5f, %d, %d, %d, %d, %d, %d;\n", duty_prev,
						sdrone_motors_state_handle->ina3221_handle->raw_data.channel_data[0].bus_voltage,
						sdrone_motors_state_handle->ina3221_handle->raw_data.channel_data[0].shunt_voltage,
						sdrone_motors_state_handle->ina3221_handle->raw_data.channel_data[1].bus_voltage,
						sdrone_motors_state_handle->ina3221_handle->raw_data.channel_data[1].shunt_voltage,
						sdrone_motors_state_handle->ina3221_handle->raw_data.channel_data[2].bus_voltage,
						sdrone_motors_state_handle->ina3221_handle->raw_data.channel_data[2].shunt_voltage
					  );

				ESP_ERROR_CHECK(motors_update(motors_handle));
				sdrone_motors_state_handle->input.data.tx_rx_flag =
						MOTORS_TXRX_RECEIVED;
			}
			/*
			 * calc Y(i) for each motor
			 * calc duty_cycle for each motor
			 * not interrompible
			 *   update motors
			 */

#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
	      ESP_ERROR_CHECK(sdrone_motors_horizontal_hexacopter_control(sdrone_motors_state_handle));
#else
#ifdef MOTORS_FRAME_TWO_HORIZONTAL_AXIS
			ESP_ERROR_CHECK(
					sdrone_motors_two_horizontal_axis_control(
							sdrone_motors_state_handle));
#endif
#endif
		}
	}
}

void sdrone_motors_task(void *arg) {
	sdrone_motors_state_handle_t sdrone_motors_state_handle =
			(sdrone_motors_state_handle_t) arg;
	sdrone_motors_controller_init(sdrone_motors_state_handle);
	sdrone_motors_controller_cycle(sdrone_motors_state_handle);

}
