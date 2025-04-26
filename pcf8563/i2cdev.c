#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"

#include "esp_log.h"

#include "i2cdev.h"

#define TAG "I2CDEV"

esp_err_t i2c_dev_init(i2c_dev_t *dev)
{
	i2c_master_bus_config_t i2c_mst_config = {
		.i2c_port = dev->port,
		.sda_io_num = dev->sda_io_num,
		.scl_io_num = dev->scl_io_num,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.flags.enable_internal_pullup = false,
		.glitch_ignore_cnt = 7
	};

	i2c_master_bus_handle_t bus_handle;

	esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);

	if (ret != ESP_OK) 
	{
		ESP_LOGE(TAG, "Failed to initialize I2C master bus: %d", ret);
	}

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = dev->addr,
		.scl_speed_hz = dev->clk_speed ? dev->clk_speed : I2C_FREQ_HZ,
	};

	ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->device_handle);

	if (ret != ESP_OK) 
	{
		ESP_LOGE(TAG, "Could not add device to I2C bus: %d", ret);
		return ret;
	}

	return ret;
}

esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
	if (!dev || !in_data || !in_size) return ESP_ERR_INVALID_ARG;
	
	esp_err_t ret;

	if (out_data && out_size) {
		// Write then read operation
		ret = i2c_master_transmit_receive(
			dev->device_handle,
			(const uint8_t *)out_data, out_size, 
			(uint8_t *)in_data, in_size, 
			I2CDEV_TIMEOUT);
	} else {
		// Read-only operation
		ret = i2c_master_receive(dev->device_handle, (uint8_t *)in_data, in_size, I2CDEV_TIMEOUT);
	}

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d", dev->addr, dev->port, ret);
	}

	return ret;
}

esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
	if (!dev || !out_data || !out_size) return ESP_ERR_INVALID_ARG;

	// Prepare combined buffer for writing
	size_t total_size = out_reg_size + out_size;
	uint8_t *write_buffer = malloc(total_size);
	
	if (write_buffer == NULL) {
		ESP_LOGE(TAG, "Failed to allocate memory for write buffer");
		return ESP_ERR_NO_MEM;
	}
	
	// Copy register and data to combined buffer
	if (out_reg && out_reg_size) {
		memcpy(write_buffer, out_reg, out_reg_size);
	}
	memcpy(write_buffer + out_reg_size, out_data, out_size);
	
	// Perform write operation
	esp_err_t ret = i2c_master_transmit(dev->device_handle, write_buffer, total_size, I2CDEV_TIMEOUT);
	
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Could not write to device [0x%02x at %d]: %d", dev->addr, dev->port, ret);
	}

	free(write_buffer);
	return ret;
}

esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *in_data, size_t in_size)
{
	return i2c_dev_read(dev, &reg, 1, in_data, in_size);
}

esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *out_data, size_t out_size)
{
	return i2c_dev_write(dev, &reg, 1, out_data, out_size);
}