#ifndef MAIN_I2CDEV_H_
#define MAIN_I2CDEV_H_

#include "driver/i2c_master.h"

#define I2C_FREQ_HZ 100000
#define I2CDEV_TIMEOUT 1000

typedef struct {
	i2c_port_t port;	// I2C port number
	uint8_t addr;		// I2C address
	gpio_num_t sda_io_num;	// GPIO number for I2C sda signal
	gpio_num_t scl_io_num;	// GPIO number for I2C scl signal
	uint32_t clk_speed;		// I2C clock frequency for master mode
	i2c_master_dev_handle_t device_handle;	// I2C device handle
} i2c_dev_t;

esp_err_t i2c_dev_init(i2c_dev_t *dev);
esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size);
esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size);
esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg, void *in_data, size_t in_size);
esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg, const void *out_data, size_t out_size);
#endif /* MAIN_I2CDEV_H_ */
