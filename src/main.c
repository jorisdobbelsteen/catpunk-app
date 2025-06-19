#include <stdio.h>

#include <zephyr/kernel.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <app_version.h>

#define SLEEP_TIME_MS 50

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *const accel = DEVICE_DT_GET(DT_ALIAS(accel0));
static const struct device *const gyro = DEVICE_DT_GET(DT_ALIAS(gyro0));
static const struct device *const magn = DEVICE_DT_GET(DT_ALIAS(magn0));

K_SEM_DEFINE(accel_sem, 0, 1); /* starts off "not available" */
//K_SEM_DEFINE(gyro_sem, 0, 1); /* starts off "not available" */
K_SEM_DEFINE(magn_sem, 0, 1); /* starts off "not available" */

static void trigger_handler(const struct device *dev, const struct sensor_trigger *trigger)
{
  ARG_UNUSED(trigger);

  /* Always fetch the sample to clear the data ready interrupt in the
   * sensor.
   */
  if (sensor_sample_fetch(dev)) {
    printf("sensor_sample_fetch failed\n");
    return;
  }

  if (dev == accel) {
    k_sem_give(&accel_sem);
//  } else if (dev == gryo) {
//    k_sem_give(&gyro_sem);
  } else if (dev == magn) {
    k_sem_give(&magn_sem);
  }
}

int main(void)
{
  int ret;

	printk("Zephyr Example Application %s\n", APP_VERSION_STRING);

  bool led_state = true;

  // LED Initialization

  if (!gpio_is_ready_dt(&led)) {
    printf("GPIO %s is not ready\n", led.port->name);
    return 0;
  }

  if(gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0) {
    printf("GPIO %s cannot be configured\n", led.port->name);
    return 0;
  }

  // Accelerometer initialization

  struct sensor_trigger accel_trig = {
    .type = SENSOR_TRIG_DATA_READY,
    .chan = SENSOR_CHAN_ACCEL_XYZ,
  };

  if (!device_is_ready(accel)) {
    printf("Device %s is not ready\n", accel->name);
    return 0;
  }

  if (sensor_trigger_set(accel, &accel_trig, trigger_handler)) {
    printf("Could not set trigger\n");
    return 0;
  }

  // Magnetometer initialization

  struct sensor_trigger magn_trig = {
          .type = SENSOR_TRIG_DATA_READY,
          .chan = SENSOR_CHAN_MAGN_XYZ,
  };

  if (!device_is_ready(magn)) {
    printf("Device %s is not ready\n", magn->name);
    return 0;
  }

  if (sensor_trigger_set(magn, &magn_trig, trigger_handler)) {
    printf("Could not set trigger\n");
    return 0;
  }

  // Magnetometer initialization

  // struct sensor_trigger gyro_trig = {
  //   .type = SENSOR_TRIG_DATA_READY,
  //   .chan = SENSOR_CHAN_GYRO_XYZ,
  // };

  if (!device_is_ready(gyro)) {
    printf("Device %s is not ready\n", magn->name);
    return 0;
  }

  struct sensor_value gyro_odr = { 0 };
  sensor_value_from_double(&gyro_odr, 100.0);
  if (!sensor_attr_set(gyro, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &gyro_odr)) {
    printf("Could not configure gyro\n");
  }

  // if (sensor_trigger_set(gyro, &gyro_trig, trigger_handler)) {
  //   printf("Could not set trigger\n");
  //   return 0;
  // }

  // LED toggle & processing

  struct sensor_value data[3];

  while (1) {
    if (gpio_pin_toggle_dt(&led) < 0) {
      printf("GPIO %s cannot be toggled\n", led.port->name);
      return 0;
    }

    led_state = !led_state;
    //printf("LED state: %s\n", led_state ? "ON" : "OFF");

    if (k_sem_take(&accel_sem, K_NO_WAIT)) {
      sensor_channel_get(accel, SENSOR_CHAN_ACCEL_XYZ, data);

      /* Print accel x,y,z data */
      printf("accel [m/s^2]: (%12.6f, %12.6f, %12.6f)\n",
             sensor_value_to_double(&data[0]), sensor_value_to_double(&data[1]),
             sensor_value_to_double(&data[2]));

      sensor_sample_fetch(gyro);
      if (sensor_channel_get(gyro, SENSOR_CHAN_GYRO_XYZ, data) == 0)
      {
        /* Print gyro x,y,z data, but only if available */
        printf("gyro [rad/s]:  (%12.6f, %12.6f, %12.6f)\n",
               sensor_value_to_double(&data[0]), sensor_value_to_double(&data[1]),
               sensor_value_to_double(&data[2]));
      }
    }

    if (k_sem_take(&magn_sem, K_NO_WAIT)) {
      sensor_channel_get(magn, SENSOR_CHAN_MAGN_XYZ, data);

      /* Print magnetometer x,y,z data */
      printf("magn  [G]:     (%12.6f, %12.6f, %12.6f)\n",
             sensor_value_to_double(&data[0]), sensor_value_to_double(&data[1]),
             sensor_value_to_double(&data[2]));
    }

    k_msleep(SLEEP_TIME_MS);
  }
  return 0;
}
