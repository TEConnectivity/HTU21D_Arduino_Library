#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Enum
enum htu21_status {
  htu21_status_ok,
  htu21_status_i2c_transfer_error,
  htu21_status_no_i2c_acknowledge,
  htu21_status_crc_error
};
enum htu21_i2c_master_mode { htu21_i2c_no_hold, htu21_i2c_hold };
enum htu21_status_code {
  htu21_STATUS_OK = 0,
  htu21_STATUS_ERR_OVERFLOW = 1,
  htu21_STATUS_ERR_TIMEOUT = 4
};
enum htu21_heater_status { htu21_heater_off, htu21_heater_on };
enum htu21_battery_status { htu21_battery_ok, htu21_battery_low };
enum htu21_resolution {
  htu21_resolution_t_14b_rh_12b = 0,
  htu21_resolution_t_12b_rh_8b,
  htu21_resolution_t_13b_rh_10b,
  htu21_resolution_t_11b_rh_11b
};

// Functions
class htu21d {

public:
  htu21d();

  /**
   * \brief Perform initial configuration. Has to be called once.
   */
  void begin(void);

  /**
   * \brief Check whether HTU21 device is connected
   *
   * \return bool : status of HTU21
   *       - true : Device is present
   *       - false : Device is not acknowledging I2C address
   */
  boolean is_connected(void);

  /**
   * \brief Get heater status
   *
   * \param[in] htu21_heater_status* : Return heater status (above or below
   *2.5V)
   *	                    - htu21_heater_off,
   *                      - htu21_heater_on
   *
   * \return htu21_status : status of HTU21
   *       - htu21_status_ok : I2C transfer completed successfully
   *       - htu21_status_i2c_transfer_error : Problem with i2c transfer
   *       - htu21_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum htu21_status get_heater_status(enum htu21_heater_status *heater);

  /**
   * \brief Enable heater
   *
   * \return htu21_status : status of HTU21
   *       - htu21_status_ok : I2C transfer completed successfully
   *       - htu21_status_i2c_transfer_error : Problem with i2c transfer
   *       - htu21_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum htu21_status enable_heater(void);

  /**
   * \brief Disable heater
   *
   * \return htu21_status : status of HTU21
   *       - htu21_status_ok : I2C transfer completed successfully
   *       - htu21_status_i2c_transfer_error : Problem with i2c transfer
   *       - htu21_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum htu21_status disable_heater(void);

  /**
   * \brief Provide battery status
   *
   * \param[out] htu21_battery_status* : Battery status
   *                      - htu21_battery_ok,
   *                      - htu21_battery_low
   *
   * \return status of HTU21
   *       - htu21_status_ok : I2C transfer completed successfully
   *       - htu21_status_i2c_transfer_error : Problem with i2c transfer
   *       - htu21_status_no_i2c_acknowledge : I2C did not acknowledge
   */
  enum htu21_status get_battery_status(enum htu21_battery_status *);

  /**
   * \brief Reads the htu21 serial number.
   *
   * \param[out] uint64_t* : Serial number
   *
   * \return htu21_status : status of HTU21
   *       - htu21_status_ok : I2C transfer completed successfully
   *       - htu21_status_i2c_transfer_error : Problem with i2c transfer
   *       - htu21_status_no_i2c_acknowledge : I2C did not acknowledge
   *       - htu21_status_crc_error : CRC check error
   */
  enum htu21_status reset(void);

  /**
   * \brief Reads the relative humidity and temperature value.
   *
   * \param[out] float* : Celsius Degree temperature value
   * \param[out] float* : %RH Relative Humidity value
   *
   * \return htu21_status : status of HTU21
   *       - htu21_status_ok : I2C transfer completed successfully
   *       - htu21_status_i2c_transfer_error : Problem with i2c transfer
   *       - htu21_status_no_i2c_acknowledge : I2C did not acknowledge
   *       - htu21_status_crc_error : CRC check error
   */
  enum htu21_status read_temperature_and_relative_humidity(float *temperature,
                                                           float *humidity);

  /**
   * \brief Set temperature and humidity ADC resolution.
   *
   * \param[in] htu21_resolution : Resolution requested
   *
   * \return htu21_status : status of HTU21
   *       - htu21_status_ok : I2C transfer completed successfully
   *       - htu21_status_i2c_transfer_error : Problem with i2c transfer
   *       - htu21_status_no_i2c_acknowledge : I2C did not acknowledge
   *       - htu21_status_crc_error : CRC check error
   */
  enum htu21_status set_resolution(enum htu21_resolution res);

  /**
   * \brief Returns result of compensated humidity
   *
   * \param[in] float - Actual temperature measured (degC)
   * \param[in] float - Actual relative humidity measured (%RH)
   *
   * \return float - Compensated humidity (%RH).
   */
  float compute_compensated_humidity(float temperature,
                                     float relative_humidity);

  /**
   * \brief Returns the computed dew point
   *
   * \param[in] float - Actual temperature measured (degC)
   * \param[in] float - Actual relative humidity measured (%RH)
   *
   * \return float - Dew point temperature (DegC).
   */
  float compute_dew_point(float temperature, float relative_humidity);

  /**
   * \brief Reads the htu21 serial number.
   *
   * \param[out] uint64_t* : Serial number
   *
   * \return htu21_status : status of HTU21
   *       - htu21_status_ok : I2C transfer completed successfully
   *       - htu21_status_i2c_transfer_error : Problem with i2c transfer
   *       - htu21_status_no_i2c_acknowledge : I2C did not acknowledge
   *       - htu21_status_crc_error : CRC check error
   */
  enum htu21_status read_serial_number(uint64_t *serial_number);

  /**
   * \brief Set I2C master mode.
   *        This determines whether the program will hold while ADC is accessed
   * or will wait some time
   *
   * \param[in] htu21_i2c_master_mode : I2C mode
   *
   */
  void set_i2c_master_mode(enum htu21_i2c_master_mode mode);

private:
  enum htu21_status read_user_register(uint8_t *value);
  enum htu21_status write_user_register(uint8_t value);
  enum htu21_status write_command(uint8_t cmd);
  enum htu21_status temperature_conversion_and_read_adc(uint16_t *adc);
  enum htu21_status humidity_conversion_and_read_adc(uint16_t *adc);
  enum htu21_status crc_check(uint16_t value, uint8_t crc);
};
