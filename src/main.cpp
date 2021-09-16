/* 
 *  Basic blink brogram that runs on the ESP32's 3rd processor!
 *  By: Ray Richter
 * 
 *  This program sets up then runs the ESP32's Ultra Low-power
 *  Processor (ULP). THe ULP is programmed in assembly as it only
 *  has 4 registers (and a weird 5th one not used here).
 */

#include <Arduino.h>
#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <soc/rtc.h>

#define GPIO_RTC_OFFSET 10 // RTC value for GPIO 4 - The RTC Pin number for GPIO4


/**
 * @brief 
 *    Initialization of the ULP
 * 
 * @details 
 *    This function contains the ULP assembly code written using macros.
 *    It also puts that assembly code into the 8KB block of memory that
 *    the ULP uses.
 * 
 */
void startULP() {
  // Set GPIO 4 as an output
  rtc_gpio_init(GPIO_NUM_4);
  rtc_gpio_set_direction(GPIO_NUM_4, RTC_GPIO_MODE_OUTPUT_ONLY);

  //calculate the actual ULP clock
  unsigned long rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 1000);
  unsigned long rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;

  Serial.print("Real RTC clock: ");
  Serial.println(rtc_fast_freq_hz);

  // The assembly code put onto the ULP
const ulp_insn_t instructions[] = {
  M_LABEL(1),         // Program loop to here
  I_WR_REG_BIT(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + GPIO_RTC_OFFSET, 1), // Toggle LED OFF
  I_MOVI(R0, 800),    // Set the off counter to 800
  M_LABEL(2),         // Off loop label
  I_DELAY(10000),     // Off loop delay
  I_SUBI(R0, R0, 1),  // Subtract the off counter
  M_BGE(2, 1),        // Branch to off loop if r0 >= 1
  I_WR_REG_BIT(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + GPIO_RTC_OFFSET, 1), // Toggle LED ON
  I_MOVI(R0, 200),    // Set the on counter to 200
  M_LABEL(3),         // On loop label
  I_DELAY(10000),     // On loop delay
  I_SUBI(R0, R0, 1),  // Subtract the on counter
  M_BGE(3, 1),        // Branch to on loop if r0 >= 1
  M_BX(1)             // Branch to start
};

  // Load the assembly code onto the ULP memory
  size_t load_addr = 0;
  size_t size = sizeof(instructions)/sizeof(ulp_insn_t);
  esp_err_t error = ulp_process_macros_and_load(load_addr, instructions, &size);

  // Make sure there were no errors
  if(error != ESP_OK){
    Serial.println("Error loading ULP code");
    return;
  }

  // Start the ULP and check for errors
  error = ulp_run(0);
  if(error != ESP_OK){
    Serial.println("Error running ULP code");
    return;
  }

  Serial.println("ULP has been programmed!");
}

/**
 * @brief 
 *    Basic arduino setup function
 * 
 * @details 
 *    This is where you would put other setup functions
 * 
 */
void setup() {
  // Set up a serial monitor for debugging
  Serial.begin(115200);
  Serial.println("Starting the ULP");

  // Our ULP initializer program
  startULP();
}

/**
 * @brief 
 *    Basic Arduino loop function
 * 
 * @details 
 *    Add your program here! The ULP will not cause interrupts or interfeere with timing!
 * 
 */
void loop() {
  // put your main code here, to run repeatedly:
  // This code is not effected by the ULP unless a stop code is used. Neat!
}