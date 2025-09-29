#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"

#define MOTOR1_TIMER            LEDC_TIMER_0 // 0, 1, 2, or 3
#define MOTOR2_TIMER            LEDC_TIMER_1 // 0, 1, 2, or 3
#define LEDC_MODE               LEDC_LOW_SPEED_MODE // SPEED_MODE, HIGH_SPEED_MODE, or LOW_SPEED_MODE
#define MOTOR1_PUL              13 // Define the output GPIO
#define MOTOR1_DIR              12 // Define the output GPIO
#define MOTOR1_CHANNEL          LEDC_CHANNEL_0
#define MOTOR2_CHANNEL          LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 8 bits (Actually sets it to 6 bits?)
#define LEDC_DUTY               (128) // Set duty to 50%. (2 ** 12) * 50% = 2048
#define LEDC_FREQUENCY          (100) // Frequency in Hertz. Set frequency at 4 kHz

/* Warning:f
 * For ESP32, ESP32S2, ESP32S3, ESP32C3, ESP32C2, ESP32C6, ESP32H2, ESP32P4 targets,
 * when LEDC_DUTY_RES selects the maximum duty resolution (i.e. value equal to SOC_LEDC_TIMER_BIT_WIDTH),
 * 100% duty cycle is not reachable (duty cannot be set to (2 ** SOC_LEDC_TIMER_BIT_WIDTH)).
 */

bool UP = false;
bool DOWN = false;
bool LEFT = false;
bool RIGHT = false;
bool STANDBY = true;
uint32_t FREQ = 500;
uint32_t DIR = 1; // 0 = 
uint32_t THRESHOLD[2] = {500, 5000};

void setup() {
  // Set the LEDC peripheral configuration
  config_motor(MOTOR1_PUL, MOTOR1_TIMER, MOTOR1_CHANNEL);

  pinMode(MOTOR1_DIR, OUTPUT);
  digitalWrite(MOTOR1_DIR, HIGH);

  Serial.begin(9600);

}

void loop() {
  /////////////////////////////////////////
  ////// HIGH IS DOWN, LOW IS UP //////////
  /////////////////////////////////////////
  
  //update_freq(MOTOR1_TIMER, 500);
  //digitalWrite(MOTOR1_DIR, LOW);
  //delay(1000);
  
  if (Serial.available()) {
      int key = Serial.read();

      switch (key) {
          case '1': UP = true; DOWN = false; LEFT = false; RIGHT = false; STANDBY = false; break;
          case '2': DOWN = true; UP = false; LEFT = false; RIGHT = false; STANDBY = false; break;
          case '3': LEFT = true; UP = false; DOWN = false; RIGHT = false; STANDBY = false; break;
          case '4': RIGHT = true; UP = false; DOWN = false; LEFT = false; STANDBY = false; break;
          case '5': STANDBY = true; UP = false; DOWN = false; LEFT = false; RIGHT = false; break;
      }

      if ((UP) || (DOWN) || (LEFT) || (RIGHT)) {
        FREQ = move_shoulder(MOTOR1_TIMER, MOTOR1_DIR, FREQ, UP, DOWN, LEFT, RIGHT);
        update_duty(MOTOR1_CHANNEL, 50);
      }
      else if (STANDBY) {
        update_duty(MOTOR1_CHANNEL, 0);
      }
      Serial.printf("Key: %c | Freq: %lu Hz | Dir: %lu\n", key, FREQ, DIR);
  }
}

uint32_t move_shoulder(ledc_timer_t TIMER, uint32_t DIRPIN, uint32_t FREQ, bool UP, bool DOWN, bool LEFT, bool RIGHT)
{
  // update_duty(MOTOR1_CHANNEL, 50);
  // if ((UP == false) && (DOWN == false) && (LEFT == false) && (RIGHT == false)) {
  //   update_duty(MOTOR1_CHANNEL, 0);
  // }
  
  // update_duty(MOTOR1_CHANNEL, 50);
  // else {
  //   update_duty(MOTOR1_CHANNEL, 50);
  // }

  Serial.println(UP);
  Serial.println(DOWN);
  Serial.println(LEFT);
  Serial.println(RIGHT);

  if (UP) {
    DIR = 0;
  }
  if (DOWN) {
    DIR = 1;
  }
  if (LEFT && FREQ > THRESHOLD[0]) {
    FREQ -= 100;
  }
  if (RIGHT && FREQ < THRESHOLD[1]) {
    FREQ += 100;
  }

  digitalWrite(DIRPIN, DIR);
  update_freq(TIMER, FREQ);

  return FREQ;
} 

static void config_motor(uint32_t OUTPUT_IO, ledc_timer_t TIMER, ledc_channel_t CHANNEL)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    // MUST BE DONE IN THIS ORDER
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = OUTPUT_IO,
        .speed_mode     = LEDC_MODE,
        .channel        = CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = TIMER,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void update_duty(ledc_channel_t CHANNEL, int duty)
{
    double BX_duty = pow(2, 8) * (double(duty) / 100);
    // Set duty
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, CHANNEL, uint32_t(BX_duty)));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, CHANNEL));
}

void update_freq(ledc_timer_t TIMER, uint32_t FREQ)
{
    // Set frequency
    ESP_ERROR_CHECK(ledc_set_freq(LEDC_MODE, TIMER, FREQ));
}