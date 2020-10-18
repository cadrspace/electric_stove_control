#include <PID_v1.h>

const int8_t RELAY_PIN = 3;

const int32_t THERMISTOR_PIN      = A0;
const int32_t THERMISTOR_NOMINAL  = 100000; // Ohms
const int32_t RESISTOR_NOMINAL    = 82000;  // Ohms
const int32_t TEMPERATURE_NOMINAL = 25;  // C
const int32_t NUM_SAMPLES         = 5;
const int32_t B_COEFFICIENT       = 3950;

const int8_t BTN_T_UP_PIN   = 2;
const int8_t BTN_T_DOWN_PIN = 3;

uint16_t samples[NUM_SAMPLES];

const int16_t T_MIN = 0;   // C
const int16_t T_MAX = 250; // C
uint8_t t_limit = 50;      // C

// PID
double setpoint = 30, input, output;
double kp = 2, ki = 5, kd = 1;
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);
const int16_t WINDOW_SIZE = 5000;
uint32_t window_start_time;

void setup() {
  Serial.begin(9600);
  pinMode(BTN_T_UP_PIN, INPUT);
  pinMode(BTN_T_DOWN_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  window_start_time = millis();
  pid.SetOutputLimits(0, WINDOW_SIZE);

  //turn the PID on
  pid.SetMode(AUTOMATIC);
}

int16_t get_thermistor_value() {
  return analogRead(THERMISTOR_PIN);
}

void take_samples_x() {
  for (uint16_t idx = 0; idx < NUM_SAMPLES; ++idx) {
    samples[idx] = get_thermistor_value();
    delay(10);
  }
}

float calculate_avg() {
  float sum = 0;
  for (uint8_t idx = 0; idx < NUM_SAMPLES; ++idx) {
    sum += samples[idx];
  }
  return sum / NUM_SAMPLES;
}

float value_to_resistance(float val) {
  return RESISTOR_NOMINAL / (1023 / val - 1);
}

// Converts resistance to temperature.
float steinhart(float resistance) {
  float result;
  result = resistance / THERMISTOR_NOMINAL; // (R/Ro)
  result = log(result);                     // ln(R/Ro)
  result /= B_COEFFICIENT;                  // 1/B * ln(R/Ro)
  result += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
  result = 1.0 / result;                    // Invert
  result -= 273.15;                         // convert to C
  return result;
}

void update_setpoint_x() {
  if (digitalRead(BTN_T_UP_PIN) && (t_limit < T_MAX)) {
    setpoint++;
  } else if (digitalRead(BTN_T_DOWN_PIN) && (t_limit > T_MIN)) {
    setpoint--;
  }
}

float get_temp() {
  take_samples_x();
  float avg = calculate_avg();
  Serial.print("Average: ");
  Serial.println(avg);

  avg = value_to_resistance(avg);
  Serial.print("Thermistor resistance: ");
  Serial.println(avg);

  return steinhart(avg);
}

void loop() {
  input = get_temp();
  Serial.print("Temperature: ");
  Serial.println(input);
  pid.Compute();

  update_setpoint_x();
  Serial.print("Limit: ");
  Serial.println(setpoint);

  delay(1000);
}
