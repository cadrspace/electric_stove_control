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

const int16_t T_MIN = 0;   // C
const int16_t T_MAX = 250; // C

// PID
double setpoint = 30, input, output;
double kp = 2, ki = 5, kd = 1;
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);
const int16_t WINDOW_SIZE = 5000;
uint32_t window_start_time;

void setup() {
  Serial.begin(9600);
  pinMode(BTN_T_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_T_DOWN_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);

  window_start_time = millis();
  pid.SetOutputLimits(0, WINDOW_SIZE);

  //turn the PID on
  pid.SetMode(AUTOMATIC);
}

int16_t get_thermistor_value() {
  return analogRead(THERMISTOR_PIN);
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
  if (! digitalRead(BTN_T_UP_PIN) && (setpoint < T_MAX)) {
    setpoint++;
  } else if (! digitalRead(BTN_T_DOWN_PIN) && (setpoint > T_MIN)) {
    setpoint--;
  }
}

float get_temp() {
  int16_t raw_value = get_thermistor_value();
  float avg = value_to_resistance(raw_value);
  Serial.print("Thermistor resistance: ");
  Serial.println(avg);
  return steinhart(avg);
}

void loop() {
  input = get_temp();
  Serial.print("Temperature: ");
  Serial.println(input);

  update_setpoint_x();
  Serial.print("Limit: ");
  Serial.println(setpoint);

  pid.Compute();

  if ((millis() - window_start_time) > WINDOW_SIZE) {
    window_start_time += WINDOW_SIZE;
  }
  if ((output < millis()) - window_start_time) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }
  delay(500);
}
