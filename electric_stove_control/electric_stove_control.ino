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

void setup() {
  Serial.begin(9600);
  pinMode(BTN_T_UP_PIN, INPUT);
  pinMode(BTN_T_DOWN_PIN, INPUT);
}

void take_samples_x() {
  for (uint16_t idx = 0; idx < NUM_SAMPLES; ++idx) {
    samples[idx] = analogRead(THERMISTOR_PIN);
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

void update_t_limit_x() {
  if (digitalRead(BTN_T_UP_PIN) && (t_limit < T_MAX)) {
    t_limit++;
  } else if (digitalRead(BTN_T_DOWN_PIN) && (t_limit > T_MIN)) {
    t_limit--;
  }
}

void loop() {
  take_samples_x();
  float avg = calculate_avg();
  Serial.print("Average: ");
  Serial.println(avg);

  avg = value_to_resistance(avg);
  Serial.print("Thermistor resistance: ");
  Serial.println(avg);

  float temp = steinhart(avg);
  Serial.print("Temperature: ");
  Serial.println(temp);

  update_t_limit_x();
  Serial.print("Limit: ");
  Serial.println(t_limit);

  delay(1000);
}
