#include <Adafruit_INA219.h>
#include <Arduino.h>
// Piny dla enkodera
#define ENCODER_AY 19  // Kanał A enkodera (żółty) - przerwanie na zbocze narastające CW
#define ENCODER_BG 23  // Kanał B enkodera (zielony) - odczytywany w ISR CCW

// Piny sterujące silnikiem
#define ENA 18  // Pin PWM (szybkość)
#define IN1 16  // Kierunek jazdy
#define IN2 17

#define ENCODER_PULSES_PER_REV 16  // Liczba impulsów na pełny obrót enkodera
#define GEAR_RATIO 9.6             // Przekładnia 9.6

// Kierunek obrotu
enum Direction { LEFT, RIGHT };

// Obiekt do pomiaru napięcia i prądu
Adafruit_INA219 ina219;
Direction dir = LEFT;

// Zmienne dla pomiarów z INA219
float shunt_voltage = 0;
float bus_voltage = 0;
float current_mA = 0;
float load_voltage = 0;
float power_mW = 0;

// Zmienne globalne do obliczeń prędkości
long prev_t = 0;   // Poprzedni czas (mikrosekundy)
int pos_prev = 0;  // Poprzednia pozycja enkodera

// Zmienna pozycji z przerwania - volatile, bo zmieniana w ISR
volatile long pos_i = 0;

// Filtrowanie dolnoprzepustowe
float v1_filt = 0;
float v1_prev = 0;
float v2_filt = 0;
float v2_prev = 0;

// PID
float eintegral = 0;
float eprev = 0;

// Deklaracje funkcji
void read_encoder();
void IRAM_ATTR set_motor(Direction, int, int, int, int);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    while (!ina219.begin()) {
        Serial.println("Failed to find INA219 chip");
        delay(100);
    }
    pinMode(ENCODER_AY, INPUT);
    pinMode(ENCODER_BG, INPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_AY), read_encoder, RISING);
    ina219.setCalibration_32V_1A();
    delay(500);
}

void loop() {
    int pos = 0;

    // Odczyt pozycji z zachowaniem bezpieczeństwa przerwań
    noInterrupts();
    pos = pos_i;
    interrupts();

    // Obliczenie czasu deltaT
    long curr_t = micros();
    float delta_t = ((float)(curr_t - prev_t)) / 1.0e6;

    // Obliczenie prędkości (imp/s)
    float velocity1 = (pos - pos_prev) / delta_t;
    pos_prev = pos;
    prev_t = curr_t;

    // Konwersja prędkości do RPM
    float v1 = velocity1 / GEAR_RATIO * ENCODER_PULSES_PER_REV * 60;

    // Filtrowanie dolnoprzepustowe (5 Hz)
    v1_filt = 1.5610 * v1_filt - 0.6414 * v2_filt + 0.0201 * v1 + 0.0402 * v1_prev + 0.0201 * v2_prev;
    v2_filt = v1_filt;
    v2_prev = v1_prev;
    v1_prev = v1;

    // Wartość zadana prędkości (150 RPM sinusoidalnie zmieniana) taki przebieg prostokątny
    float vt = 150 * (sin(curr_t / 1e6) > 0);
    Serial.print(">v_target:");
    Serial.println(vt);
    // Parametry PID
    float kp = 1;
    float ki = 1;
    float kd = 0.1;

    // Obliczenia regulatora PID
    float e = vt - v1_filt;
    eintegral = eintegral + e * delta_t;
    float dedt = (e - eprev) / delta_t;
    float u = kp * e + ki * eintegral + kd * dedt;
    eprev = e;

    // Kierunek obrotu
    dir = LEFT;
    if (u < 0) {
        dir = RIGHT;
    }

    // Wartość PWM (ograniczona do 255)
    int pwr = (int)fabs(u);
    if (pwr > 255) {
        pwr = 255;
    }

    // Debug prędkości
    // Serial.println(pwr);
    set_motor(dir, ENA, pwr, IN1, IN2);

    // Serial.print(">Vel1:");
    // Serial.println(v1_filt);

    // Read voltage and current from INA219.
    // Serial.print(">EN_Yellow:");
    // Serial.println(5 * a);
    // Serial.print(">EM_green:");
    // Serial.println(5 * b);

    // shuntvoltage = ina219.getShuntVoltage_mV();
    bus_voltage = ina219.getBusVoltage_V();
    // current_mA = ina219.getCurrent_mA();
    // power_mW = ina219.getPower_mW();
    // loadvoltage = busvoltage + (shuntvoltage / 1000);

    Serial.print(">Bus voltage:");
    Serial.println(bus_voltage);
    // Serial.println(" V");
    // Serial.print("Shunt voltage:       ");
    // Serial.print(shuntvoltage);
    // Serial.println(" mV");
    // Serial.print(">Load voltage:");
    // Serial.println(loadvoltage);
    // Serial.println(" V");
    // Serial.print(">Current:");
    // Serial.println(current_mA);
    // Serial.println(" mA");
    // Serial.print(">Power:");
    // Serial.println(power_mW);
    // Serial.println(" mW");
    // Serial.println("");
    delay(1);
}

/*Czytamy enkoder b jak a będzie miało interupta na rising, jak kręcimy w
prawko czyli CW to A jest przed, czyli w momencie po interupcie B jest = 0 czyli odejmujemy.
Jak CCW to B jest przed A, czyli dodajemy*/
void IRAM_ATTR read_encoder() {
    // Read encoder B when ENCA rises
    int b = digitalRead(ENCODER_BG);
    int increment = 0;
    if (b > 0) {
        // If B is high, increment forward
        increment = 1;
    } else {
        // Otherwise, increment backward
        increment = -1;
    }
    pos_i = pos_i + increment;
}

void set_motor(Direction dir, int pwm, int pwm_val, int in1, int in2) {
    analogWrite(pwm, pwm_val);
    if (dir == LEFT) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (dir == RIGHT) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
}