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

#define START_BYTE 0xA5          // Początek ramki
#define START_BYTE_RECEIVE 0xB5  // Początek ramki odbieranej
#define FRAME_LENGTH 35          // Długość danych w ramce

// Długość ramki odbiorczej
#define RX_FRAME_LENGTH 7

enum DataType : uint8_t {
    TYPE_PWM = 0x01,
    TYPE_KP = 0x02,
    TYPE_KI = 0x03,
    TYPE_KD = 0x04
    // Dodaj więcej w razie potrzeby
};

enum Mode : uint8_t {
    Manual = 0x00,
    Automatic = 0x01
};

// Bufor odbiorczy
uint8_t rx_buffer[RX_FRAME_LENGTH];
uint8_t rx_index = 0;
bool receiving = false;

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
int pwr = 0;

float kp = 0;
float ki = 0;
float kd = 0;

// PID
float eintegral = 0;
float eprev = 0;

unsigned long lastPwrUpdate = 0;
bool pwrIncreasing = true;

// Deklaracje funkcji
void read_encoder();
void IRAM_ATTR set_motor(Direction, int, int, int, int);
void send_qt_binary(float, int, float, float, float, float, float, float, Mode);
void debug_data(uint8_t *, size_t length);
// Funkcja obliczająca checksum (XOR wszystkich bajtów)
uint8_t calculate_checksum(uint8_t *, int);
void parse_qt_frame(uint8_t *frame);
void read_qt_binary();
void simulate_qt_frame_debug();

void setup() {
    Serial.begin(115200);
    Wire.begin();
    // while (!ina219.begin()) {
    //     Serial.println("Failed to find INA219 chip");
    //     delay(100);
    // }
    pinMode(ENCODER_AY, INPUT);
    pinMode(ENCODER_BG, INPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_AY), read_encoder, RISING);
    pinMode(2, OUTPUT);
    // ina219.setCalibration_32V_1A();
    delay(500);
}

void loop() {
    // simulate_qt_frame_debug();
    read_qt_binary();
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
    v1_filt =
        1.5610 * v1_filt - 0.6414 * v2_filt + 0.0201 * v1 + 0.0402 * v1_prev + 0.0201 * v2_prev;
    v2_filt = v1_filt;
    v2_prev = v1_prev;
    v1_prev = v1;
    // Wartość zadana prędkości (150 RPM sinusoidalnie zmieniana) taki przebieg prostokątny
    float vt = 150 * (sin(curr_t / 1e6) > 0);
    // Serial.print(">v_target:");
    // Serial.println(vt);
    // Parametry PID
    kp = 1;
    ki = 1;
    kd = 0.1;

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
    // pwr = (int)fabs(u);

    // if (millis() - lastPwrUpdate >= 1000) {
    //     lastPwrUpdate = millis();
    //     pwr = random(0, 256);
    // }

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

    // shunt_voltage = ina219.getShuntVoltage_mV();
    // bus_voltage = ina219.getBusVoltage_V();
    // current_mA = ina219.getCurrent_mA();
    // power_mW = ina219.getPower_mW();
    float loadvoltage = bus_voltage + (shunt_voltage / 1000);

    // Serial.print(">Bus voltage:");
    // Serial.println(bus_voltage);
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

    // Transmisja danych do Qt
    send_qt_binary(v1_filt, pwr, current_mA, load_voltage, power_mW, kp, ki, kd, Mode::Automatic);
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

void send_qt_binary(float rpm, int pwm, float current, float voltage, float power, float k_p, float k_i, float k_d, Mode mode) {
    
    uint8_t start_byte = START_BYTE;
    float f_pwm = static_cast<float>(pwm);
    
    // 1 + 1 + 5*4 23
    // Zbiór bajtów do wysłania
    uint8_t buffer[FRAME_LENGTH];  // 35 bajty (start byte + dane + tryb + checksum)

    // Kopiowanie danych do bufora
    memcpy(buffer, &start_byte, sizeof(start_byte));   // Start byte
    memcpy(buffer + 1, &rpm, sizeof(rpm));           // RPM
    memcpy(buffer + 5, &f_pwm, sizeof(f_pwm));           // PWM
    memcpy(buffer + 9, &current, sizeof(current));  // Current
    memcpy(buffer + 13, &voltage, sizeof(voltage));  // Voltage
    memcpy(buffer + 17, &power, sizeof(power));      // Power
    memcpy(buffer + 21, &k_p, sizeof(k_p));
    memcpy(buffer + 25, &k_i, sizeof(k_i));
    memcpy(buffer + 29, &k_d, sizeof(k_d));
    memcpy(buffer + 33, &mode, sizeof(mode));
    
    
    // Oblicz checksum
    uint8_t checksum = calculate_checksum(buffer, sizeof(buffer) - 1);  

    // Dodanie checksum do ramki
    memcpy(buffer + 34, &checksum, sizeof(checksum));

    // Wysyłanie ramki
    Serial.write(buffer, sizeof(buffer));
    // debug_data(buffer,sizeof(buffer));
}

uint8_t calculate_checksum(uint8_t *data, int length) {
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void read_qt_binary() {
    while (Serial.available()) {
        uint8_t byte = Serial.read();

        if (!receiving) {
            if (byte == START_BYTE_RECEIVE) {
                receiving = true;
                rx_index = 0;
                rx_buffer[rx_index++] = byte;
            }
        } else {
            if (rx_index < RX_FRAME_LENGTH) {
                rx_buffer[rx_index++] = byte;
            }

            if (rx_index >= RX_FRAME_LENGTH) {
                receiving = false;
                // Sprawdź checksum
                uint8_t checksum = calculate_checksum(rx_buffer, sizeof(rx_buffer) - 1);
                if (checksum == rx_buffer[RX_FRAME_LENGTH - 1]) {
                    parse_qt_frame(rx_buffer);

                } else {
                    Serial.println("Błąd: zła suma kontrolna!");
                }
            }
        }
    }
}

void parse_qt_frame(uint8_t *frame) {
    uint8_t type = frame[1];
    float value;
    memcpy(&value, frame + 2, sizeof(float));

    switch (type) {
        case TYPE_PWM:
            Serial.println(value);
            pwr = static_cast<int>(value);
            break;
        case TYPE_KP:
            Serial.println(value);
            // kp = value;
            break;
        case TYPE_KI:
            Serial.println(value);
            // ki = value;
            break;
        case TYPE_KD:
            Serial.println(value);
            // kd = value;
            break;
        default:
            Serial.println(type, HEX);
            break;
    }
}

void debug_data(uint8_t *data, size_t length) {
    Serial.print("Sent Frame: ");
    for (size_t i = 0; i < length; ++i) {
        if (data[i] < 0x10) Serial.print("0");  // Wyrównanie do 0x0X
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}
void simulate_qt_frame_debug() {
    // Ramka: START_BYTE, typ, float(250.0), checksum
    uint8_t test_frame[7] = {0xB5, 0x01, 0x00, 0x00, 0x7A, 0x43, 0x8D};

    // Oblicz checksum dla kontroli
    uint8_t checksum = calculate_checksum(test_frame, 6);
    if (checksum != test_frame[6]) {
        Serial.println("Uwaga: Błędna suma kontrolna w symulowanej ramce!");
        Serial.print("Obliczona suma: 0x");
        Serial.println(checksum, HEX);
    }

    parse_qt_frame(test_frame);
}
