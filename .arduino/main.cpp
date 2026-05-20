#include <Arduino.h>

// ─── Pin Definitions ──────────────────────────────────────────────────────────
// H-Bridge 1 — left wheel
constexpr uint8_t IN1_PIN = PA0;      // Direction A
constexpr uint8_t IN2_PIN = PA1;      // Direction B
constexpr uint8_t PWM_LEFT_PIN = PB6; // Must be PWM-capable

// H-Bridge 2 — right wheel
constexpr uint8_t IN3_PIN = PA2;       // Direction A
constexpr uint8_t IN4_PIN = PA3;       // Direction B
constexpr uint8_t PWM_RIGHT_PIN = PB7; // Must be PWM-capable

// ─── Protocol ─────────────────────────────────────────────────────────────────
// Frame: [0xAA] [0x55] [L_lo] [L_hi] [R_lo] [R_hi]
// Left/Right PWM are signed 16-bit little-endian, range -100 to +100
constexpr uint8_t HEADER1 = 0xAA;
constexpr uint8_t HEADER2 = 0x55;
constexpr uint8_t MSG_SIZE = 6;
constexpr uint32_t BAUD_RATE = 115200;

// ─── Watchdog ─────────────────────────────────────────────────────────────────
constexpr uint32_t WATCHDOG_MS = 500; // Stop motors if silent for 500 ms

// ─── State ────────────────────────────────────────────────────────────────────
static uint8_t rx_buf[MSG_SIZE];
static uint8_t rx_idx = 0;
static uint32_t last_cmd_time = 0;
static bool motors_active = false;

// ─── Motor Driver ─────────────────────────────────────────────────────────────
// pwm: -100 (full reverse) … 0 (stop) … +100 (full forward)
static void setMotor(uint8_t pinA, uint8_t pinB, uint8_t pwmPin, int16_t pwm)
{
    // Clamp input
    if (pwm > 100)
        pwm = 100;
    if (pwm < -100)
        pwm = -100;

    // Map magnitude 0-100 → 0-255
    uint8_t duty = (uint8_t)((uint32_t)abs(pwm) * 255 / 100);

    if (pwm > 0)
    {
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
    }
    else if (pwm < 0)
    {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, HIGH);
    }
    else
    {
        // Active brake: both LOW, PWM = 0
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
        duty = 0;
    }

    analogWrite(pwmPin, duty);
}

static inline void stopAll()
{
    setMotor(IN1_PIN, IN2_PIN, PWM_LEFT_PIN, 0);
    setMotor(IN3_PIN, IN4_PIN, PWM_RIGHT_PIN, 0);
    motors_active = false;
    digitalWrite(LED_BUILTIN, LOW);
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup()
{
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    pinMode(PWM_LEFT_PIN, OUTPUT);
    pinMode(PWM_RIGHT_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    stopAll();

    Serial1.begin(BAUD_RATE);

    last_cmd_time = millis();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop()
{
    // ── Packet parser ───────────────────────────────────────────────────────
    while (Serial1.available())
    {
        uint8_t c = (uint8_t)Serial1.read();

        switch (rx_idx)
        {
        case 0:
            if (c == HEADER1)
                rx_idx = 1;
            break;

        case 1:
            if (c == HEADER2)
                rx_idx = 2;
            else
                rx_idx = 0; // restart search
            break;

        default:
            rx_buf[rx_idx++] = c;

            if (rx_idx == MSG_SIZE)
            {
                // Reconstruct signed 16-bit little-endian values
                int16_t left_pwm = (int16_t)((uint16_t)rx_buf[3] << 8 | rx_buf[2]);
                int16_t right_pwm = (int16_t)((uint16_t)rx_buf[5] << 8 | rx_buf[4]);

                setMotor(IN1_PIN, IN2_PIN, PWM_LEFT_PIN, left_pwm);
                setMotor(IN3_PIN, IN4_PIN, PWM_RIGHT_PIN, right_pwm);

                last_cmd_time = millis();
                motors_active = true;
                digitalWrite(LED_BUILTIN, HIGH);

                rx_idx = 0;
            }
            break;
        }
    }

    // ── Watchdog ────────────────────────────────────────────────────────────
    if (motors_active && (millis() - last_cmd_time > WATCHDOG_MS))
    {
        stopAll();
    }
}
