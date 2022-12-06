

// External libraries
#include <stdlib.h>
#include <Arduino.h>

// My libraries
extern "C" {
  #include <inverse_kinematics.h>
}
// Pin definitions:
#define PIN_STEP_A 4
#define PIN_DIR_A 5
#define PIN_EN_A 10

#define LED_PIN 2

#define PIN_STEP_B 6
#define PIN_DIR_B 7
#define PIN_EN_B 11

#define H_PULSE 2
#define PULSE 20
#define PULSE_2 28

// Circle
#define CIRCLE_R 300
#define CIRCLE_N 15

// Rectangle
#define R_SIDE 500
#define R_PULSE 200

// Stepper motor
#define M_ROT_STEPS 6400

char message[2];

void Serial_clear() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
}

void stepper_step(bool dir, uint8_t pin_dir, uint8_t pin_step, uint32_t pulse) {
  digitalWrite(pin_dir, dir);
  digitalWrite(pin_step, HIGH);
  delayMicroseconds(H_PULSE);
  digitalWrite(pin_step, LOW);
  delayMicroseconds(pulse - H_PULSE);
}

void stepper_step_ab(bool dir_a, bool dir_b, uint32_t pulse, uint8_t pin_dir_a, uint8_t pin_step_a, uint8_t pin_dir_b, uint8_t pin_step_b) {
  digitalWrite(pin_dir_a, dir_a);
  digitalWrite(pin_dir_b, dir_b);
  digitalWrite(pin_step_a, HIGH);
  digitalWrite(pin_step_b, HIGH);
  delayMicroseconds(H_PULSE);
  digitalWrite(pin_step_a, LOW);
  digitalWrite(pin_step_b, LOW);
  delayMicroseconds(pulse - H_PULSE);
}

void enable_motors(bool on_off) {
  digitalWrite(PIN_EN_A, on_off);
  digitalWrite(PIN_EN_B, on_off);
}
/*
void linear_move(int x_prev, int y_prev, int x, int y) {
  int dx = x - x_prev;
  int dy = y - y_prev;

  int x_dir = dx > 0;
  int y_dir = dy > 0;

  dx = abs(dx);
  dy = abs(dy);
  x = 0;
  y = 0;

  int steps = dx > dy ? dx : dy;
  float k;
  if (dx == 0) {
    k = FLT_MAX;
  } else {
    k = (float)dy / dx;
  }

  for (int i = 1; i < steps + 1; i++) {
    if (dx < dy) {
      if  (x + 0.5 < i / k) {
        stepper_step_ab(x_dir, y_dir, PULSE_2, PIN_DIR_A, PIN_STEP_A, PIN_DIR_B, PIN_STEP_B);
        x++;
        y++;
        continue;
      }
      stepper_step(y_dir, R_PULSE, PIN_STEP_B, R_PULSE);
      y++;
    } else {
      if  (y + 0.5 < i * k) {
        stepper_step_ab(x_dir, y_dir, PULSE_2, PIN_DIR_A, PIN_STEP_A, PIN_DIR_B, PIN_STEP_B);
        x++;
        y++;
        continue;
      }
      stepper_step(x_dir, R_PULSE, PIN_STEP_A, R_PULSE);
      x++;
    }
  }
}

*/
void linear_move(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0);
  int dy = -abs(y1 - y0);

  int8_t sx = (x0 < x1) ? 1 : -1;
  int8_t sy = (y0 < y1) ? 1 : -1;

  int error = dx + dy;
  int e2;
  bool flag_A, flag_B;

  digitalWrite(PIN_DIR_A, x0 < x1);
  digitalWrite(PIN_DIR_B, y0 < y1);

  while (true) {
    if ((x0 == x1) && (y0 == y1)) {
        break;
    }
    e2 = 2 * error;

    if (e2 >= dy) {
      if (x0 == x1) { 
          break;
      }
      error += dy;
      x0 += sx;
      digitalWrite(PIN_STEP_A, HIGH);
      flag_A = 1;
    }

    if (e2 <= dx) {
      if (y0 == y1) {
          break; 
      }
      error += dx;
      y0 += sy;
      digitalWrite(PIN_STEP_B, HIGH);
      flag_B = 1;
    }

    delayMicroseconds(H_PULSE);
    digitalWrite(PIN_STEP_A, LOW);
    digitalWrite(PIN_STEP_B, LOW);

    if ((flag_A == 1) && (flag_B == 1)) {
      delayMicroseconds(PULSE_2);
    } else {
      delayMicroseconds(PULSE);
    }
    flag_A = 0;
    flag_B = 0;
  }
}


void heart() {
  static bool led = false;
  if (led) {
    digitalWrite(LED_PIN, LOW);
    led = false;
  } else {
    digitalWrite(LED_PIN, HIGH);
    led = true;
  }
  const int n_h = 66;
  static int data_h[n_h] = {0, 0, 28, 40, 66, 86, 117, 138, 173, 185, 198, 213, 211, 242,
                    215, 282, 208, 313, 189, 343, 160, 367, 123, 380, 89, 380,
                    58, 370, 35, 355, 17, 336, 0, 300, -17, 336, -35, 355, -58, 370,
                    -89, 380, -123, 380, -160, 367, -189, 343, -208, 313, -215, 282,
                    -211, 242, -198, 213, -173, 185, -117, 138, -66, 86, -28, 40, 0, 0
                };

  static bool flag = false;

  if (flag == false) {
    for (int i = 0; i < n_h/2; i++) {
      int temp_a, temp_b;
      temp_a = (int)(inverse_kinematics(data_h[i*2], data_h[i*2+1], 0) * M_ROT_STEPS / M_PI);
      temp_b = (int)(inverse_kinematics(data_h[i*2], data_h[i*2+1], 1) * M_ROT_STEPS / M_PI);
      data_h[i*2] = temp_a;
      data_h[i*2+1] = temp_b;
    }
    flag = true;
  }

  int p_a = 0;
  int p_b = 0;

  for (int i = 0; i < n_h/2; i++) {
    linear_move(p_a, p_b, data_h[i*2], data_h[i*2+1]);
    p_a = data_h[i*2];
    p_b = data_h[i*2+1];
  }
}

void rectangle(int side) {
  for (int i = 0; i < side; i++) {
    stepper_step(1, PIN_DIR_A, PIN_STEP_A, R_PULSE);
  }
  for (int i = 0; i < side; i++) {
    stepper_step(0, PIN_DIR_B, PIN_STEP_B, R_PULSE);
  }
  for (int i = 0; i < side; i++) {
    stepper_step(0, PIN_DIR_A, PIN_STEP_A, R_PULSE);
  }
  for (int i = 0; i < side; i++) {
    stepper_step(1, PIN_DIR_B, PIN_STEP_B, R_PULSE);
  }
}

void circle(int R, int n) {
  int steps_a, steps_b, x, y;
  int p_a = R;
  int p_b = 0;
  for (int i = 1; i < n + 1; i++) {
    x = sin(M_PI * 2 * (float)i / n) * R;
    y = cos(M_PI * 2 * (float)i / n) * R;
    steps_a = (int)(inverse_kinematics(x, y, 0) * M_ROT_STEPS / M_PI);
    steps_b = (int)(inverse_kinematics(x, y, 1) * M_ROT_STEPS / M_PI);

    linear_move(p_a, p_b, steps_a, steps_b);
    p_a = steps_a;
    p_b = steps_b;
  }
}

void command(byte dir) {
  switch (dir) {
    case 0:
      //random_move();
      rectangle(R_SIDE);
      break;

    case 1:
      stepper_step_ab(1, 0, PULSE_2, PIN_DIR_A, PIN_STEP_A, PIN_DIR_B, PIN_STEP_B);
      break;

    case 2: // DOLU
      stepper_step(1, PIN_DIR_A, PIN_STEP_A, PULSE);
      break;

    case 3:
      stepper_step_ab(1, 1, PULSE_2,  PIN_DIR_A, PIN_STEP_A, PIN_DIR_B, PIN_STEP_B);
      break;
    case 4: // LEVO
      stepper_step(0, PIN_DIR_B, PIN_STEP_B, PULSE);
      break;

    case 5:

      break;
    case 6: // PRAVO
      stepper_step(1, PIN_DIR_B, PIN_STEP_B, PULSE);
      break;

    case 7:
      stepper_step_ab(0, 0, PULSE_2, PIN_DIR_A, PIN_STEP_A, PIN_DIR_B, PIN_STEP_B);
      break;

    case 8: // NAHORU
      stepper_step(0, PIN_DIR_A, PIN_STEP_A, PULSE);
      break;

    case 9:
      stepper_step_ab(0, 1, PULSE_2, PIN_DIR_A, PIN_STEP_A, PIN_DIR_B, PIN_STEP_B);
      break;

    case 10:
      circle(CIRCLE_R, CIRCLE_N);
      break;

    case 11:

      heart();
     break;

    default:
      break;
  }
}


void setup() {
  pinMode(PIN_STEP_A, OUTPUT);
  pinMode(PIN_STEP_B, OUTPUT);
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(250000);
  delay(500);

  enable_motors(true);
}

void loop() {
  if (Serial.available() > 0) {
    int b = Serial.readBytes(message, 2);
    
      if (b != 2) {
      Serial.print(message);
      Serial.print("; Delka: ");
      Serial.println(b);
      Serial_clear();
      return;
      }
    if (message[0] == 'S') {
      command(message[1]);
    }
    Serial_clear();
  }
}