#include <Arduino.h>
#include <TMCStepper.h>
#include <HardwareSerial.h>
#include "AccelStepper.h"

#include "parameter.h"
#include "gpio.h"
#include "motor_init.h"

HardwareSerial TMCSerial(1); // Menggunakan UART1
HardwareSerial TMCSerial2(2); // Menggunakan UART2

//TMC Library sebagai inisialisasi driver untuk siap menerima perintah step dan dir
TMC2209Stepper driver[] = {
  TMC2209Stepper(&TMCSerial, R_SENSE, 0b00),
  TMC2209Stepper(&TMCSerial, R_SENSE, 0b01),
  TMC2209Stepper(&TMCSerial, R_SENSE, 0b10),
  TMC2209Stepper(&TMCSerial2, R_SENSE, 0b00),
  TMC2209Stepper(&TMCSerial2, R_SENSE, 0b01),
  TMC2209Stepper(&TMCSerial2, R_SENSE, 0b10)
};
// Accel Library sebagai pengirim step dan dir sesuai dengan target posisi dan kecepatan
AccelStepper stepper[] = {
  AccelStepper(AccelStepper::DRIVER, STEP1, DIR1),
  AccelStepper(AccelStepper::DRIVER, STEP2, DIR2),
  AccelStepper(AccelStepper::DRIVER, STEP3, DIR3),
  AccelStepper(AccelStepper::DRIVER, STEP4, DIR4),
  AccelStepper(AccelStepper::DRIVER, STEP5, DIR5),
  AccelStepper(AccelStepper::DRIVER, STEP6, DIR6)
};

struct MotorStruct Joint[NUMBER_OF_JOINTS];
float jointAngleSetpoint[NUMBER_OF_JOINTS] = {0};

// --- Konstanta homing (ubah jika perlu) ---
constexpr float HOME_SPEED  = 800.0f;   // langkah/s
constexpr float HOME_ACCEL  = 1500.0f;   // langkah/s^2
constexpr long  SEARCH_EXTRA_STEPS = 200000; // maks jarak mencari limit
constexpr long  BACKOFF_STEPS = 50;    // mundur setelah switch kena

// -- konstanta saat sistem bekerja --
constexpr float   DEFAULT_ACCEL = 4000.0f;        // steps/s^2
constexpr float   DEFAULT_MAX_SPEED = 6000.0f;    // fallback deg/s
constexpr uint32_t COMMAND_TIMEOUT_MS = 1000;

// ==== Status sistem ====
enum class MotionMode : uint8_t { Idle, MoveJ, Homing, EStop };

struct MotionState {
  MotionMode mode = MotionMode::Idle;
  bool estop = false;
  bool fault = false;
  bool homed = false;
  float q_now_deg[NUMBER_OF_JOINTS] = {};    // posisi terakhir (deg)
  long steps_now[NUMBER_OF_JOINTS] = {};     // posisi (steps)
  long target_steps[NUMBER_OF_JOINTS] = {};
  float maxSpeedSteps[NUMBER_OF_JOINTS] = {};
  float delta_deg[NUMBER_OF_JOINTS] = {};
  float delta_steps[NUMBER_OF_JOINTS] = {};
  float v_global_deg = DEFAULT_MAX_SPEED;
} motion;

// Read All Function on below
void Init_TMC2209_drivers(int num); //Inisialisasi TMC2209 untuk setiap driver
void Init_Steppeer_parameters(); //Inisialisasi parameter Accel Stepper
void handleSerialAngleCommand();
float stepsPerDegree(uint8_t jointIndex);
long angleDegreesToSteps(uint8_t jointIndex, float angleDeg);
bool homeJoint(uint8_t idx);
bool homeJoint2(uint8_t idx);
void processHostCommand();
bool parseMoveJ(const String& line, float (&q_target)[NUMBER_OF_JOINTS], float& v_global);
void planMoveJ(const float (&q_target)[NUMBER_OF_JOINTS], float v_global);
void updatePositionCache();
bool allJointsCompleted();
void handleHomeAll();
void handleStop();
void handleEStop(bool latch);
void publishStatus();
bool homeJointManual(uint8_t idx);

void setup() {

  Init_Joint_1(&Joint[0]);
  Init_Joint_2(&Joint[1]);
  Init_Joint_3(&Joint[2]);
  Init_Joint_4(&Joint[3]);
  Init_Joint_5(&Joint[4]);
  Init_Joint_6(&Joint[5]);

  Serial.begin(115200);
  TMCSerial.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  TMCSerial2.begin(115200, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);

  delay(500); // Wait for serial to initialize  
  Serial.println("TMC2209 Test");

  //Init GPIO Inputs and Outputs
  Init_all_gpio();
  Serial.println("Initialize GPIO done");

  for(int i = 0; i < NUMBER_OF_JOINTS; i++) {
    Init_TMC2209_drivers(i);
    stepper[i].setPinsInverted(Joint[i].direction_reversed != 0);
    delay(90);
  }

  // Initialize stepper parameters
  Init_Steppeer_parameters();
  //homeJoint2(0);
  homeJoint2(1);
  //homeJointManual(0);
  //Home all joints
  // for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
  //   if (!homeJoint(i)) {
  //     Serial.printf("Homing joint %d gagal, cek limit switch\n", i + 1);
  //     while (true) { delay(1000); }
  //   }
  // }
  Serial.println("Semua joint homed, siap menerima perintah sudut.");

}

void loop() {

  processHostCommand();      // baca serial → ubah state

  switch (motion.mode) {
    case MotionMode::MoveJ:
      for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
        stepper[i].run();
      }
      if (allJointsCompleted()) {
        updatePositionCache();
        motion.mode = MotionMode::Idle;
        Serial.println(F("MOVEJ done"));
      }
      break;

    case MotionMode::Homing:
      // jalankan state machine homing (lihat fungsi homeJoint sebelumnya)
      // setelah selesai:
      // motion.homed = true; motion.mode = MotionMode::Idle;
      break;

    case MotionMode::EStop:
      // motor dimatikan → tunggu release
      break;

    case MotionMode::Idle:
      // optionally low-power / monitoring
      break;
  }
  
}


void Init_TMC2209_drivers(int num) { // Masih Di TMCStepper 

  driver[num].begin();
  driver[num].pdn_disable(true);       // gunakan UART, bukan PDN pin
  driver[num].mstep_reg_select(true);  // mikrostep diatur dari register, bukan pin MS1/MS2
  driver[num].I_scale_analog(false);   // referensi arus internal
  driver[num].rms_current(Joint[num].motor_max_current * 0.85, Joint[num].hold_multiplier);
  driver[num].en_spreadCycle(false);
  driver[num].toff(4);
  driver[num].blank_time(24);
  driver[num].pwm_autoscale(true);
  driver[num].microsteps(Joint[num].microstep);

  Joint[num].current = driver[num].rms_current();
}

void Init_Steppeer_parameters() { // Paramter Accel Stepper
  stepper[5].setMaxSpeed(50000);
  stepper[5].setAcceleration(100);
  stepper[5].setSpeed(0);

  stepper[0].setMaxSpeed(50000);
  stepper[0].setAcceleration(1000);
  stepper[0].setSpeed(0);

  stepper[1].setMaxSpeed(50000);
  stepper[1].setAcceleration(1000);
  stepper[1].setSpeed(0);

  stepper[2].setMaxSpeed(50000);
  stepper[2].setAcceleration(500);
  stepper[2].setSpeed(0);

  stepper[3].setMaxSpeed(50000);
  stepper[3].setAcceleration(500);
  stepper[3].setSpeed(0);

  stepper[4].setMaxSpeed(50000);
  stepper[4].setAcceleration(500);
  stepper[4].setSpeed(0);

}

void handleSerialAngleCommand() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.isEmpty()) return;

  uint8_t jointIndex = 0;              // default: joint 1
  float angleDeg = 0.0f;
  bool parsed = false;

  int sep = line.indexOf(' ');
  if (sep < 0) {
    // format sederhana: "30" → joint 1 ke +30°
    angleDeg = line.toFloat();
    parsed = true;
  } else {
    String first = line.substring(0, sep);
    String second = line.substring(sep + 1);
    first.trim();
    second.trim();

    if (first.startsWith("J") || first.startsWith("j")) {
      int jointNum = first.substring(1).toInt();
      if (jointNum >= 1 && jointNum <= NUMBER_OF_JOINTS) {
        jointIndex = static_cast<uint8_t>(jointNum - 1);
        angleDeg = second.toFloat();
        parsed = true;
      }
    } else {
      angleDeg = first.toFloat();
      parsed = true;
    }
  }

  if (!parsed) {
    Serial.println(F("Format salah. Gunakan '30' atau 'J2 -45'"));
    return;
  }
  
  jointAngleSetpoint[jointIndex] += angleDeg;
  long targetSteps = angleDegreesToSteps(jointIndex, angleDeg);
  stepper[jointIndex].moveTo(targetSteps);
  Serial.printf("Joint %u → %.2f° (%ld langkah)\n",
                jointIndex + 1, angleDeg, targetSteps);
}

// Menghitung berapa langkah per derajat untuk suatu joint (menggunakan reduction ratio dan microstepping)
float stepsPerDegree(uint8_t jointIndex) {
  float micro = Joint[jointIndex].microstep > 0
                  ? static_cast<float>(Joint[jointIndex].microstep)
                  : static_cast<float>(MICROSTEP);
  float gear = Joint[jointIndex].reduction_ratio > 0.0f
                  ? Joint[jointIndex].reduction_ratio
                  : 1.0f;
  const float motorStepsPerRev = 200.0f * micro;
  return (motorStepsPerRev * gear) / 360.0f;
}
// Conversi sudut derajat ke langkah
long angleDegreesToSteps(uint8_t jointIndex, float angleDeg) {
  return lround(angleDeg * stepsPerDegree(jointIndex));
}

bool homeJoint(uint8_t idx) {
  const uint8_t limitPin = Joint[idx].LIMIT;
  const int trigger = Joint[idx].limit_switch_trigger; // motor_init.cpp:31,73,115
  const bool invertedDir = Joint[idx].direction_reversed; // motor_init.cpp:41,83,124

  stepper[idx].setMaxSpeed(HOME_SPEED);
  stepper[idx].setAcceleration(HOME_ACCEL);

  // 1) Cari limit switch
  long startPos = stepper[idx].currentPosition();
  long searchTarget = invertedDir ? (startPos - SEARCH_EXTRA_STEPS)
                                  : (startPos + SEARCH_EXTRA_STEPS);
  stepper[idx].moveTo(searchTarget);

  while (digitalRead(limitPin) != trigger) {
    stepper[idx].run();
    //Serial.printf("ok\n");
    if (labs(stepper[idx].currentPosition() - startPos) >= SEARCH_EXTRA_STEPS) {
      Serial.printf("Joint %u homing gagal: limit tidak terdeteksi\n", idx + 1);
      stepper[idx].stop();
      stepper[idx].runToPosition();
      return false;
    }
  }
  stepper[idx].setSpeed(0);
  stepper[idx].setCurrentPosition(0);

  // 2) Mundur sedikit supaya switch lepas
  long backoff = invertedDir ? BACKOFF_STEPS : -BACKOFF_STEPS;
  stepper[idx].move(backoff);
  stepper[idx].runToPosition();
  Serial.printf("State 2\n");
  // 3) Bergerak ke posisi homed yang sudah didefinisikan (motor_init.cpp:15/58/99)
  long homedSteps = Joint[idx].homed_position;
  stepper[idx].setCurrentPosition(0);
  stepper[idx].moveTo(homedSteps);
  stepper[idx].runToPosition();
  Serial.printf("State 3\n");
  // 4) Set status & posisi logis
  stepper[idx].setCurrentPosition(homedSteps);
  Joint[idx].position = homedSteps;
  Joint[idx].homed = 1;

  Serial.printf("Joint %u homed @ %ld langkah\n", idx + 1, homedSteps);
  return true;
}

bool homeJoint2(uint8_t idx) {
  const uint8_t limitPin = Joint[idx].LIMIT;
  const int trigger      = Joint[idx].limit_switch_trigger;
  const bool invertedDir = Joint[idx].direction_reversed;

  stepper[idx].setMaxSpeed(HOME_SPEED);
  stepper[idx].setAcceleration(HOME_ACCEL);

  long startPos = stepper[idx].currentPosition();
  long searchTarget = invertedDir ? (startPos - SEARCH_EXTRA_STEPS)
                                  : (startPos + SEARCH_EXTRA_STEPS);
  stepper[idx].moveTo(searchTarget);

  while (digitalRead(limitPin) != trigger) {
    stepper[idx].run();
    if (labs(stepper[idx].currentPosition() - startPos) >= SEARCH_EXTRA_STEPS) {
      Serial.printf("Joint %u homing gagal: limit tidak terdeteksi\n", idx + 1);
      stepper[idx].stop();
      stepper[idx].runToPosition();
      return false;
    }
  }

  stepper[idx].stop();
  stepper[idx].runToPosition();
  stepper[idx].setSpeed(0);
  stepper[idx].setCurrentPosition(0);
  delay(2000);
  long backoff = invertedDir ? BACKOFF_STEPS : -BACKOFF_STEPS;
  Serial.printf("Backoff %ld steps\n", backoff);
  stepper[idx].runToNewPosition(backoff);
  stepper[idx].setCurrentPosition(0);
  // Set nol di titik referensi backoff dan berhenti di sini (tidak ke homed_position)

  Joint[idx].position = 0;
  Joint[idx].homed = 0; // belum homed lengkap

  long homed_pos = Joint[idx].homed_position;
  stepper[idx].runToNewPosition(homed_pos);
  stepper[idx].setCurrentPosition(homed_pos);
  
  Serial.printf("Debug stop: Joint %u di posisi %ld (%.2f deg)\n",
                idx + 1, stepper[idx].currentPosition(),
                stepper[idx].currentPosition() / stepsPerDegree(idx));
  return true;
}

//Kebawahnya code jalan utama
//Fungsi untuk memproses perintah dari host (serial)
void processHostCommand() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.isEmpty()) return;

  if (line.startsWith("MOVEJ")) {
    float q_target[NUMBER_OF_JOINTS] = {};
    float v_global = DEFAULT_MAX_SPEED;
    if (!motion.homed) {
      Serial.println(F("ERR: not homed"));
      return;
    }
    if (parseMoveJ(line, q_target, v_global)) {
      planMoveJ(q_target, v_global);
      motion.mode = MotionMode::MoveJ;
    } else {
      Serial.println(F("ERR: bad MOVEJ frame"));
    }
  } else if (line.equalsIgnoreCase("HOME ALL")) {
    handleHomeAll();
  } else if (line.equalsIgnoreCase("STOP")) {
    handleStop();
  } else if (line.equalsIgnoreCase("ESTOP")) {
    handleEStop(true);
  } else if (line.equalsIgnoreCase("RESET")) {
    handleEStop(false);  // release estop
  } else if (line.equalsIgnoreCase("STATUS?")) {
    publishStatus();
  } else {
    Serial.println(F("ERR: unknown command"));
  }
}

//membaca string perintah yang datang lewat serial
bool parseMoveJ(const String& line, float (&q_target)[NUMBER_OF_JOINTS], float& v_global) {
  // Contoh frame: MOVEJ 30.0 10.0 -40.0 0 20.0 5.0 V=60
  int firstSpace = line.indexOf(' ');
  if (firstSpace < 0) return false;

  String payload = line.substring(firstSpace + 1);
  payload.trim();
  if (payload.isEmpty()) return false;

  int idx = 0;
  char* endPtr = nullptr;
  char buffer[128];
  payload.toCharArray(buffer, sizeof(buffer));

  char* token = strtok(buffer, " ");
  while (token && idx < NUMBER_OF_JOINTS) {
    if (toupper(token[0]) == 'V' && token[1] == '=') break;
    q_target[idx++] = atof(token);
    token = strtok(nullptr, " ");
  }
  if (idx != NUMBER_OF_JOINTS) return false;

  while (token) {
    if ((toupper(token[0]) == 'V') && token[1] == '=') {
      v_global = atof(token + 2);
      break;
    }
    token = strtok(nullptr, " ");
  }
  return true;
}

// Hitung selisih sudut/steps tiap joint terhadap posisi sekarang.
//Konversi ke langkah (delta_steps).
//Cari T_total supaya semua joint selesai bersamaan berdasarkan v_global.
//Dari T_total hitung maxSpeedSteps[i], set stepper[i].setMaxSpeed(...), setAcceleration(...), dan panggil stepper[i].moveTo(target_steps[i]).
void planMoveJ(const float (&q_target)[NUMBER_OF_JOINTS], float v_global_deg) {
  updatePositionCache(); // isi q_now_deg & steps_now dari AccelStepper

  float stepsPerDeg[NUMBER_OF_JOINTS];
  float v_steps[NUMBER_OF_JOINTS];
  float t_i[NUMBER_OF_JOINTS];
  float max_t = 0.0f;

  for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
    stepsPerDeg[i] = stepsPerDegree(i);
    motion.delta_deg[i] = q_target[i] - motion.q_now_deg[i];
    motion.delta_steps[i] = motion.delta_deg[i] * stepsPerDeg[i];
    motion.target_steps[i] = motion.steps_now[i] + motion.delta_steps[i];
    v_steps[i] = abs(v_global_deg) * stepsPerDeg[i]; // langkah/s
    if (v_steps[i] < 1.0f) v_steps[i] = 1.0f; // jaga jangan nol
    t_i[i] = abs(motion.delta_steps[i]) / v_steps[i];
    if (t_i[i] > max_t) max_t = t_i[i];
  }
  if (max_t < 0.001f) max_t = 0.001f;

  for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
    motion.maxSpeedSteps[i] = abs(motion.delta_steps[i]) / max_t;
    if (motion.maxSpeedSteps[i] < 1.0f) motion.maxSpeedSteps[i] = 1.0f;
    stepper[i].setMaxSpeed(motion.maxSpeedSteps[i]);
    stepper[i].setAcceleration(DEFAULT_ACCEL);
    stepper[i].moveTo(motion.target_steps[i]);
  }

  Serial.print(F("MOVEJ plan: T_total="));
  Serial.println(max_t, 3);
}

void updatePositionCache() {
  for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
    motion.steps_now[i] = stepper[i].currentPosition();
    motion.q_now_deg[i] = motion.steps_now[i] / stepsPerDegree(i);
  }
}

bool allJointsCompleted() {
  for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
    if (stepper[i].distanceToGo() != 0) return false;
  }
  return true;
}

void publishStatus() {
  updatePositionCache();
  Serial.print(F("STATUS mode="));
  Serial.print(static_cast<int>(motion.mode));
  Serial.print(F(" estop="));
  Serial.print(motion.estop);
  Serial.print(F(" homed="));
  Serial.println(motion.homed);
  Serial.print(F("pos(deg): "));
  for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
    Serial.print(motion.q_now_deg[i], 2);
    Serial.print(i == NUMBER_OF_JOINTS - 1 ? '\n' : ',');
  }
}

void handleHomeAll() {
  if (motion.estop) {
    Serial.println(F("Cannot home: ESTOP latched"));
    return;
  }
  motion.mode = MotionMode::Homing;
  // jalankan state machine homing per joint (lihat fungsi homeJoint() sebelumnya)
  // setelah selesai:
  motion.homed = true;
  motion.mode = MotionMode::Idle;
  updatePositionCache();
  Serial.println(F("HOME ALL done"));
}

void handleStop() {
  if (motion.mode == MotionMode::MoveJ) {
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
      stepper[i].stop();          // ramp berhenti
    }
    motion.mode = MotionMode::Idle;
    updatePositionCache();
    Serial.println(F("STOP executed"));
  }
}

void handleEStop(bool latch) {
  if (latch) {
    motion.estop = true;
    motion.mode = MotionMode::EStop;
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
      stepper[i].stop();
    }
    // Matikan ENABLE driver (mis: digitalWrite(GLOBAL_ENABLE, HIGH));
    Serial.println(F("ESTOP latched"));
  } else {
    // Lepas ESTOP setelah user yakin aman
    motion.estop = false;
    motion.mode = MotionMode::Idle;
    // Nyalakan lagi driver (GLOBAL_ENABLE LOW)
    Serial.println(F("ESTOP cleared"));
  }
}

bool homeJointManual(uint8_t idx) {
  const uint8_t limitPin = Joint[idx].LIMIT;
  const int trigger      = Joint[idx].limit_switch_trigger;  // 0 jika INPUT_PULLUP
  const bool invertedDir = Joint[idx].direction_reversed;

  // Kecepatan homing
  stepper[idx].setMaxSpeed(800.0f);
  stepper[idx].setAcceleration(1500.0f);

  // 1) Cari limit switch
  long startPos = stepper[idx].currentPosition();
  long searchTarget = invertedDir ? (startPos - SEARCH_EXTRA_STEPS)
                                  : (startPos + SEARCH_EXTRA_STEPS);
  stepper[idx].moveTo(searchTarget);

  while (digitalRead(limitPin) != trigger) {
    stepper[idx].run();
    if (labs(stepper[idx].currentPosition() - startPos) >= SEARCH_EXTRA_STEPS) {
      Serial.printf("Joint %u homing gagal: limit tidak terdeteksi\n", idx + 1);
      stepper[idx].stop();
      stepper[idx].runToPosition();
      return false;
    }
  }

  // 2) Stop & lepas sensor
  stepper[idx].stop();
  stepper[idx].runToPosition();
  stepper[idx].setSpeed(0);
  stepper[idx].setCurrentPosition(0);
  delay(3000);
  long backoff = (searchTarget > startPos) ? -BACKOFF_STEPS : BACKOFF_STEPS;
  Serial.printf("Backoff %ld steps\n", backoff);
  stepper[idx].runToNewPosition(backoff);

  // Set nol di posisi referensi ini
  stepper[idx].setCurrentPosition(0);
  Joint[idx].position = 0;
  
  long homed_pos = Joint[idx].homed_position;
  stepper[idx].runToNewPosition(homed_pos);
  
  long offsetAccum = 0;
  Serial.printf("Joint %u: ketik offset (steps atau 'Xdeg'), ketik 'ok' untuk selesai\n", idx + 1);

  while (true) {
    while (!Serial.available()) { delay(10); }
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    if (line.equalsIgnoreCase("ok")) {
      break;  // keluar loop
    }

    long deltaSteps = 0;
    if (line.endsWith("deg") || line.endsWith("DEG")) {
      line.replace("deg", ""); line.replace("DEG", ""); line.trim();
      float deg = line.toFloat();
      deltaSteps = lround(deg * stepsPerDegree(idx));
    } else {
      deltaSteps = line.toInt();
    }

    offsetAccum += deltaSteps;
    Serial.printf("Jalankan %ld steps (total=%ld)\n", deltaSteps, offsetAccum);
    stepper[idx].runToNewPosition(stepper[idx].currentPosition() + deltaSteps);
    stepper[idx].setCurrentPosition(stepper[idx].currentPosition()); // sinkronkan counter
  }

  // setelah loop, set posisi logis akhir
  stepper[idx].setCurrentPosition(offsetAccum);
  Joint[idx].position = offsetAccum;
  Joint[idx].homed = 1;
  Serial.printf("Joint %u selesai, total = %ld steps (%.2f deg)\n",
                idx + 1, offsetAccum, offsetAccum / stepsPerDegree(idx));
}


