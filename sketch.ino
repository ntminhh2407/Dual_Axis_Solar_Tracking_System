#include <ESP32Servo.h>
#include <Fuzzy.h>

// ---------------------------------------------------------
// 1. CẤU HÌNH PHẦN CỨNG
// ---------------------------------------------------------
// Định nghĩa các chân cho 4 con LDR
const int LDR_TL = 32; // Trên - Trái (Top-Left)
const int LDR_TR = 33; // Trên - Phải (Top-Right)
const int LDR_BL = 34; // Dưới - Trái (Bottom-Left)
const int LDR_BR = 35; // Dưới - Phải (Bottom-Right)

// Định nghĩa chân cho 2 Servo
const int SERVO_PAN_PIN = 12;  // Trục Ngang (Đế) - Dùng MG996R
const int SERVO_TILT_PIN = 13; // Trục Dọc (Gật) - Dùng MG90S

Servo servoPan;
Servo servoTilt;

// Các biến lưu góc hiện tại của Servo
float currentPanPos = 90.0;
float currentTiltPos = 90.0;

// Giới hạn góc quay để bảo vệ dây điện và cơ khí
const int PAN_MIN = 0;
const int PAN_MAX = 180;
const int TILT_MIN = 20;  // Không cho gập quá sâu
const int TILT_MAX = 160; // Không cho ngửa quá đà

// ---------------------------------------------------------
// 2. KHỞI TẠO BỘ ĐIỀU KHIỂN MỜ (FUZZY LOGIC)
// ---------------------------------------------------------
// Chúng ta cần 2 bộ xử lý mờ riêng biệt cho Trục Ngang và Trục Dọc
Fuzzy *fuzzyPan = new Fuzzy();
Fuzzy *fuzzyTilt = new Fuzzy();

// Hàm cài đặt Luật Mờ (Chạy trong setup)
void setupFuzzyLogic(Fuzzy *fuzzySystem) {
  // --- INPUT: Sai số ánh sáng (Error) ---
  // Sai số nằm trong khoảng -4095 đến 4095 (Do ESP32 đọc ADC 12-bit)
  FuzzyInput *errorInput = new FuzzyInput(1);

  // Tạo 5 Tập mờ (Membership Functions) cho đầu vào
  // Thông số (a, b, c, d) đại diện cho tọa độ các đỉnh của hình thang/tam giác
  FuzzySet *NB = new FuzzySet(-4095, -4095, -2000, -500); // Âm lớn (Lệch trái/dưới nhiều)
  FuzzySet *NS = new FuzzySet(-2000, -500, -500, -50);    // Âm nhỏ (Lệch trái/dưới ít)
  FuzzySet *ZE = new FuzzySet(-150, 0, 0, 150);           // Cân bằng (Deadzone)
  FuzzySet *PS = new FuzzySet(50, 500, 500, 2000);        // Dương nhỏ (Lệch phải/trên ít)
  FuzzySet *PB = new FuzzySet(500, 2000, 4095, 4095);     // Dương lớn (Lệch phải/trên nhiều)

  errorInput->addFuzzySet(NB);
  errorInput->addFuzzySet(NS);
  errorInput->addFuzzySet(ZE);
  errorInput->addFuzzySet(PS);
  errorInput->addFuzzySet(PB);
  fuzzySystem->addFuzzyInput(errorInput);

  // --- OUTPUT: Bước nhảy góc (Delta Angle) ---
  // Bước nhảy từ -5 độ đến +5 độ mỗi chu kỳ
  FuzzyOutput *deltaAngle = new FuzzyOutput(1);

  FuzzySet *MoveFastNeg = new FuzzySet(-5, -5, -4, -2); // Lùi nhanh (-5 độ)
  FuzzySet *MoveSlowNeg = new FuzzySet(-3, -1, -1, 0);  // Lùi chậm (-1 độ)
  FuzzySet *Stop        = new FuzzySet(-0.5, 0, 0, 0.5);// Đứng yên (0 độ)
  FuzzySet *MoveSlowPos = new FuzzySet(0, 1, 1, 3);     // Tiến chậm (+1 độ)
  FuzzySet *MoveFastPos = new FuzzySet(2, 4, 5, 5);     // Tiến nhanh (+5 độ)

  deltaAngle->addFuzzySet(MoveFastNeg);
  deltaAngle->addFuzzySet(MoveSlowNeg);
  deltaAngle->addFuzzySet(Stop);
  deltaAngle->addFuzzySet(MoveSlowPos);
  deltaAngle->addFuzzySet(MoveFastPos);
  fuzzySystem->addFuzzyOutput(deltaAngle);

  // --- HỆ LUẬT MỜ (RULE BASE) ---
  // Luật 1: NẾU Error là NB THÌ quay Lùi Nhanh
  FuzzyRuleAntecedent *ifNB = new FuzzyRuleAntecedent();
  ifNB->joinSingle(NB);
  FuzzyRuleConsequent *thenMFN = new FuzzyRuleConsequent();
  thenMFN->addOutput(MoveFastNeg);
  fuzzySystem->addFuzzyRule(new FuzzyRule(1, ifNB, thenMFN));

  // Luật 2: NẾU Error là NS THÌ quay Lùi Chậm
  FuzzyRuleAntecedent *ifNS = new FuzzyRuleAntecedent();
  ifNS->joinSingle(NS);
  FuzzyRuleConsequent *thenMSN = new FuzzyRuleConsequent();
  thenMSN->addOutput(MoveSlowNeg);
  fuzzySystem->addFuzzyRule(new FuzzyRule(2, ifNS, thenMSN));

  // Luật 3: NẾU Error là ZE THÌ Đứng yên
  FuzzyRuleAntecedent *ifZE = new FuzzyRuleAntecedent();
  ifZE->joinSingle(ZE);
  FuzzyRuleConsequent *thenStop = new FuzzyRuleConsequent();
  thenStop->addOutput(Stop);
  fuzzySystem->addFuzzyRule(new FuzzyRule(3, ifZE, thenStop));

  // Luật 4: NẾU Error là PS THÌ quay Tiến Chậm
  FuzzyRuleAntecedent *ifPS = new FuzzyRuleAntecedent();
  ifPS->joinSingle(PS);
  FuzzyRuleConsequent *thenMSP = new FuzzyRuleConsequent();
  thenMSP->addOutput(MoveSlowPos);
  fuzzySystem->addFuzzyRule(new FuzzyRule(4, ifPS, thenMSP));

  // Luật 5: NẾU Error là PB THÌ quay Tiến Nhanh
  FuzzyRuleAntecedent *ifPB = new FuzzyRuleAntecedent();
  ifPB->joinSingle(PB);
  FuzzyRuleConsequent *thenMFP = new FuzzyRuleConsequent();
  thenMFP->addOutput(MoveFastPos);
  fuzzySystem->addFuzzyRule(new FuzzyRule(5, ifPB, thenMFP));
}

// ---------------------------------------------------------
// 3. HÀM SETUP
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Cấu hình PWM cho Servo trên ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);

  // Gắn chân Servo
  servoPan.attach(SERVO_PAN_PIN, 500, 2400);
  servoTilt.attach(SERVO_TILT_PIN, 500, 2400);

  // Đưa Servo về vị trí 90 độ (Home)
  servoPan.write(currentPanPos);
  servoTilt.write(currentTiltPos);
  delay(1000); // Chờ Servo về đúng vị trí

  // Khởi tạo Bộ điều khiển mờ cho 2 trục
  setupFuzzyLogic(fuzzyPan);
  setupFuzzyLogic(fuzzyTilt);
  
  Serial.println("Fuzzy Logic Solar Tracker Started!");
}

// ---------------------------------------------------------
// 4. VÒNG LẶP CHÍNH (LOOP)
// ---------------------------------------------------------
void loop() {
  // 1. Đọc giá trị từ 4 LDR (0 - 4095)
  int tl = analogRead(LDR_TL);
  int tr = analogRead(LDR_TR);
  int bl = analogRead(LDR_BL);
  int br = analogRead(LDR_BR);

  // 2. Tính toán sai số (Error) cho 2 trục
  // Trục Ngang (Pan): Trái trừ Phải
  int errorPan = (tl + bl) - (tr + br); 
  // Trục Dọc (Tilt): Trên trừ Dưới
  int errorTilt = (tl + tr) - (bl + br);

  // --- XỬ LÝ MỜ CHO TRỤC NGANG ---
  fuzzyPan->setInput(1, errorPan);
  fuzzyPan->fuzzify();
  float deltaPan = fuzzyPan->defuzzify(1); // Xuất ra góc nhích (-5 đến 5)

  // --- XỬ LÝ MỜ CHO TRỤC DỌC ---
  fuzzyTilt->setInput(1, errorTilt);
  fuzzyTilt->fuzzify();
  float deltaTilt = fuzzyTilt->defuzzify(1); // Xuất ra góc nhích (-5 đến 5)

  // 3. Cập nhật góc mới
  currentPanPos += deltaPan;
  currentTiltPos += deltaTilt;

  // 4. Giới hạn góc để bảo vệ cơ khí
  currentPanPos = constrain(currentPanPos, PAN_MIN, PAN_MAX);
  currentTiltPos = constrain(currentTiltPos, TILT_MIN, TILT_MAX);

  // 5. Ra lệnh cho Servo quay
  servoPan.write(currentPanPos);
  servoTilt.write(currentTiltPos);

  // 6. In ra Serial Monitor để theo dõi và tinh chỉnh
  Serial.print("E_Pan: "); Serial.print(errorPan);
  Serial.print(" | dPan: "); Serial.print(deltaPan);
  Serial.print(" | E_Tilt: "); Serial.print(errorTilt);
  Serial.print(" | dTilt: "); Serial.print(deltaTilt);
  Serial.println();

  // Tốc độ lấy mẫu (100ms/lần giúp hệ thống mượt mà)
  delay(100); 
}