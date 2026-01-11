#include <WiFi.h>          // Thư viện hỗ trợ các tính năng Wi-Fi trên ESP32
#include <esp_now.h>       // Thư viện giao thức ESP-NOW để truyền dữ liệu không dây tốc độ cao
#include <ESP32Servo.h>    // Thư viện điều khiển Servo dành riêng cho dòng ESP32

// ---------- CẤU HÌNH HỆ THỐNG CẢM BIẾN ----------
#define NUM_SENSORS 5      // Tổng số lượng cảm biến siêu âm được gắn trên xe
// Mảng chứa các chân phát tín hiệu (Trig) cho 5 cảm biến
const int trigPins[NUM_SENSORS] = {13, 11, 9, 8, 6}; 
// Mảng chứa các chân nhận tín hiệu phản hồi (Echo) cho 5 cảm biến
const int echoPins[NUM_SENSORS] = {14, 12, 10, 7, 5}; 
int distances[NUM_SENSORS]; // Mảng lưu trữ khoảng cách đo được (đơn vị cm) của 5 cảm biến

// ---------- CẤU HÌNH CHÂN ĐIỀU KHIỂN (ESP32-S3) ----------
#define RPWM 36      // Chân phát xung PWM để xe chạy TIẾN (nối vào mạch cầu H)
#define LPWM 35      // Chân phát xung PWM để xe chạy LÙI (nối vào mạch cầu H)
#define SERVO_PIN 37 // Chân gửi tín hiệu điều khiển góc quay cho Servo lái
#define COI_PIN 48   // Chân điều khiển còi báo hiệu (Buzzer) gắn trên mạch

// ---------- THÔNG SỐ LÁI & VI SAI ----------
#define SERVO_CENTER 90  // Góc Servo khi bánh xe ở vị trí thẳng tuyệt đối
#define SERVO_LEFT 60    // Giới hạn góc bẻ lái tối đa sang bên TRÁI
#define SERVO_RIGHT 120  // Giới hạn góc bẻ lái tối đa sang bên PHẢI

// ---------- CÁC NGƯỠNG KHOẢNG CÁCH TỐI ƯU ----------
#define SAFE_DISTANCE 60    // Khoảng cách xe bắt đầu nhận biết vật cản để chuẩn bị cua
#define TURN_THRESHOLD 30   // Ngưỡng khoảng cách bắt đầu thực hiện bẻ lái gắt
#define DANGER_DISTANCE 15  // Ngưỡng quá gần, xe phải dừng khẩn cấp và lùi lại
// Kp (Hệ số tỉ lệ): Phản ứng dựa trên lỗi hiện tại giữa 2 tường
float Kp = 1.4;             
// Kd (Hệ số vi phân): Dự đoán và ngăn chặn xe lao vào tường quá nhanh (chống đảo lái)
float Kd = 0.9;             
int lastSideError = 0;      // Lưu lại độ lệch của vòng lặp trước để tính toán vi phân (D)

Servo steering;                // Tạo đối tượng 'steering' để điều khiển thiết bị Servo
float currentSteerAngle = SERVO_CENTER; // Biến lưu góc lái hiện tại, dùng để nội suy mượt mà
bool lastChedo = true;         // Lưu trạng thái chế độ vòng trước để phát hiện lúc gạt công tắc

// ---------- CẤU TRÚC GÓI TIN ESP-NOW ----------
struct struct_message {
    uint8_t speed;      // Tín hiệu ga từ cò súng tay cầm (0-255)
    uint8_t servo;      // Tín hiệu hướng lái từ bánh xe tay cầm (0-180)
    uint8_t trimspeed;  // Giới hạn tốc độ tối đa thiết lập trên tay cầm
    uint8_t trimservo;  // Cân chỉnh độ thẳng cho xe từ tay cầm
    bool ketnoi : 1;    // Trạng thái kết nối (Bit-field 1 bit)
    bool chedo : 1;     // Chế độ (true: Lái tay, false: Tự hành)
} controlData;

unsigned long lastRecvTime = 0;      // Lưu mốc thời gian cuối cùng nhận được gói tin
const unsigned long TIMEOUT_MS = 1000; // Thời gian tối đa (1s) nếu mất sóng sẽ dừng xe

// --- Hàm Callback: Chạy tự động mỗi khi xe nhận được dữ liệu từ tay cầm ---
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    // Sao chép vùng nhớ dữ liệu nhận được vào biến cấu trúc controlData
    memcpy(&controlData, incomingData, sizeof(controlData));
    lastRecvTime = millis(); // Cập nhật thời gian nhận mới nhất
}

void setup() {
    Serial.begin(115200);   // Khởi tạo cổng giao tiếp máy tính để debug
    WiFi.mode(WIFI_STA);    // Thiết lập Wi-Fi ở chế độ Station
    WiFi.channel(1);        // Cố định kênh Wi-Fi là 1 để trùng với bộ phát
    WiFi.setTxPower(WIFI_POWER_19_5dBm); // Thiết lập công suất phát sóng cao nhất

    // Thiết lập các chân điều khiển động cơ và còi là đầu ra (OUTPUT)
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(COI_PIN, OUTPUT);
    digitalWrite(COI_PIN, LOW); // Đảm bảo còi tắt khi mới khởi động

    // Cấu hình điều khiển Servo: Sử dụng Timer 0, tần số 50Hz, dải xung 500-2400us
    ESP32PWM::allocateTimer(0);
    steering.setPeriodHertz(50);
    steering.attach(SERVO_PIN, 500, 2400);
    steering.write(SERVO_CENTER); // Đưa bánh xe về vị trí thẳng

    // Cấu hình chân cho 5 cảm biến siêu âm
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(trigPins[i], OUTPUT); // Chân Trig là đầu ra để phát sóng
        pinMode(echoPins[i], INPUT);  // Chân Echo là đầu vào để nhận sóng về
    }

    // Khởi tạo ESP-NOW, nếu lỗi sẽ dừng chương trình
    if (esp_now_init() != ESP_OK) return;
    // Đăng ký hàm OnDataRecv để hệ thống gọi khi có gói tin đến
    esp_now_register_recv_cb(OnDataRecv);

    stopMotor(); // Đảm bảo xe đứng yên khi mới bật điện
}

void loop() {
    // 1. Cập nhật khoảng cách từ 5 mắt cảm biến liên tục mỗi vòng lặp
    readAllUltrasonics(); 

    // 2. Hệ thống Failsafe: Nếu mất tín hiệu quá 1 giây thì dừng xe ngay lập tức
    if (millis() - lastRecvTime > TIMEOUT_MS) {
        stopMotor();
        return; // Thoát vòng lặp hiện tại để đảm bảo an toàn
    }

    // 3. Kiểm tra sự kiện chuyển đổi chế độ (Lái tay <-> Tự hành)
    if (controlData.chedo != lastChedo) {
        stopMotor(); // Dừng xe để chuyển đổi trạng thái
        if (controlData.chedo == false) { // Nếu vừa gạt sang chế độ AUTO
            digitalWrite(COI_PIN, HIGH); delay(200); digitalWrite(COI_PIN, LOW); // Kêu còi báo hiệu
        }
        lastChedo = controlData.chedo; // Cập nhật trạng thái chế độ mới
    }

    // 4. Quyết định chạy hàm xử lý tương ứng với chế độ đang chọn
    if (controlData.chedo == false) {
        navigateAuto(); // Chạy thuật toán tự hành thi đấu
    } else {
        handleManualControl(); // Chạy theo lệnh điều khiển từ tay cầm
    }
    
    delay(5); // Nghỉ 5ms để vòng lặp chạy ổn định (tần số ~200Hz)
}

// ---------- HÀM ĐIỀU KHIỂN TAY  ----------
void handleManualControl() {
    int speedIn = controlData.speed;     // Lấy giá trị ga (0-255)
    int limit = controlData.trimspeed;   // Lấy giá trị giới hạn tốc độ (trim)

    int center = controlData.trimservo;  // Lấy giá trị cân bằng lái
    int steerIn = controlData.servo;     // Lấy hướng lái từ tay cầm
    float targetAngle = center;          // Mặc định hướng lái là thẳng

    // Chuyển đổi dữ liệu tay cầm sang góc quay của Servo
    if (steerIn > 100) targetAngle = map(steerIn, 100, 180, center, center + 40);
    else if (steerIn < 80) targetAngle = map(steerIn, 80, 0, center, center - 40);
    
    // --- LOGIC HỖ TRỢ VI SAI: GIẢM GA KHI CUA ---
    // Tính toán độ gắt của góc lái hiện tại
    float steerFactor = abs(targetAngle - center) / 45.0; 
    // Giảm ga tỉ lệ thuận với góc cua để bộ vi sai không bị trượt bánh (giảm tối đa 40%)
    int adjustedLimit = limit * (1.0 - (steerFactor * 0.4)); 

    // Xử lý lệnh TIẾN/LÙI/DỪNG từ cò súng
    if (speedIn > 70 && speedIn < 150) { // Kéo cò: TIẾN
        uint8_t val = map(speedIn, 70, 150, 0, adjustedLimit);
        driveForward(val); 
    } 
    else if (speedIn < 35) { // Đẩy cò: LÙI
        driveBackward(map(speedIn, 35, 0, 0, 130));
    } 
    else stopMotor(); // Buông tay: DỪNG

    // Thuật toán làm mượt lái: Góc hiện tại tiến dần tới góc mục tiêu với tốc độ 35%
    currentSteerAngle += (targetAngle - currentSteerAngle) * 0.35;
    // Xuất tín hiệu điều khiển Servo sau khi giới hạn trong dải an toàn
    steering.write(constrain(currentSteerAngle, SERVO_LEFT, SERVO_RIGHT));
}

// ---------- HÀM TỰ HÀNH THI ĐẤU CHO XE ----------
void navigateAuto() {
    // Đặt tên gợi nhớ cho dữ liệu từ 5 cảm biến
    int fL = distances[0]; int fC = distances[1]; int fR = distances[2]; 
    int sL = distances[3]; int sR = distances[4]; 

    int maxSpeed = controlData.trimspeed; // Lấy tốc độ tối đa cho phép
    float targetAngle = controlData.trimservo; // Mặc định là góc lái thẳng

    // 1. XỬ LÝ KHẨN CẤP (FAILSAFE): Khi xe bị kẹt hoặc quá sát vật cản
    if (fC < DANGER_DISTANCE || fL < 8 || fR < 8) {
        stopMotor();           // Dừng xe ngay lập tức
        driveBackward(115);    // Chạy lùi chậm
        // Lùi về hướng có khoảng trống lớn hơn dựa trên cảm biến hông
        if (sL > sR) steering.write(SERVO_LEFT); else steering.write(SERVO_RIGHT);
        delay(400);            // Lùi trong 0.4 giây
        return;                // Thoát hàm để quét lại cảm biến mới
    }

    // --- 2. THUẬT TOÁN PD GIỮ XE LUÔN Ở GIỮA ĐƯỜNG ---
    int currentSideError = sL - sR; // Tính độ lệch giữa tường bên trái và tường bên phải
    int derivative = currentSideError - lastSideError; // Tính tốc độ xe đang lao vào tường
    // Tính toán góc lái bù: P (lỗi hiện tại) + D (dự đoán xu hướng)
    targetAngle += (currentSideError * Kp) + (derivative * Kd);
    lastSideError = currentSideError; // Lưu lỗi lại cho vòng lặp kế tiếp

    // 3. XỬ LÝ ĐIỀU HƯỚNG VÀ TỐC ĐỘ THEO SA BÀN
    int baseSpeed = 120; // Tốc độ cơ bản khi chạy tự hành

    if (fC < SAFE_DISTANCE || fL < 35 || fR < 35) {
        // PHÁT HIỆN CUA: Giảm tốc độ dựa trên khoảng cách tới vật cản
        baseSpeed = map(fC, DANGER_DISTANCE, SAFE_DISTANCE, 75, 120);
        
        // Quyết định bẻ lái gắt về hướng có tổng khoảng trống (trước + hông) lớn nhất
        if (fL + sL > fR + sR) targetAngle -= map(fC, 0, SAFE_DISTANCE, 45, 10);
        else targetAngle += map(fC, 0, SAFE_DISTANCE, 45, 10);
    } 
    else if (fC > 180) { // ĐƯỜNG THẲNG: Tận dụng vi sai để bứt tốc tối đa (Turbo)
        baseSpeed = maxSpeed; 
    }

    // --- 4. KIỂM SOÁT ỔN ĐỊNH: TỰ GIẢM GA KHI ĐANG CUA GẮT ---
    float turnIntensity = abs(targetAngle - controlData.trimservo); // Độ gắt của hướng lái
    if (turnIntensity > 10) {
        // Giảm ga tỉ lệ thuận với độ gắt để vi sai phân bổ lực đều, không bị quay bánh ảo
        baseSpeed = baseSpeed * (1.0 - (turnIntensity / 100.0));
    }

    // Ra lệnh chạy xe tiến tới sau khi giới hạn tốc độ an toàn
    driveForward(constrain(baseSpeed, 65, maxSpeed));
    
    // Làm mượt góc lái khi tự hành (hệ số 0.28 giúp xe chắc lái ở tốc độ cao)
    currentSteerAngle += (targetAngle - currentSteerAngle) * 0.28;
    steering.write(constrain(currentSteerAngle, SERVO_LEFT, SERVO_RIGHT));
}

// --- Hàm phụ trợ: Đo khoảng cách siêu âm ---
int getDistance(int trig, int echo) {
    digitalWrite(trig, LOW);        // Đảm bảo chân Trig đang thấp
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);       // Phát xung siêu âm trong 10us
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    
    // Đo thời gian chờ tín hiệu phản hồi (Timeout 20ms tương đương ~3.4 mét)
    long duration = pulseIn(echo, HIGH, 20000); 
    if (duration == 0) return 300;  // Nếu không thấy vật cản, mặc định là 300cm
    int dist = duration * 0.034 / 2; // Tính toán khoảng cách theo tốc độ âm thanh
    return (dist > 300) ? 300 : dist; // Giới hạn giá trị trả về tối đa 3 mét
}

// --- Hàm phụ trợ: Đọc dữ liệu từ cả 5 mắt cảm biến ---
void readAllUltrasonics() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        distances[i] = getDistance(trigPins[i], echoPins[i]);
    }
}

// --- Hàm điều khiển xe chạy tiến ---
void driveForward(int speed) {
    analogWrite(RPWM, speed); // Cấp xung PWM vào chân tiến
    analogWrite(LPWM, 0);     // Ngắt chân lùi
}

// --- Hàm điều khiển xe chạy lùi ---
void driveBackward(int speed) {
    analogWrite(RPWM, 0);     // Ngắt chân tiến
    analogWrite(LPWM, speed); // Cấp xung PWM vào chân lùi
}

// --- Hàm dừng xe hoàn toàn ---
void stopMotor() {
    analogWrite(RPWM, 0); // Ngắt cả 2 xung điều khiển
    analogWrite(LPWM, 0);
}
