#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Chân ADC
#define speedPotPin  17
#define servoPotPin  15
#define speedtrimPin 16
#define servotrimPin 18 
#define congtac      39

// OLED config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_SDA 5
#define OLED_SCL 4

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Địa chỉ MAC của ESP32 nhận
uint8_t receiverMacAddress[] = {0x30, 0xED, 0xA0, 0xA3, 0x11, 0x90};//30:ed:a0:a3:11:90

// Cấu trúc dữ liệu
struct struct_message {
    uint8_t speed;      // 0-255
    uint8_t servo;      // 0-180
    uint8_t trimspeed;  // 50-255
    uint8_t trimservo;  // 0-180
    bool ketnoi : 1 ; 
    bool chedo : 1;
} controlData;

// Bộ lọc trung bình trượt
#define WINDOW_SIZE 5
int speedBuffer[WINDOW_SIZE] = {0}, servoBuffer[WINDOW_SIZE] = {0};
int speedtrimBuffer[WINDOW_SIZE] = {0}, servotrimBuffer[WINDOW_SIZE] = {0};
int bufferIndex = 0;

// Timer gửi dữ liệu
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 10; // Gửi mỗi 10ms (~100 Hz)

// Callback gửi dữ liệu
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Trạng thái gửi: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Thành công" : "Thất bại");
}

// Đọc ADC ổn định
int readADCStable(int pin) {
    const int numReadings = 10;
    long sum = 0;
    for (int i = 0; i < numReadings; i++) {
        sum += analogRead(pin);
        delayMicroseconds(100);
    }
    return sum / numReadings;
}

// Bộ lọc trung bình trượt
int movingAverage(int newValue, int* buffer) {
    buffer[bufferIndex] = newValue;
    bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
    long sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += buffer[i];
    }
    return sum / WINDOW_SIZE;
}

// Gửi dữ liệu với retry
void sendData() {
    const int maxRetries = 3;
    esp_err_t result;
    for (int i = 0; i < maxRetries; i++) {
        result = esp_now_send(receiverMacAddress, (uint8_t *)&controlData, sizeof(controlData));
        if (result == ESP_OK) {
          controlData.ketnoi = true ;
            break;
        }
        Serial.println("Thử lại gửi dữ liệu...");
        delay(10);
    }
    if (result != ESP_OK) {
        controlData.ketnoi = false;
        Serial.println("Lỗi gửi dữ liệu sau nhiều lần thử");
    }
}

void setup() {
    pinMode ( congtac , INPUT_PULLUP);
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.channel(1); // Cố định kênh Wi-Fi
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    esp_wifi_set_max_tx_power(8); // Giảm công suất truyền

    Serial.println("MAC ESP32: " + WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("Lỗi khởi tạo ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
    peerInfo.channel = 1; // Phải khớp với WiFi.channel()
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Lỗi thêm peer");
        return;
    }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("OLED & ESP-NOW OK");
  display.display();
  delay(1000);
}

void loop() {
    unsigned long currentTime = millis();
    if (currentTime - lastSendTime >= sendInterval) {
        // Đọc ADC
        int speedRaw = readADCStable(speedPotPin);
        int servoRaw = readADCStable(servoPotPin);
        int speedtrimRaw = readADCStable(speedtrimPin);
        int servotrimRaw = readADCStable(servotrimPin);

        // Lọc trung bình trượt
        int speedFiltered = movingAverage(speedRaw, speedBuffer);
        int servoFiltered = movingAverage(servoRaw, servoBuffer);
        int speedtrimFiltered = movingAverage(speedtrimRaw, speedtrimBuffer);
        int servotrimFiltered = movingAverage(servotrimRaw, servotrimBuffer);

        // Giới hạn giá trị
        speedFiltered = constrain(speedFiltered, 0, 4095);
        servoFiltered = constrain(servoFiltered, 0, 4095);
        speedtrimFiltered = constrain(speedtrimFiltered, 0, 4095);
        servotrimFiltered = constrain(servotrimFiltered, 0, 4095);

        // Ánh xạ
        controlData.speed = map(speedFiltered, 0, 2905, 0, 255);
        controlData.servo = map(servoFiltered, 240, 2820, 0, 180);
        controlData.trimspeed = map(speedtrimFiltered, 0, 2900, 50, 255);
        controlData.trimservo = map(servotrimFiltered, 0, 3410, 0, 180);
        controlData.chedo = digitalRead (congtac);

        // Debug
        Serial.print("Tốc độ (raw): "); Serial.print(speedRaw);
        Serial.print(", Tốc độ (lọc): "); Serial.print(controlData.speed);
        Serial.print(", Servo (raw): "); Serial.print(servoRaw);
        Serial.print(", Servo (lọc): "); Serial.print(controlData.servo);
        Serial.print(", Servotrim: "); Serial.print(controlData.trimservo);
        Serial.print(", speedtrim: "); Serial.print(controlData.trimspeed);
        Serial.print(", chedo: "); Serial.println(controlData.chedo);

        // Gửi dữ liệu
        sendData();

        // OLED hiển thị
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("TAY CAM DIEU KHIEN");
    display.print("Trang thai: ");
    display.println(controlData.ketnoi ? "OK" : "NOT OK");

    display.print("Che do: ");
    display.println(controlData.chedo ? "Tu hanh" : "Tay");

    display.print("Throttle: ");
    display.println(controlData.speed);

    display.print("Steer: ");
    display.println(controlData.servo);

    display.print("Trim toc do: ");
    display.println(controlData.trimspeed);

    display.print("Trim goc giua: ");
    display.println(controlData.trimservo);

    display.display();
        lastSendTime = currentTime;
    }
}