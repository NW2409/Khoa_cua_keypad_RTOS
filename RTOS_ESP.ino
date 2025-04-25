#define BLYNK_TEMPLATE_ID "TMPL6cHopklRI"
#define BLYNK_TEMPLATE_NAME "FreeRTOS"
#define BLYNK_AUTH_TOKEN "icRFroeiR1uMnfUpIxUTmtrxvFJL-qbD"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>


char ssid[] = "QBAH";
char pass[] = "qbah12345";
#define RX_PIN 16
#define TX_PIN 17
String oldPassword = "";
String newPassword = "";
bool changePasswordTriggered = false;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("ESP32 khởi động...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  if (Blynk.connected()) {
    Serial.println("Kết nối Blynk thành công!");
  } else {
    Serial.println("Lỗi: Không kết nối được với Blynk!");
    while (1);
  }
}

BLYNK_WRITE(V1) {
  oldPassword = param.asStr();
  Serial.print("Mật khẩu cũ từ Blynk: ");
  Serial.println(oldPassword);
}

BLYNK_WRITE(V2) {
  newPassword = param.asStr();
  Serial.print("Mật khẩu mới từ Blynk: ");
  Serial.println(newPassword);
}

BLYNK_WRITE(V0) {
  changePasswordTriggered = param.asInt();
  if (changePasswordTriggered) {
    Serial.println("Nhận lệnh đổi mật khẩu từ Blynk...");
    Serial1.print("CHANGE_PASS:");
    Serial1.print(oldPassword);
    Serial1.print(":");
    Serial1.println(newPassword);
    Blynk.virtualWrite(V3, "Đang xử lý...");
  }
}

void loop() {
  Blynk.run();
  if (!Blynk.connected()) {
    Serial.println("Mất kết nối với Blynk!");
    delay(5000);
    return;
  }
  if (Serial1.available()) {
    String receivedData = Serial1.readStringUntil('\n');
    receivedData.trim();
    Serial.print("ESP32: Nhận được từ Arduino: ");
    Serial.println(receivedData);
    if (receivedData == "PASS_CHANGE_SUCCESS") {
      Blynk.virtualWrite(V0, "Đổi mật khẩu thành công!");
      Serial.println("Đã cập nhật trạng thái thành công trên Blynk!");
    } else if (receivedData == "PASS_CHANGE_FAIL") {
      Blynk.virtualWrite(V0, "Đổi mật khẩu thất bại!");
      Serial.println("Đã cập nhật trạng thái thất bại trên Blynk!");
    } else if (receivedData == "P") {
      Serial.println("Nhận được 'P', gửi thông báo đến Blynk...");
      Blynk.logEvent("person_detected", "Phát hiện có người ngoài cửa ⚠️");
      Serial.println("Đã gửi thông báo đến Blynk!");
    } else if (receivedData == "W") {
      Serial.println("Nhận được 'W', gửi cảnh báo nhập sai mật khẩu đến Blynk...");
      Blynk.logEvent("wrong_password", "CẢNH BÁO: Hệ thống khóa 10s ⚠️");
      Serial.println("Đã gửi cảnh báo nhập sai mật khẩu đến Blynk!");
    }
  }
}