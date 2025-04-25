#include "Arduino_FreeRTOS.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Servo.h>
#include <EEPROM.h>
#include <semphr.h>
#include <queue.h>

//#define LED_PIN 13   // LED báo có người
#define SERVO_PIN 10  // Servo mở khóa
#define IR_SENSOR_PIN A0 
#define TRIG_PIN 11  // Chân Trig của HC-SR04
#define ECHO_PIN 12  // Chân Echo của HC-SR04
#define LDR_PIN 22      // Chân digital đọc tín hiệu từ cảm biến ánh sáng
#define LED_PIN 13      // LED tích hợp trên board Mega

LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo doorLock;

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
String inputPassword = "";
String savedPassword = "1111"; // Sử dụng String thay vì char[5]
int wrongAttempts = 0;
bool isChangingPassword = false;

// FreeRTOS handles
SemaphoreHandle_t passwordSemaphore;
SemaphoreHandle_t personDetectedSemaphore; 
QueueHandle_t distanceQueue;
 

void loadPassword();
void savePassword(const String& newPass);

void setup() {
  
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  doorLock.attach(SERVO_PIN);
  doorLock.write(0);

  lcd.init();
  lcd.noBacklight();
  lcd.setCursor(0, 0);
  lcd.print("Nhap mat khau:");

  Serial.begin(9600);
  Serial1.begin(9600);
  loadPassword();
  Serial.print("Mật khẩu ban đầu: ");
  Serial.println(savedPassword);

  passwordSemaphore = xSemaphoreCreateBinary();
  personDetectedSemaphore = xSemaphoreCreateBinary();
  distanceQueue = xQueueCreate(5, sizeof(int));

  if (passwordSemaphore == NULL || personDetectedSemaphore == NULL || distanceQueue == NULL) {
    Serial.println("Lỗi tạo FreeRTOS objects!");
    while (1);
  }

  xTaskCreate(Task_Keypad, "Keypad", 256, NULL, 3, NULL);
  xTaskCreate(Task_Servo, "Servo", 128, NULL, 2, NULL);
  xTaskCreate(Task_HCSR04, "HCSR04", 128, NULL, 3, NULL);
  xTaskCreate(Task_PhotoSensor, "PhotoSensor", 96, NULL, 2, NULL);
  xTaskCreate(Task_ChangePassword, "ChangePassword", 256, NULL, 2, NULL);

  vTaskStartScheduler();
}

void loop() {
  // Không dùng loop() trong FreeRTOS
}

void loadPassword() {
  int length = EEPROM.read(0); // Đọc độ dài mật khẩu từ byte đầu tiên
  if (length == 255 || length > 20) { // Giới hạn độ dài tối đa là 20 ký tự
    length = 4;
    savedPassword = "1111"; // Mặc định nếu EEPROM trống
  } else {
    savedPassword = "";
    for (int i = 0; i < length; i++) {
      char c = EEPROM.read(i + 1);
      if (c == 255) c = '1'; // Mặc định nếu byte trống
      savedPassword += c;
    }
  }
  Serial.print("Mật khẩu tải từ EEPROM: ");
  Serial.println(savedPassword);
}

void savePassword(const String& newPass) {
  int length = newPass.length();
  if (length > 20) length = 20; // Giới hạn độ dài tối đa
  EEPROM.write(0, length); // Lưu độ dài vào byte đầu tiên
  for (int i = 0; i < length; i++) {
    EEPROM.write(i + 1, newPass[i]);
  }
}
void Task_Keypad(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    char key = keypad.getKey();
    if (key) {
      lcd.backlight();
      if (key == 'A') {
        savePassword("1111");
        loadPassword();
        lcd.clear();
        lcd.print("Reset mat khau");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        lcd.clear();
        lcd.print("Nhap mat khau:");
      } 
      else if (key == 'D') {
        isChangingPassword = true;
        inputPassword = "";
        lcd.clear();
        lcd.print("Nhap MK cu:");
      } 
      else if (key == '#') {  
        if (isChangingPassword) {
          if (inputPassword == savedPassword) {  
            lcd.clear();
            lcd.print("Nhap MK moi:");
            inputPassword = "";
            String newPass = "";
            while (true) { 
              char newKey = keypad.getKey();
              if (newKey) {
                if (newKey == '#') {
                  if (newPass.length() > 0) { // Chỉ cần không rỗng
                    savePassword(newPass);
                    savedPassword = newPass;
                    Serial.print("Mật khẩu mới từ keypad: ");
                    Serial.println(savedPassword);
                    lcd.clear();
                    lcd.print("Doi thanh cong!");
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                  } else {
                    lcd.clear();
                    lcd.print("MK khong duoc rong!");
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                  }
                  break;
                } 
                else if (newKey == '*') {
                  if (newPass.length() > 0) {
                    newPass.remove(newPass.length() - 1);
                    lcd.setCursor(0, 1);
                    lcd.print("                ");
                    lcd.setCursor(0, 1);
                    lcd.print(newPass);
                  }
                } 
                else {
                  newPass += newKey;
                  lcd.setCursor(0, 1);
                  lcd.print(newPass);
                }
              }
            }
          } else {
            lcd.clear();
            lcd.print("MK cu sai!");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
          }
          isChangingPassword = false;
          lcd.clear();
          lcd.print("Nhap mat khau:");
        } 
        else {
          Serial.print("Mật khẩu nhập từ keypad: ");
          Serial.println(inputPassword);
          Serial.print("Mật khẩu hiện tại: ");
          Serial.println(savedPassword);
          if (inputPassword == savedPassword) {  
            xSemaphoreGive(passwordSemaphore);
            wrongAttempts = 0;
          } else {
            wrongAttempts++;
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Sai lan ");
            lcd.print(wrongAttempts);
            lcd.print("!");
            lcd.setCursor(0, 1);
            lcd.print("Thu lai...");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            lcd.clear();
            lcd.print("Nhap mat khau:");
            if (wrongAttempts >= 3) {
              lcd.clear();
              lcd.print("Canh bao!");
              Serial1.write('W'); // Gửi lệnh 'W' đến ESP32 để kích hoạt thông báo Blynk
              Serial.println("Arduino Mega: Gửi 'W' qua Serial1 để cảnh báo Blynk");
              for (int i = 0; i < 5; i++) {
               // digitalWrite(LED_PIN, HIGH);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                //digitalWrite(LED_PIN, LOW);
                vTaskDelay(500 / portTICK_PERIOD_MS);
              }
              lcd.clear();
              lcd.print("Khoa 10s...");
              vTaskDelay(10000 / portTICK_PERIOD_MS);
              lcd.clear();
              lcd.print("Nhap mat khau:");
    
              wrongAttempts = 0;
            }
          }
          inputPassword = "";
        }
      } 
      else if (key == '*') {  
        inputPassword = "";
        lcd.clear();
        lcd.print("Nhap mat khau:");
      } 
      else {
        inputPassword += key;
        lcd.setCursor(0, 1);
        lcd.print(inputPassword);
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void Task_Servo(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    if (xSemaphoreTake(passwordSemaphore, portMAX_DELAY) == pdTRUE) {
      lcd.backlight();
      lcd.clear();
      lcd.print("Mo cua...");
      doorLock.write(90);
      vTaskDelay(500 / portTICK_PERIOD_MS);

      lcd.clear();
      lcd.print("Cho di vao!");
      
      while (digitalRead(IR_SENSOR_PIN) == HIGH) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }

      int noPersonCount = 0;
      while (noPersonCount < 3) {  
        if (digitalRead(IR_SENSOR_PIN) == HIGH) {  
          noPersonCount++;  
        } else {  
          noPersonCount = 0;  
        }
        vTaskDelay(300 / portTICK_PERIOD_MS);
      }

      lcd.clear();
      lcd.print("Dong cua...");
      doorLock.write(0);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      lcd.clear();
      lcd.print("Nhap mat khau:");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void Task_ChangePassword(void *pvParameters) {
  (void) pvParameters;
  String receivedData = "";
  while (1) {
    if (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\n') {
        receivedData.trim();
        Serial.print("Arduino Mega: Nhận được từ ESP32: ");
        Serial.println(receivedData);
        if (receivedData.startsWith("CHANGE_PASS:")) {
          receivedData.replace("CHANGE_PASS:", "");
          int separatorIndex = receivedData.indexOf(':');
          if (separatorIndex != -1) {
            String oldPass = receivedData.substring(0, separatorIndex);
            String newPass = receivedData.substring(separatorIndex + 1);
            oldPass.trim();
            newPass.trim();
            Serial.print("Mật khẩu cũ nhận được: ");
            Serial.println(oldPass);
            Serial.print("Mật khẩu mới nhận được: ");
            Serial.println(newPass);

            if (oldPass == savedPassword) {
              if (newPass.length() > 0) { // Chỉ cần không rỗng
                savePassword(newPass);
                savedPassword = newPass;
                Serial.print("Mật khẩu mới đã cập nhật: ");
                Serial.println(savedPassword);
                lcd.clear();
                lcd.print("Doi MK hoan tat");
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                lcd.clear();
                lcd.print("Nhap mat khau:");
                Serial1.println("PASS_CHANGE_SUCCESS");
                Serial.println("Arduino Mega: Gửi PASS_CHANGE_SUCCESS");
              } else {
                lcd.clear();
                lcd.print("MK moi khong duoc rong");
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                lcd.clear();
                lcd.print("Nhap mat khau:");
                Serial1.println("PASS_CHANGE_FAIL");
                Serial.println("Arduino Mega: Gửi PASS_CHANGE_FAIL (MK mới rỗng)");
              }
            } else {
              lcd.clear();
              lcd.print("MK cu sai");
              vTaskDelay(2000 / portTICK_PERIOD_MS);
              lcd.clear();
              lcd.print("Nhap mat khau:");
              Serial1.println("PASS_CHANGE_FAIL");
              Serial.println("Arduino Mega: Gửi PASS_CHANGE_FAIL (MK cũ sai)");
            }
          } else {
            Serial.println("Arduino Mega: Dữ liệu không đúng định dạng!");
          }
        }
        receivedData = "";
      } else {
        receivedData += c;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void Task_HCSR04(void *pvParameters) {
  (void) pvParameters;
  static unsigned long lastPersonTime = 0;
  const unsigned long backlightTimeout = 10000;
  const int distanceThreshold = 50;
  static bool wasPersonDetected = false;
  static bool notificationSent = false;

  while (1) {
    digitalWrite(TRIG_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(2));
    digitalWrite(TRIG_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    int distance = duration * 0.034 / 2;

    xQueueSend(distanceQueue, &distance, 0);

    if (distance > 0 && distance < distanceThreshold) {
      lcd.backlight();
      if (!wasPersonDetected) {
        if (!notificationSent) {
          Serial.println("Arduino Mega: Gửi 'P' qua Serial1...");
          Serial1.write('P');
          notificationSent = true;
        }
        wasPersonDetected = true;
      }
      lastPersonTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
      //digitalWrite(LED_PIN, HIGH);
      Serial.println("Phát hiện có người ngoài cửa!");
      xSemaphoreGive(personDetectedSemaphore);
    } else {
      wasPersonDetected = false;
      notificationSent = false;
      //digitalWrite(LED_PIN, LOW);
      if (lastPersonTime != 0 && 
          (xTaskGetTickCount() * portTICK_PERIOD_MS - lastPersonTime) > backlightTimeout) {
        lcd.noBacklight();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
void Task_PhotoSensor(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    int lightState = digitalRead(LDR_PIN); // Đọc tín hiệu digital từ cảm biến ánh sáng
    if (lightState == LOW) { // Trời tối
      digitalWrite( LED_PIN, LOW); // Bật LED
      //Serial.println("Tối - LED bật");
    } else { // Trời sáng
      digitalWrite( LED_PIN, HIGH); // Tắt LED
      //Serial.println("Sáng - LED tắt");
    }
  }
}