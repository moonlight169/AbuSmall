#include <Arduino.h>
#include <Ros2Duino.h>

// สร้าง Object ชื่อ esp32hello
Ros2Duino esp32hello; 

// ฟังก์ชัน Callback: จะทำงานเมื่อมีข้อความ String ส่งมาจาก ROS 2
void stringCallback(String data) {
  Serial.print("ได้รับข้อความจาก ROS 2: ");
  Serial.println(data);
  
  // ถ้าได้รับคำว่า "hello" ให้เปิดไฟ LED บนบอร์ดโชว์เลย
  if(data == "hello") {
    digitalWrite(2, HIGH); 
    delay(500);
    digitalWrite(2, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT); // LED บนบอร์ด ESP32 ส่วนใหญ่คือขา 2

  // 1. ตั้งค่า Identity (ID: 1, Name: master_esp32)
  esp32hello.identity(1, "master_esp32"); 

  // 2. เลือกการเชื่อมต่อแบบ Serial (สาย USB)
  esp32hello.useSerial(Serial, 115200); 

  // 3. ผูกฟังก์ชัน Callback สำหรับรอรับ String[cite: 1]
  esp32hello.onString(stringCallback); 

  // 4. เริ่มต้นระบบ[cite: 3]
  esp32hello.begin(); 
  
  Serial.println("ESP32 Master พร้อมรับข้อมูลแล้ว...");
}

void loop() {
  // 5. บรรทัดสำคัญ: ต้องเรียกใช้เพื่อเช็กข้อมูลขาเข้าตลอดเวลา[cite: 3]
  esp32hello.spinOnce(); 
}