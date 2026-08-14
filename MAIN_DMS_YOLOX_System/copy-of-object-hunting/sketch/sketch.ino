// SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
//
// SPDX-License-Identifier: MPL-2.0

/* 
   DMS Master Decision Engine Sketch for Arduino MCU
   - Nhận Gói A (DMS Bundle) từ MPU Python -> Tự đếm số lượng Frame liên tục
   - Ra quyết định cảnh báo / Bật còi vật lý.
   - XUẤT DỮ LIỆU XÁC NHẬN ĐỒNG THỜI RA CẢ 3 CỔNG (MONITOR, SERIAL, SERIAL1) CHO MỖI FRAME
   - PHẢN HỒI ACK NGƯỢC LÊN MPU PYTHON ĐỂ XÁC NHẬN KẾT NỐI HAI CHIỀU THÀNH CÔNG
*/

#include "Arduino_RouterBridge.h"
#include <math.h>

#define BAUD_RATE 115200
#define BUZZER_PIN 8      // Chân PWM còi báo động trực tiếp trên Arduino

// ======================================================
// 1. CẤU HÌNH THỜI GIAN THỰC TẾ & NGƯỠNG SETUP
// ======================================================
#define DEBUG_MODE 1 // Set 1: MCU in Label & Confidence nhận từ MPU | Set 0: Tắt debug, hiển thị gọn theo ALERT

const unsigned int FRAME_TIME_MS = 400; // Thời gian xử lý thực tế 1 frame (ms)

// Ngưỡng độ tin cậy AI Model (YOLOX) định nghĩa trực tiếp trên MCU
const float CONF_EYE_TH  = 0.65f; // Ngưỡng tin cậy nhận diện nhắm/mở mắt
const float CONF_YAWN_TH = 0.70f; // Ngưỡng tin cậy nhận diện ngáp

// Thời gian mục tiêu mong muốn (ms)
const unsigned int TARGET_EYE_WARN_MS  = 1000; // 1.0s -> Nhắc nhở
const unsigned int TARGET_EYE_ALARM_MS = 2000; // 2.0s -> Báo động nguy hiểm
const unsigned int TARGET_YAWN_WARN_MS = 1500; // 1.5s -> Nhắc ngáp

// Số lượng Frame liên tục tự động tính tại setup()
int EYE_WARN_FRAMES;
int EYE_ALARM_FRAMES;
int YAWN_WARN_FRAMES;

// Biến bộ đếm Frame liên tục trên MCU
int currentEyeClosedFrames = 0;
int currentYawnFrames = 0;
bool wasEyeClosedLong = false; // Theo dõi trạng thái đã nhắm mắt lâu trước đó

// Biến giả lập tốc độ xe (km/h)
int simulatedSpeed = 75;
unsigned long lastSpeedUpdate = 0;

int getVehicleSpeed() {
    // Giả lập tốc độ xe biến thiên tự nhiên từ 50 đến 115 km/h mỗi giây
    if (millis() - lastSpeedUpdate > 1000) {
        lastSpeedUpdate = millis();
        simulatedSpeed += random(-3, 4);
        if (simulatedSpeed < 50) simulatedSpeed = 50;
        if (simulatedSpeed > 115) simulatedSpeed = 115;
    }
    return simulatedSpeed;
}

// RPC: Tắt cảnh báo khi người dùng chạm màn hình Web UI
void dismiss_alert() {
    currentEyeClosedFrames = 0;
    currentYawnFrames = 0;
    wasEyeClosedLong = false;
    noTone(BUZZER_PIN);
    logBoth("🖐️ [MCU DISMISS] Người dùng chạm màn hình tắt cảnh báo -> Reset cờ & còi!");
    
    // Đẩy ngay telemetry an toàn (Level 0) lên MPU
    Bridge.notify("on_mcu_telemetry", 0, 100, getVehicleSpeed(), 0, 0);
}

String rxBufferSerial = "";
String rxBufferSerial1 = "";

// ======================================================
// HÀM BỔ TRỢ: XUẤT LOG ĐỒNG THỜI RA TẤT CẢ CÁC CỔNG (MONITOR, SERIAL, SERIAL1)
// ======================================================
void logBoth(const String& msg) {
    Monitor.println(msg);     // 1. Log bên trong (App Lab Serial Monitor)
    Serial.println(msg);      // 2. Log ra USB Serial
    Serial1.println(msg);     // 3. Log ra Hardware Serial1 (UART Pins)
}

void printBoth(const String& msg) {
    Monitor.print(msg);
    Serial.print(msg);
    Serial1.print(msg);
}

// ======================================================
// 2. MCU TIẾP NHẬN TOÀN BỘ CLASS & CONFIDENCE TỪ MPU DẠNG 1 MESSAGE
// ======================================================
void send_dms_bundle(int frame_id, String detections_str) {
    String cleanDetections = detections_str;
    cleanDetections.toLowerCase();
    cleanDetections.trim();

    bool isEyeClosed = false;
    bool isEyeOpen   = false;
    bool isYawning   = false;

    // Phân tích chuỗi danh sách class trong message (dạng "closed_eye:0.85,yawning:0.72")
    int start = 0;
    while (start < cleanDetections.length()) {
        int end = cleanDetections.indexOf(',', start);
        if (end == -1) end = cleanDetections.length();

        String item = cleanDetections.substring(start, end);
        item.trim();
        start = end + 1;

        if (item.length() == 0) continue;

        int colon = item.indexOf(':');
        String label = (colon != -1) ? item.substring(0, colon) : item;
        float conf = (colon != -1) ? item.substring(colon + 1).toFloat() : 1.0f;
        label.trim();

        if ((label == "closed_eye" || label == "eye_closed" || label == "drowsy") && conf >= CONF_EYE_TH) {
            isEyeClosed = true;
        }
        if ((label == "open_eye" || label == "eye_open") && conf >= CONF_EYE_TH) {
            isEyeOpen = true;
        }
        if ((label == "yawning" || label == "yawn" || label == "mouth_open") && conf >= CONF_YAWN_TH) {
            isYawning = true;
        }
    }

    // Cập nhật bộ đếm nhắm mắt & phát hiện sự kiện mở mắt trở lại sau khi nhắm lâu
    if (isEyeClosed && !isEyeOpen) {
        currentEyeClosedFrames++;
        if (currentEyeClosedFrames >= EYE_WARN_FRAMES) {
            wasEyeClosedLong = true; // Đánh dấu đã xảy ra nhắm mắt lâu
        }
    } else if (isEyeOpen) {
        if (wasEyeClosedLong) {
            // TÍN HIỆU PHẢN HỒI: TÀI XẾ ĐÃ MỞ MẮT TRỞ LẠI SAU KHI NHẮM MẮT LÂU
            String eventLog = "👁️ [MCU EVENT] DRIVER REOPENED EYES AFTER PROLONGED CLOSURE (" + String(currentEyeClosedFrames) + " frames)! RECOVERED.";
            logBoth(eventLog);

            // Gửi sự kiện phản hồi lên MPU Python
            Bridge.notify("on_driver_reopened", getVehicleSpeed(), currentEyeClosedFrames);
            wasEyeClosedLong = false;
        }
        currentEyeClosedFrames = 0; // Reset đếm frame nhắm mắt
    } else {
        if (currentEyeClosedFrames > 0) currentEyeClosedFrames--; // Giảm dần an toàn
    }

    // Cập nhật bộ đếm ngáp
    if (isYawning) {
        currentYawnFrames++;
    } else {
        currentYawnFrames = 0; // Reset ngay khi hết ngáp
    }

    // --- B. ĐÁNH GIÁ MỨC CẢNH BÁO (LEVEL 0 - 2) ---
    int alertLevel = 0;
    int alertCode = 100;

    // Ưu tiên 1: Ngủ gật nguy hiểm (Nhắm mắt >= EYE_ALARM_FRAMES, tức >= 5 frames = 2.0s)
    if (currentEyeClosedFrames >= EYE_ALARM_FRAMES) {
        alertLevel = 2;  // SEVERE_ALARM (Ngủ gật)
        alertCode = 200;
    }
    // Ưu tiên 2: Nhắc nhở nhắm mắt (EYE_WARN_FRAMES <= frames < EYE_ALARM_FRAMES, tức 3-4 frames)
    else if (currentEyeClosedFrames >= EYE_WARN_FRAMES) {
        alertLevel = 1;  // LIGHT_WARN (Nhắm mắt)
        alertCode = 102;
    }
    // Ưu tiên 3: Nhắc ngáp kéo dài (>= YAWN_WARN_FRAMES, tức >= 4 frames = 1.5s)
    else if (currentYawnFrames >= YAWN_WARN_FRAMES) {
        alertLevel = 1;  // LIGHT_WARN (Ngáp)
        alertCode = 101;
    }

    int currentSpeed = getVehicleSpeed();

    // --- B2. GỬI PHẢN HỒI TELEMETRY (ALERT LEVEL & TỐC ĐỘ) LÊN MPU PYTHON ---
    Bridge.notify("on_mcu_telemetry", alertLevel, alertCode, currentSpeed, currentEyeClosedFrames, currentYawnFrames);

#if DEBUG_MODE == 1
    // --- C. LOG DEBUG CHI TIẾT KHI DEBUG_MODE = 1 ---
    String debugLog = "🔍 [MCU DEBUG] Frame #" + String(frame_id) +
                      " | Speed: " + String(currentSpeed) + " km/h" +
                      " | Rx: [" + detections_str + "]" +
                      " | EyeCnt: " + String(currentEyeClosedFrames) +
                      " | YawnCnt: " + String(currentYawnFrames);
    logBoth(debugLog);
#endif

    // --- D. PHẢN HỒI ACK NGƯỢC LÊN MPU ---
    String ackMsg = "ACK_FRAME_" + String(frame_id);
    Bridge.notify("on_mcu_ack", ackMsg);

    // --- E. KÍCH HOẠT PHẦN CỨNG & XUẤT GÓI B KHI CÓ CẢNH BÁO ---
    if (alertLevel > 0) {
        String packetB = "   🚨 [DMS_ALERT] SEQ:" + String(frame_id) +
                         ", LEVEL:" + String(alertLevel) +
                         ", CODE:" + String(alertCode) +
                         ", SPEED:" + String(currentSpeed) +
                         ", DETECTIONS:" + detections_str +
                         ", EYE_FRAMES:" + String(currentEyeClosedFrames) +
                         ", YAWN_FRAMES:" + String(currentYawnFrames);

        logBoth(packetB);

        // Điều khiển Còi báo động vật lý trực tiếp
        if (alertLevel == 2) {
            tone(BUZZER_PIN, 2000, 300); // Tần số gắt 2000Hz (Báo động ngủ gật)
        } else {
            tone(BUZZER_PIN, 1000, 100); // Bíp ngắn 1000Hz (Nhắc nhở nhẹ)
        }
    } else {
        noTone(BUZZER_PIN);
    }
}

void setup() {
    // 1. Khởi tạo đồng thời 3 cổng log với baudrate 115200
    Monitor.begin(BAUD_RATE);
    Serial.begin(BAUD_RATE);
    Serial1.begin(BAUD_RATE);
    
    pinMode(BUZZER_PIN, OUTPUT);

    // 2. TỰ ĐỘNG QUY ĐỔI SỐ FRAME LIÊN TỤC TẠI SETUP
    EYE_WARN_FRAMES  = ceil((float)TARGET_EYE_WARN_MS / FRAME_TIME_MS);  // ceil(1000/400) = 3 frames
    EYE_ALARM_FRAMES = ceil((float)TARGET_EYE_ALARM_MS / FRAME_TIME_MS); // ceil(2000/400) = 5 frames
    YAWN_WARN_FRAMES = ceil((float)TARGET_YAWN_WARN_MS / FRAME_TIME_MS); // ceil(1500/400) = 4 frames

    // Xuất thông số khởi tạo đồng thời ra 3 cổng
    logBoth("==================================================");
    logBoth("  DMS MCU MASTER DECISION ENGINE STARTED");
    logBoth("  • Frame Time Input: " + String(FRAME_TIME_MS) + " ms");
    logBoth("  • Eye Warn Limit  : " + String(EYE_WARN_FRAMES) + " frames (1.0s)");
    logBoth("  • Eye Alarm Limit : " + String(EYE_ALARM_FRAMES) + " frames (2.0s)");
    logBoth("  • Yawn Warn Limit : " + String(YAWN_WARN_FRAMES) + " frames (1.5s)");
    logBoth("==================================================");
    
    Bridge.begin();
    Bridge.provide("send_dms_bundle", send_dms_bundle);
    Bridge.provide("dismiss_alert", dismiss_alert);
}

void loop() {
    // 1. Đọc lệnh từ cổng USB Serial
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            rxBufferSerial.trim();
            if (rxBufferSerial.length() > 0) {
                logBoth("📥 [MCU Serial Rx] Nhận lệnh từ Serial: " + rxBufferSerial);
                Bridge.notify("on_dms_config", rxBufferSerial);
            }
            rxBufferSerial = "";
        } else {
            rxBufferSerial += c;
            if (rxBufferSerial.length() > 256) {
                rxBufferSerial = ""; // Reset an toàn tránh tràn RAM
            }
        }
    }

    // 2. Đọc lệnh từ cổng Hardware Serial1
    while (Serial1.available() > 0) {
        char c = Serial1.read();
        if (c == '\n') {
            rxBufferSerial1.trim();
            if (rxBufferSerial1.length() > 0) {
                logBoth("📥 [MCU Serial1 Rx] Nhận lệnh từ Serial1: " + rxBufferSerial1);
                Bridge.notify("on_dms_config", rxBufferSerial1);
            }
            rxBufferSerial1 = "";
        } else {
            rxBufferSerial1 += c;
            if (rxBufferSerial1.length() > 256) {
                rxBufferSerial1 = ""; // Reset an toàn tránh tràn RAM
            }
        }
    }
}
