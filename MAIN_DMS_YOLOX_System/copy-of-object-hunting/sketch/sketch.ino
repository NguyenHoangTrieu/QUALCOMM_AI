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
#include <CAN.h>

#define BAUD_RATE 115200
#define BUZZER_PIN 8      // Chân PWM còi báo động trực tiếp trên Arduino

// ======================================================
// 0. FDCAN1 -> vcs-mcxn947 (real CAN link, added 2026-08-15)
// ======================================================
// D4(PA12)=TX, D5(PA11)=RX qua transceiver ngoài (xem
// ../../../dms-ap-uno-q/README.md "Hardware setup that is CONFIRMED to
// work" -- cùng board, cùng cách đấu dây, đã test thật trên phần cứng).
// alertLevel do send_dms_bundle() tính từ YOLOX được bắn thành DMS_STATUS
// (ICD 0x100) mỗi 100 ms độc lập với tốc độ camera (xem loop()) -- vì
// FRAME_TIME_MS=400ms chậm hơn nhiều so với chu kỳ 100ms mà vcs-mcxn947
// mong đợi (CAN-060..066), nếu bắn DMS_STATUS đúng lúc có frame mới thì
// link bên VCS sẽ liên tục rơi vào DEGRADED. Tách nhịp CAN heartbeat khỏi
// nhịp AI giải quyết việc này.
#define CANID_DMS_STATUS (0x100U)
#define CANID_VCS_STATUS (0x200U)

static uint8_t Crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
        }
    }
    return (uint8_t)(crc ^ 0xFF);
}

static int s_lastAlertLevel = 0;      // cập nhật bởi send_dms_bundle(), bắn ra CAN bởi loop()
static uint8_t s_dmsStatusSeq = 0;

// Tốc độ xe THẬT nhận về từ vcs-mcxn947 qua VCS_STATUS (duty_left/duty_right,
// 0-100%) -- thay cho random-walk giả lập trước đây. vcs-mcxn947 không có
// cảm biến tốc độ thật (xem vcs-mcxn947/README.md "What is NOT wired yet"),
// nên đây vẫn là quy đổi duty% -> km/h giả lập, chỉ khác là lấy dữ liệu
// THẬT qua CAN thay vì random() cục bộ trên UNO Q.
#define MAX_SPEED_KMH (40)
static uint8_t s_lastDutyLeft = 0;
static uint8_t s_lastDutyRight = 0;
static bool s_haveRealSpeed = false;

static void OnCanReceive(CanFDMsg const &msg, void *user_data) {
    (void)user_data;
    if (msg.getStandardId() == CANID_VCS_STATUS && msg.data_length == 8) {
        s_lastDutyLeft   = msg.data[2] & 0x7F;
        s_lastDutyRight  = msg.data[3] & 0x7F;
        s_haveRealSpeed  = true;
    }
    // VCS_EVENT/EMERGENCY_STOP: không xử lý ở app này -- xem
    // dms-ap-uno-q/sketch.ino nếu cần forward các loại frame đó.
}

// ======================================================
// 1. CẤU HÌNH THỜI GIAN THỰC TẾ & NGƯỠNG SETUP
// ======================================================
#define DEBUG_MODE 1 // Set 1: MCU in Label & Confidence nhận từ MPU | Set 0: Tắt debug, hiển thị gọn theo ALERT

const unsigned int FRAME_TIME_MS = 400; // Thời gian xử lý thực tế 1 frame (ms)

// Ngưỡng độ tin cậy AI Model (YOLOX) định nghĩa trực tiếp trên MCU
const float CONF_EYE_TH  = 0.20f; // Ngưỡng tin cậy nhận diện nhắm/mở mắt
const float CONF_YAWN_TH = 0.70f; // Ngưỡng tin cậy nhận diện ngáp

// Thời gian mục tiêu mong muốn (ms)
const unsigned int TARGET_EYE_WARN_MS   = 1000; // 1.0s -> Nhắc nhở
const unsigned int TARGET_EYE_ALARM_MS  = 2000; // 2.0s -> Báo động nguy hiểm
const unsigned int TARGET_EYE_DANGER_MS = 4000; // 4.0s -> Nguy hiểm cực độ (L3_DANGER)
const unsigned int TARGET_YAWN_WARN_MS  = 1500; // 1.5s -> Nhắc ngáp

// Số lượng Frame liên tục tự động tính tại setup()
int EYE_WARN_FRAMES;
int EYE_ALARM_FRAMES;
int EYE_DANGER_FRAMES;
int YAWN_WARN_FRAMES;

// Biến bộ đếm Frame liên tục trên MCU
int currentEyeClosedFrames = 0;
int currentYawnFrames = 0;
bool wasEyeClosedLong = false; // Theo dõi trạng thái đã nhắm mắt lâu trước đó

int getVehicleSpeed() {
    // Tốc độ THẬT lấy từ VCS_STATUS qua CAN (xem OnCanReceive ở trên).
    // Trước khi nhận được VCS_STATUS đầu tiên (chưa nối CAN / vcs-mcxn947
    // chưa bật) trả về 0, không còn random giả lập nữa.
    if (!s_haveRealSpeed) {
        return 0;
    }
    int avgDutyPct = ((int)s_lastDutyLeft + (int)s_lastDutyRight) / 2;
    return (avgDutyPct * MAX_SPEED_KMH) / 100;
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

    // --- B. ĐÁNH GIÁ MỨC CẢNH BÁO (LEVEL 0 - 3) ---
    int alertLevel = 0;
    int alertCode = 100;

    // Ưu tiên 1 (2026-08-16, thêm theo yêu cầu): nguy hiểm cực độ -- nhắm
    // mắt >= EYE_DANGER_FRAMES (>= 10 frames = 4.0s, gấp đôi ngưỡng L2).
    // Trước đây app này KHÔNG BAO GIỜ tự phát L3_DANGER (dừng ở L2 dù nhắm
    // mắt bao lâu) -- L3 bên vcs-mcxn947 trước đó chỉ từng xuất hiện qua
    // fallback CAN-063 khi mất link CAN, chưa bao giờ do chính DMS phán
    // đoán. Nhánh này lấp đúng khoảng trống đó.
    if (currentEyeClosedFrames >= EYE_DANGER_FRAMES) {
        alertLevel = 3;  // DANGER (Ngủ gật nguy hiểm cực độ, kéo dài)
        alertCode = 300;
    }
    // Ưu tiên 2: Ngủ gật nguy hiểm (Nhắm mắt >= EYE_ALARM_FRAMES, tức >= 5 frames = 2.0s)
    else if (currentEyeClosedFrames >= EYE_ALARM_FRAMES) {
        alertLevel = 2;  // SEVERE_ALARM (Ngủ gật)
        alertCode = 200;
    }
    // Ưu tiên 3: Nhắc nhở nhắm mắt (EYE_WARN_FRAMES <= frames < EYE_ALARM_FRAMES, tức 3-4 frames)
    else if (currentEyeClosedFrames >= EYE_WARN_FRAMES) {
        alertLevel = 1;  // LIGHT_WARN (Nhắm mắt)
        alertCode = 102;
    }
    // Ưu tiên 4: Nhắc ngáp kéo dài (>= YAWN_WARN_FRAMES, tức >= 4 frames = 1.5s)
    else if (currentYawnFrames >= YAWN_WARN_FRAMES) {
        alertLevel = 1;  // LIGHT_WARN (Ngáp)
        alertCode = 101;
    }

    int currentSpeed = getVehicleSpeed();

    // alertLevel (0-3) ánh xạ trực tiếp vào DMS_STATUS.alert_level
    // (0=L0_NORMAL 1=L1_EARLY 2=L2_DROWSY 3=L3_DANGER) qua CAN thật.
    // CAN TX thật sự nằm trong loop() (nhịp 100ms riêng, không phụ thuộc
    // tốc độ camera) -- ở đây chỉ cập nhật giá trị mới nhất.
    s_lastAlertLevel = alertLevel;

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
        if (alertLevel == 3) {
            tone(BUZZER_PIN, 3000, 500); // Gắt nhất, 3000Hz kéo dài (Nguy hiểm cực độ)
        } else if (alertLevel == 2) {
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

    // FDCAN1 -> vcs-mcxn947 thật (xem khối "0." ở đầu file)
    if (!CAN.begin(CanBitRate::BR_500k)) {
        logBoth("[CAN] CAN.begin(BR_500k) THAT BAI -- khong ket noi duoc voi vcs-mcxn947");
    } else {
        CAN.onReceive(OnCanReceive, nullptr);
        logBoth("[CAN] FDCAN1 up 500 kbit/s (classic CAN)");
    }

    // 2. TỰ ĐỘNG QUY ĐỔI SỐ FRAME LIÊN TỤC TẠI SETUP
    EYE_WARN_FRAMES   = ceil((float)TARGET_EYE_WARN_MS / FRAME_TIME_MS);   // ceil(1000/400) = 3 frames
    EYE_ALARM_FRAMES  = ceil((float)TARGET_EYE_ALARM_MS / FRAME_TIME_MS);  // ceil(2000/400) = 5 frames
    EYE_DANGER_FRAMES = ceil((float)TARGET_EYE_DANGER_MS / FRAME_TIME_MS); // ceil(4000/400) = 10 frames
    YAWN_WARN_FRAMES  = ceil((float)TARGET_YAWN_WARN_MS / FRAME_TIME_MS);  // ceil(1500/400) = 4 frames

    // Xuất thông số khởi tạo đồng thời ra 3 cổng
    logBoth("==================================================");
    logBoth("  DMS MCU MASTER DECISION ENGINE STARTED");
    logBoth("  • Frame Time Input: " + String(FRAME_TIME_MS) + " ms");
    logBoth("  • Eye Warn Limit  : " + String(EYE_WARN_FRAMES) + " frames (1.0s)");
    logBoth("  • Eye Alarm Limit : " + String(EYE_ALARM_FRAMES) + " frames (2.0s)");
    logBoth("  • Eye Danger Limit: " + String(EYE_DANGER_FRAMES) + " frames (4.0s)");
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

    // 3. Bắn DMS_STATUS thật lên FDCAN1 mỗi 100ms -- nhịp riêng, KHÔNG chờ
    // frame camera mới (xem giải thích ở khối "0." đầu file). Luôn dùng
    // s_lastAlertLevel mới nhất do send_dms_bundle() cập nhật.
    static unsigned long lastCanTx = 0;
    if (millis() - lastCanTx >= 100) {
        lastCanTx = millis();

        uint8_t payload[8] = {0};
        payload[0] = (uint8_t)((s_lastAlertLevel & 0x0F) | ((s_dmsStatusSeq & 0x0F) << 4));
        payload[5] = 100; // face_conf_pct, không phải 255 ("no face")
        payload[7] = Crc8(payload, 7);
        s_dmsStatusSeq = (uint8_t)((s_dmsStatusSeq + 1) & 0x0F);

        CanMsg msg(CanStandardId(CANID_DMS_STATUS), 8, payload);
        int rc = CAN.write(msg);
        static uint32_t txOk = 0, txFail = 0;
        if (rc > 0) txOk++; else txFail++;
        static unsigned long lastCanStat = 0;
        if (millis() - lastCanStat >= 1000) {
            lastCanStat = millis();
            logBoth("[CAN] tx_ok=" + String(txOk) + " tx_fail=" + String(txFail) +
                    " (last rc=" + String(rc) + ")");
        }
    }
}
