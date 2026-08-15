# SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l. and/or its affiliated companies
#
# SPDX-License-Identifier: MPL-2.0

import time
from datetime import datetime, UTC
from arduino.app_utils import App, Bridge, Logger
from arduino.app_peripherals.camera import Camera
from arduino.app_bricks.web_ui import WebUI
from arduino.app_bricks.video_objectdetection import VideoObjectDetection

logger = Logger("DMS_ObjectHunting")
ui = WebUI()
camera = Camera(fps=3)
detection_stream = VideoObjectDetection(camera=camera, debounce_sec=0.5, camera_preview=False)

ui.on_message("override_th", lambda sid, threshold: detection_stream.override_threshold(threshold))

# Biến toàn cục theo dõi thời gian và đếm khung hình
last_inference_time = None
frame_id_counter = 0

def get_confidence(obj) -> float:
    """Hàm an toàn lấy độ tin cậy từ đối tượng phát hiện"""
    if isinstance(obj, dict):
        return float(obj.get("confidence", 0.0))
    elif isinstance(obj, (int, float)):
        return float(obj)
    return 0.0

def get_bbox(obj):
    """Hàm an toàn lấy tọa độ Bounding Box từ đối tượng phát hiện (Trả về None nếu không có tọa độ thực)"""
    if isinstance(obj, dict):
        if "box" in obj and isinstance(obj["box"], list) and len(obj["box"]) == 4:
            return obj["box"]
        if "bbox" in obj and isinstance(obj["bbox"], list) and len(obj["bbox"]) == 4:
            return obj["bbox"]
        if "xmin" in obj and all(k in obj for k in ("xmin", "ymin", "xmax", "ymax")):
            return [obj.get("xmin"), obj.get("ymin"), obj.get("xmax"), obj.get("ymax")]
    return None
    
def send_detections_to_ui(detections: dict):
    """
    Hàm xử lý kết quả nhận diện từ YOLOX (Đã tối ưu giảm độ trễ):
    1. Tính toán FPS và hiệu năng xử lý.
    2. Gom toàn bộ Bounding Box và FPS vào 1 gói tin WebSocket duy nhất gửi Web UI.
    3. Gom Class & Confidence vào 1 message duy nhất gửi MCU.
    """
    global last_inference_time, frame_id_counter
    frame_id_counter += 1
    current_time = time.perf_counter()

    if last_inference_time is not None:
        dt = current_time - last_inference_time
        fps = 1.0 / dt if dt > 0 else 0.0
    else:
        fps = 0.0
    last_inference_time = current_time

    boxes_data = []
    detection_items = []

    if isinstance(detections, dict):
        for label, object_list in detections.items():
            if isinstance(object_list, list):
                for obj in object_list:
                    accuracy = get_confidence(obj)
                    bbox = get_bbox(obj)
                    item = {
                        "label": str(label),
                        "confidence": round(accuracy, 2)
                    }
                    if bbox is not None:
                        item["bbox"] = bbox
                    boxes_data.append(item)
                    detection_items.append(f"{label}:{accuracy:.2f}")

    # Gửi 1 message duy nhất cập nhật UI (Tránh lặp làm ngẽn đường truyền Socket)
    ui.send_message("frame_boxes", message={"boxes": boxes_data, "fps": round(fps, 1)})

    # Gom thành 1 message duy nhất gửi MCU
    all_detections_str = ",".join(detection_items) if detection_items else "none"

    logger.info(f"📤 [MPU Tx -> MCU] Frame #{frame_id_counter} | Detections: {all_detections_str}")

    try:
        Bridge.call("send_dms_bundle", frame_id_counter, all_detections_str)
    except Exception as e:
        logger.error(f"Lỗi gửi Bundle xuống MCU: {e}")

# Callback tiếp nhận phản hồi xác nhận nhận dữ liệu thành công từ MCU
def on_mcu_ack(ack_msg: str):
    logger.info(f"📩 [MPU Rx <- MCU ACK] MCU đã xác nhận nhận thành công: {ack_msg}")

# Callback tiếp nhận phản hồi hoặc thay đổi cấu hình từ UART ngoại vi (qua MCU)
def on_dms_config(raw_cmd: str):
    logger.info(f"📥 [MPU Config] Nhận lệnh từ UART ngoại vi: {raw_cmd}")

# Callback tiếp nhận Telemetry (Alert Level, Speed, Frame Cnt) từ MCU đẩy lên UI
def on_mcu_telemetry(alert_level: int, alert_code: int, speed: int, eye_frames: int, yawn_frames: int):
    telemetry_payload = {
        "alert_level": alert_level,
        "alert_code": alert_code,
        "speed": speed,
        "eye_frames": eye_frames,
        "yawn_frames": yawn_frames
    }
    ui.send_message("dms_telemetry", message=telemetry_payload)

# Lắng nghe sự kiện người dùng chạm màn hình UI để tắt cảnh báo -> gọi xuống MCU
def handle_dismiss_alert(sid, data=None):
    logger.info("🖐️ [MPU Rx UI] Chạm màn hình tắt cảnh báo -> Gửi lệnh dismiss_alert xuống MCU")
    try:
        Bridge.call("dismiss_alert")
    except Exception as e:
        logger.error(f"Lỗi gọi dismiss_alert xuống MCU: {e}")

Bridge.provide("on_mcu_ack", on_mcu_ack)
Bridge.provide("on_dms_config", on_dms_config)
Bridge.provide("on_mcu_telemetry", on_mcu_telemetry)

ui.on_message("dismiss_alert", handle_dismiss_alert)

# Đăng ký hàm callback xử lý mỗi khi có kết quả phát hiện từ mô hình
detection_stream.on_detect_all(send_detections_to_ui)

App.run()