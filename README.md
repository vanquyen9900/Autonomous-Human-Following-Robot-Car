# Autonomous Human-Following Robot Car (Raspberry Pi 5)

**Mục tiêu:** Xe/robot di động tự hành **bám theo người** (human-following) sử dụng camera (Orbbec/Astra), mô hình phát hiện đối tượng (ONNX/PyTorch) và bộ theo dõi OCSort. Dự án xây dựng dưới dạng package ROS2 (tên package: `bramy`) và hướng tới chạy trên **Raspberry Pi 5**.

---

## Tổng quan kiến trúc
- **ROS2 nodes**
  - `camera_publisher` — đọc depth + color từ camera (OpenNI / Astra) và publish lên topic `color_image` / `depth_image`.
  - `tracking` — nhận ảnh, chạy phát hiện, vẽ bounding box, có phần hiển thị (cv2.imshow).
  - `postProcess` — xử lý hậu (ONNX + OCSort) để tạo các track ổn định.
  - `get_control` — giao tiếp CAN (`python-can`, `socketcan`) để điều khiển tốc độ/góc lái của xe.

- **Theo dõi**: OCSort (thư mục `bramy/ocsort`) được tích hợp để theo dõi nhiều đối tượng, chọn target bám theo, và trả về tọa độ trung tâm / offset để điều khiển xe.

---

## Yêu cầu phần cứng (hardware)
- **Raspberry Pi 5** (hoặc Pi tương thích)
- **Camera depth/color** hỗ trợ OpenNI (ví dụ Orbbec Astra / Astra Pro)
- **Motor driver / motor controller** có giao tiếp CAN hoặc module điều khiển nhận lệnh từ CAN bus
- **CAN interface**: MCP2515 (SPI) hoặc USB-CAN adapter (tương thích socketcan)
- Nguồn cấp phù hợp cho Pi + motors

---

## Yêu cầu phần mềm (packages chính)
- ROS2 Jazzy (theo README gốc) — hoặc ROS2 release tương thích trên Pi (cài đúng release bạn định dùng)
- Python packages:
  - `rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge` (ROS2)
  - `opencv-python`, `numpy`
  - `onnxruntime`, `torch` (nếu dùng PyTorch; ONNX runtime để chạy model .onnx)
  - `openni` (openni2 bindings) — driver camera Astra
  - `python-can` (để truy cập socketcan)
- Hệ thống: `colcon` để build ROS2 workspace

> **Lưu ý:** Có thể cần cài thêm các thư viện native cho OpenNI, libusb, v.v. Đảm bảo drivers của camera được cài (OpenNI2 hoặc Orbbec SDK).

---

## Cấu trúc chính của repository
```
rclpy/
  car/
    bramy/
      camera/
        astra_camera.py            # wrapper OpenNI camera (depth + color)
      camera_publisher.py          # node publish color/depth topic
      get_control.py               # node tương tác CAN -> publish control
      tracking.py                  # node nhận ảnh, phát hiện, hiển thị
      PostProcess.py               # hậu xử lý + OCSort + onnx runtime
      ocsort/                      # implementation OCSort (tracker)
      launch/bramy.launch.py       # launch file cho 4 node chính
      package.xml, setup.py, ...   # ROS2 package metadata
README.md (gốc)
LICENSE
.git/
```

---

## Cách build & chạy (trên Raspberry Pi 5)
> Giả sử bạn đã cài ROS2 (jazzy hoặc release tương ứng), `colcon` và đã thiết lập workspace.

1. Mở terminal, chuyển tới workspace (nơi chứa `rclpy`):
```bash
# ví dụ workspace là ~/ros2_ws
cd ~/ros2_ws
# nếu repo ở src/
# cp/unzip project vào src/ nếu cần
colcon build
# source environment
source /opt/ros/jazzy/setup.bash          # (hoặc release ROS2 bạn dùng)
source ./install/setup.bash
```

2. Thiết lập CAN interface (nếu dùng socketcan, ví dụ bitrate 500000):
```bash
# ví dụ dùng vcan0 cho test (no hardware)
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# nếu dùng USB-CAN hoặc MCP2515, thay 'can0' và cài driver tương ứng:
# sudo ip link set can0 up type can bitrate 500000
```

3. Bật camera / OpenNI driver
- Đảm bảo OpenNI2 libs (và driver Orbbec nếu cần) nằm ở thư mục hệ thống hoặc cập nhật `openni_libs` trong `astra_camera.py` nếu bạn gỡ thủ công libs.

4. Chạy toàn bộ nodes bằng launch file:
```bash
ros2 launch bramy bramy.launch.py
```

Hoặc chạy từng node (để debug):
```bash
ros2 run bramy camera_publisher
ros2 run bramy tracking
ros2 run bramy postProcess
ros2 run bramy get_control
```

---

## Topics & Messages (tóm tắt)
- `color_image` (sensor_msgs/Image) — ảnh màu RGB từ camera
- `depth_image` (sensor_msgs/Image) — ảnh depth (mono16)
- `get_control` node tương tác CAN trực tiếp; nội bộ node có logic để giới hạn speed/angle; bạn cần mapping lệnh CAN phù hợp với motor controller.

---

## Troubleshooting (vấn đề thường gặp)
- **Không có DISPLAY (cv2.imshow lỗi):** trên Pi headless, cv2.imshow sẽ lỗi — sử dụng stream hoặc lưu ảnh thay vì hiển thị.
- **Camera không kết nối / OpenNI error:** kiểm tra driver OpenNI, quyền truy cập USB, và đường dẫn `openni_libs`.
- **CAN error / socketcan not found:** đảm bảo kernel module và driver cho CAN adapter đã load, quyền truy cập `can0`. Dùng `ip link show` để kiểm tra.
- **Mất model (.onnx/.pt):** nếu startup báo không tìm mô hình, cần thêm file mô hình vào repo (hoặc script tải xuống). Kiểm tra folder `rclpy/car/resource/` để biết có chứa model hay chưa.
- **Hiệu năng trên Pi:** ONNXRuntime + PyTorch có thể nặng trên Pi; cân nhắc tối ưu hóa model (quantize ONNX) hoặc dùng phiên bản rút gọn.

---

## Gợi ý cải tiến / roadmap
- Đưa file model (onnx) vào `resource/` hoặc script `download_model.sh` để tải tự động.
- Thêm `requirements.txt` / `venv` instructions rõ ràng.
- Thay `cv2.imshow` sang web streaming hoặc ROS image_transport để chạy headless.
- Thêm node selector target (chọn người cần bám theo giữa nhiều người, bằng khoảng cách hoặc ID).
- Thêm test / CI đơn giản (unit tests cho utils).
- Viết hướng dẫn wiring chi tiết cho CAN → motor driver (sơ đồ, chân kết nối).

---

## License & Credits
- Kiểm tra file `LICENSE` trong repo để xác định license (nếu chưa có, hãy thêm).
- Một số thư viện tích hợp: OCSort (bộ theo dõi), OpenNI (camera), ONNXRuntime / PyTorch.

---

## Liên hệ / hỗ trợ
Nếu cần hỗ trợ triển khai trên Raspberry Pi 5 hoặc debug CAN / camera, vui lòng liên hệ qua GitHub Issues hoặc email nhóm phát triển.
