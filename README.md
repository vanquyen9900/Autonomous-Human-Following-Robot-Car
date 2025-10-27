# Autonomous Human-Following Robot Car (Raspberry Pi 5)

**Mục tiêu:** Xe/robot di động tự hành **bám theo người** (human-following) sử dụng Astra, mô hình phát hiện đối tượng (ONNX/PyTorch) và bộ theo dõi OCSort. Dự án xây dựng dưới dạng package ROS2 và hướng tới chạy trên **Raspberry Pi 5**.

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
- **Camera depth/color** hỗ trợ OpenNI (Astra camera)
- **Motor driver / motor controller** có giao tiếp CAN hoặc module điều khiển nhận lệnh từ CAN bus
- **CAN interface**: MCP2515 (SPI) hoặc USB-CAN adapter (tương thích socketcan)
- Nguồn cấp phù hợp cho Pi + motors

---

## Yêu cầu phần mềm (packages chính)
- ROS2 Jazz — hoặc ROS2 release tương thích trên Pi (cài đúng release bạn định dùng)
- Python packages:
  - `rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge` (ROS2)
  - `opencv-python`, `numpy`
  - `onnxruntime`, `torch` (nếu dùng PyTorch; ONNX runtime để chạy model .onnx)
  - `openni` (openni2 bindings) — driver camera Astra
  - `python-can` (để truy cập socketcan)
- Hệ thống: `colcon` để build ROS2 workspace

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
source /opt/ros/jazzy/setup.bash         
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
- Đảm bảo OpenNI2 libs nằm ở thư mục hệ thống hoặc cập nhật `openni_libs` trong `astra_camera.py` nếu bạn gỡ thủ công libs.

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
