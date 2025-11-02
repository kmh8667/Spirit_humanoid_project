# 🤖 Emotion Robot - Multimodal AI System

ROS2 기반 감정 인식 대화형 로봇 (경량화 버전)

## 🛠️ 설치 방법

### 1. Conda 환경 생성
```bash
conda create -n vlm python=3.10
conda activate vlm
```

### 2. 패키지 설치
```bash
pip install -r requirements.txt
```

### 3. ROS2 빌드
```bash
colcon build
source install/setup.bash
```

### 4. 환경 변수 설정
```bash
export GOOGLE_API_KEY="your_gemini_api_key"
```

## 🚀 실행 방법

```bash
# 터미널 1: 카메라
ros2 launch usb_cam camera.launch.py

# 터미널 2: YOLO 얼굴 감지
ros2 launch yolo_bringup yolov11.launch.py

# 터미널 3: AI 에이전트
conda activate vlm
ros2 run emotion_robot light_multimodal_agent --model gemini

# 터미널 4: 시각화 (선택)
rviz2
```

## 📌 핵심 기능

- YOLO11n 얼굴 감지 및 추적
- Gemini VLM 멀티모달 대화
- Whisper STT + gTTS 음성 인식/합성
- 표정-발화 불일치 감지
- 환경 기반 감정 변화

## 🔧 시스템 요구사항

- Ubuntu 22.04 (ROS2 Humble)
- Python 3.10
- CUDA 12.x
- NVIDIA GPU (4GB+ VRAM)
