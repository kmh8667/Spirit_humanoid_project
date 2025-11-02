경량화 버전
터미널 1: 카메라
     ros2 launch usb_cam camera.launch.py

     터미널 2: YOLO 감지
     ros2 launch yolo_bringup yolov11.launch.py

     터미널 3: 경량화 AI 에이전트
     conda activate emotion_robot_vlm
     ros2 run emotion_robot light_multimodal_agent --model gemini
rviz2
