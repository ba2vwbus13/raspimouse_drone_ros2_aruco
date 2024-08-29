#!/home/nakahira/work/yolov8/bin/python
import cv2
import numpy as np
from ultralytics import YOLO
import rclpy
from .drone import DroneControl
#import drone
#!/usr/bin/env python3
def main(args=None):
    # ウェブカメラのキャプチャを開始
    video_path = 0
    cap = cv2.VideoCapture(video_path)
    img_size = np.array([cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)])
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) 
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))  
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    writer = cv2.VideoWriter('video1.m4v', fourcc, fps, (w, h))  
    # Load a pretrained YOLOv8n model
    model_path = "/home/nakahira/ros2_ws/src/raspimouse_drone_ros2/pinpon.pt"
   # model_path = '/home/nakahira/work/yolov8n.pt'
    model = YOLO(model_path)

    rclpy.init()
    #drone_obj = drone.DroneControl(img_size)
    drone_obj = DroneControl(img_size) 
    rate = drone_obj.create_rate(10)
    # キャプチャがオープンしている間続ける
    while(cap.isOpened() and rclpy.ok()):
        # フレームを読み込む
        ret, frame = cap.read()
        if ret == True:
            # フレームを表示
            detections = model(frame)
            #frame = detection.plot()
            detection = detections[0]
            drone_obj.getDronePoint(detection)
            ret = drone_obj.getNextTarget(frame)
            drone_obj.getMovingDerection(ret)
            frame = detection.plot()
            frame = drone_obj.OverImage(frame)
            drone_obj.sendCommand()
            cv2.imshow('Webcam Live', frame)
            writer.write(frame)
            rclpy.spin_once(drone_obj)
            # 'q'キーが押されたらループから抜ける
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
                break

    # キャプチャをリリースし、ウィンドウを閉じる
    writer.release()
    cap.release()
    cv2.destroyAllWindows()
    # ros node破棄
    rclpy.shutdown()

if __name__ == '__main__':
    main()
