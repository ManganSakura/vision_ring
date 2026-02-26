#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SimAutoLabeler:
    def __init__(self):
        rospy.init_node('sim_auto_labeler', anonymous=True)
        self.bridge = CvBridge()
        
        # 定义保存路径 (放在工作空间根目录下的 dataset 文件夹)
        self.save_dir = os.path.expanduser("/workspace/project_ws/dataset")
        self.img_dir = os.path.join(self.save_dir, "images")
        self.lbl_dir = os.path.join(self.save_dir, "labels")
        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.lbl_dir, exist_ok=True)

        self.img_count = 0
        self.save_interval = 10  # 每隔 10 帧保存一次，避免数据太重复
        self.frame_counter = 0

        # Gazebo/Blue 在 HSV 色彩空间的大致范围
        self.lower_blue = np.array([110, 50, 50])
        self.upper_blue = np.array([130, 255, 255])

        # 订阅无人机前置摄像头的话题 (请根据你的实际话题名称修改)
        self.image_sub = rospy.Subscriber("/camera_front/image_raw", Image, self.image_callback)
        rospy.loginfo("🔥 自动标注节点已启动！正在监听摄像头数据...")

    def image_callback(self, data):
        self.frame_counter += 1
        if self.frame_counter % self.save_interval != 0:
            return # 跳过不需要保存的帧

        try:
            # 1. 将 ROS 图像转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            h, w, _ = cv_image.shape

            # 2. 转换到 HSV 色彩空间并提取蓝色
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

            # 3. 寻找蓝色物体的轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # 找到包含所有蓝色轮廓的最大边界框 (门框的上下横梁)
                x_min = w; y_min = h; x_max = 0; y_max = 0
                for cnt in contours:
                    x, y, bw, bh = cv2.boundingRect(cnt)
                    # 过滤掉太小的噪点
                    if bw * bh > 500: 
                        x_min = min(x_min, x)
                        y_min = min(y_min, y)
                        x_max = max(x_max, x + bw)
                        y_max = max(y_max, y + bh)

                # 4. 如果找到了有效的门框
                if x_min < x_max and y_min < y_max:
                    # 将绝对坐标转换为 YOLO 格式 (中心点比例，宽比例，高比例)
                    box_w = x_max - x_min
                    box_h = y_max - y_min
                    x_center = (x_min + box_w / 2.0) / w
                    y_center = (y_min + box_h / 2.0) / h
                    norm_w = box_w / w
                    norm_h = box_h / h

                    # YOLO 类别 0 代表 gate
                    class_id = 0 
                    yolo_line = f"{class_id} {x_center:.6f} {y_center:.6f} {norm_w:.6f} {norm_h:.6f}\n"

                    # 5. 保存图片和标注文件
                    base_name = f"sim_gate_{self.img_count:05d}"
                    cv2.imwrite(os.path.join(self.img_dir, f"{base_name}.jpg"), cv_image)
                    with open(os.path.join(self.lbl_dir, f"{base_name}.txt"), "w") as f:
                        f.write(yolo_line)

                    self.img_count += 1
                    rospy.loginfo(f"✅ 已保存 {base_name}.jpg 及其标注，共 {self.img_count} 张")

                    # [可视化调试] 在画面上画出框框显示一下
                    cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            
            # 显示实时画面
            cv2.imshow("Auto Labeler View", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"处理图像时出错: {e}")

if __name__ == '__main__':
    try:
        AutoLabeler = SimAutoLabeler()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
