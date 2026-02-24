#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
import glob
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
from tf.transformations import euler_from_quaternion

# ==========================================
# 配置参数
# ==========================================
SAVE_DIR = os.path.expanduser('~/airhust_tutorial_ws/mission_results')
TARGET_COUNT = 3     
MIN_HEIGHT = 1.0     
MAX_HEIGHT = 2.0     
CENTER_d = 250       
SCORE_THRESH = 0.55  
CAMERA_SCALE = 1.67  
MERGE_DIST = 1.0     

# 【新增】边缘安全距离 (像素)
# 如果卡片离画面边缘小于这个值，说明可能没拍全，直接丢弃！
BORDER_MARGIN = 20   

# ==========================================
# 算法类
# ==========================================
def resize_and_pad(image, target_size=64):
    h, w = image.shape[:2]
    scale = target_size / max(h, w)
    new_w = int(w * scale)
    new_h = int(h * scale)
    resized = cv2.resize(image, (new_w, new_h))
    final_img = np.zeros((target_size, target_size), dtype=np.uint8)
    x_offset = (target_size - new_w) // 2
    y_offset = (target_size - new_h) // 2
    final_img[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
    return final_img

def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

def crop_char_roi(image_bin):
    contours, _ = cv2.findContours(image_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None
    c = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    return image_bin[y:y+h, x:x+w]

class CharacterRecognizer:
    def __init__(self, template_dir):
        self.templates = {}
        rospy.loginfo(f"正在加载模板: {template_dir}")
        files = glob.glob(os.path.join(template_dir, "*.png"))
        for p in files:
            label = os.path.basename(p)[0]
            img = cv2.imread(p, 0)
            if img is not None:
                if img[0,0]>127: img=cv2.bitwise_not(img)
                _, bin_img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
                roi = crop_char_roi(bin_img)
                if roi is not None:
                    self.templates[label] = resize_and_pad(roi, 64)
        rospy.loginfo(f"加载了 {len(self.templates)} 个模板")

    def match_single_template(self, target_norm, template_norm):
        best_score = -1
        current = target_norm.copy()
        for _ in range(4):
            res = cv2.matchTemplate(current, template_norm, cv2.TM_CCOEFF_NORMED)
            if res.max() > best_score: best_score = res.max()
            current = cv2.rotate(current, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return best_score

    def identify_contour(self, gray, contour):
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        if len(approx) != 4: return None
        try:
            pts = approx.reshape(4, 2)
            M = cv2.getPerspectiveTransform(order_points(pts), 
                np.array([[0,0],[199,0],[199,199],[0,199]], dtype="float32"))
            warped = cv2.warpPerspective(gray, M, (200, 200))
        except: return None
        
        _, wb = cv2.threshold(warped, 127, 255, cv2.THRESH_BINARY_INV)
        troi = crop_char_roi(wb)
        if troi is None: return None
        
        tnorm = resize_and_pad(troi, 64)
        best_lbl, final_score = "?", 0
        
        for l, t in self.templates.items():
            s = self.match_single_template(tnorm, t)
            if s > final_score: 
                final_score = s
                best_lbl = l
        return best_lbl, final_score

# ==========================================
# 自动任务节点
# ==========================================
class AutoMissionNode:
    def __init__(self):
        rospy.init_node("auto_mission_strict", anonymous=True)
        if not os.path.exists(SAVE_DIR): os.makedirs(SAVE_DIR)
        
        home = os.path.expanduser('~')
        temp_path = os.path.join(home, "airhust_tutorial_ws/src/template/src/dataset_bold")
        self.recognizer = CharacterRecognizer(temp_path)
        self.bridge = CvBridge()

        self.drone_pos = [0,0,0] 
        self.drone_yaw = 0
        self.found_targets = {} 
        self.has_taken_off = False
        self.mission_finished = False
        
        self.sub_img = rospy.Subscriber("/image_raw", Image, self.img_cb) 
        self.sub_pose = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        self.pub_target = rospy.Publisher("/target", String, queue_size=10)


    def pose_cb(self, msg):
        self.drone_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.drone_yaw = yaw

        curr_h = self.drone_pos[2]
        if not self.has_taken_off and curr_h > 1.0:
            self.has_taken_off = True
            rospy.loginfo("检测到起飞")

        if self.has_taken_off and curr_h < 0.3 and not self.mission_finished:
            rospy.loginfo("检测到降落，准备结算...")
            self.finish_mission()

    def calculate_world_pos(self, u, v, img_w, img_h):
        curr_z = self.drone_pos[2]
        if curr_z < 0.1: return 0, 0 
        norm_x = (u - img_w / 2) / img_w
        norm_y = (v - img_h / 2) / img_h
        dist_x_body = -norm_y * curr_z * CAMERA_SCALE
        dist_y_body = -norm_x * curr_z * CAMERA_SCALE 
        dx_world = dist_x_body * math.cos(self.drone_yaw) - dist_y_body * math.sin(self.drone_yaw)
        dy_world = dist_x_body * math.sin(self.drone_yaw) + dist_y_body * math.cos(self.drone_yaw)
        return self.drone_pos[0] + dx_world, self.drone_pos[1] + dy_world

    def img_cb(self, msg):
        if self.mission_finished: return
        if self.drone_pos[2] < MIN_HEIGHT or self.drone_pos[2] > MAX_HEIGHT: return 

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        center_x, center_y = w // 2, h // 2
        
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) < 1000: continue
            
            x, y, bw, bh = cv2.boundingRect(cnt)
            obj_cx, obj_cy = x + bw//2, y + bh//2

            # --- 过滤 1: 中心距离 ---
            if math.sqrt((obj_cx - center_x)**2 + (obj_cy - center_y)**2) > CENTER_d: continue 

            # --- 过滤 2 (核心): 边缘检测 ---
            # 如果方框的任何一边挨着画面边缘，说明没拍全，直接扔掉！
            if x < BORDER_MARGIN or y < BORDER_MARGIN or \
               (x + bw) > (w - BORDER_MARGIN) or (y + bh) > (h - BORDER_MARGIN):
                # rospy.loginfo_throttle(1, "目标不完整(切边)，已忽略")
                continue

            # --- 过滤 3: 长宽比 (卡片应该是正方形) ---
            aspect_ratio = float(bw) / bh
            if aspect_ratio < 0.7 or aspect_ratio > 1.3:
                # 形状太长条，说明侧视角太严重，也扔掉
                continue

            label = None
            score = 0.0

            roi = gray[max(0,y):min(h,y+bh), max(0,x):min(w,x+bw)]
            qrs = decode(roi)
            if qrs:
                label = "QR_" + qrs[0].data.decode('utf-8')
                score = 0.99
            else:
                res = self.recognizer.identify_contour(gray, cnt)
                if res:
                    temp_lbl, temp_score = res
                    if temp_score > SCORE_THRESH: 
                        label, score = temp_lbl, temp_score

            if label:
                wx, wy = self.calculate_world_pos(obj_cx, obj_cy, w, h)
                
                if (label not in self.found_targets) or (score > self.found_targets[label]['score']):
                    save_img = frame.copy()
                    cv2.rectangle(save_img, (x,y), (x+bw, y+bh), (0,255,0), 2)
                    info_txt = f"{label}:{score:.2f}"
                    cv2.putText(save_img, info_txt, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                    self.found_targets[label] = {
                        'label': label,
                        'score': score,
                        'pos': (wx, wy),
                        'img': save_img
                    }
                    rospy.loginfo(f"捕获 [{label}] Score:{score:.2f} (完整度校验通过)")

    def finish_mission(self):
        self.mission_finished = True
        rospy.loginfo("========== 坐标去重 (Spatial NMS) ==========")
        candidates = list(self.found_targets.values())
        candidates.sort(key=lambda x: x['score'], reverse=True)
        unique_targets = []
        
        for cand in candidates:
            cand_x, cand_y = cand['pos']
            is_new_location = True
            for exist in unique_targets:
                exist_x, exist_y = exist['pos']
                if math.sqrt((cand_x - exist_x)**2 + (cand_y - exist_y)**2) < MERGE_DIST:
                    is_new_location = False
                    break
            if is_new_location:
                unique_targets.append(cand)

        rospy.loginfo(f"最终结果: {len(unique_targets)}")
        count = 0
        for data in unique_targets:
            if count >= TARGET_COUNT: break
            label = data['label']
            wx, wy = data['pos']
            
            msg_str = f"{label},{wx:.2f},{wy:.2f}"
            self.pub_target.publish(msg_str)
            rospy.loginfo(f">>> 发布: {msg_str}")
            rospy.sleep(0.2)
            
            filename = f"{SAVE_DIR}/Result_{label}.jpg"
            cv2.imwrite(filename, data['img'])
            with open(f"{SAVE_DIR}/Coordinate_{label}.txt", "w") as f:
                f.write(f"Target: {label}\n")
                f.write(f"World X: {wx:.4f}\n")
                f.write(f"World Y: {wy:.4f}\n")
                f.write(f"Score: {data['score']:.2f}\n")
            count += 1
            
        rospy.loginfo("========== 完成 ==========")
        rospy.sleep(1.0)
        rospy.signal_shutdown("Done")

if __name__ == '__main__':
    node = AutoMissionNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass