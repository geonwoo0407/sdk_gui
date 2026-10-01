#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class ImgSubscriber(Node):
    def __init__(self):
        super().__init__('img_subscriber')
        self.subscription_color = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  
            self.color_image_callback, 10)        
        self.bridge = CvBridge()

        self.use_roi = True 

    def color_image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        draw_image = cv_image.copy() 
        orig_height, orig_width = cv_image.shape[:2]
        
        robot_pos = (int(orig_width / 2), orig_height)
        
        if self.use_roi:
            self.roi_y_start = int(orig_height * 0.15) 
            self.roi_y_end = int(orig_height * 0.90)   
            self.roi_x_start = int(orig_width * 0.30) 
            self.roi_x_end = int(orig_width * 0.70)   

            processing_image = cv_image[self.roi_y_start:self.roi_y_end, 
                                        self.roi_x_start:self.roi_x_end]
            offset_x = self.roi_x_start
            offset_y = self.roi_y_start
        else:
            processing_image = cv_image
            offset_x = 0
            offset_y = 0

        gray_image = cv2.cvtColor(processing_image, cv2.COLOR_BGR2GRAY)
        ret, thresh_cv = cv2.threshold(gray_image, 160, 255, cv2.THRESH_BINARY)
        
        kernel = np.ones((3, 3), np.uint8)
        thresh_cv = cv2.morphologyEx(thresh_cv, cv2.MORPH_OPEN, kernel, iterations=1)
        
        contours, _ = cv2.findContours(thresh_cv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        valid_centers = []

        for certain_contour in contours:
            area = cv2.contourArea(certain_contour)
            if area < 500 or area > 10000:
                continue

            rect = cv2.minAreaRect(certain_contour)
            (x, y), (w, h), angle = rect
            if w > 0 and h > 0:
                aspect_ratio = max(w, h) / min(w, h)
                if aspect_ratio < 1.3 or aspect_ratio > 4.5:
                    continue
            else: continue

            peri = cv2.arcLength(certain_contour, True)
            approx = cv2.approxPolyDP(certain_contour, 0.08 * peri, True)
            if len(approx) != 4: continue

            box = cv2.boxPoints(rect)
            box = np.intp(box)

            sorted_by_y = sorted(box, key=lambda p: p[1])
            
            top_mid = (int((sorted_by_y[0][0] + sorted_by_y[1][0]) / 2) + offset_x, 
                       int((sorted_by_y[0][1] + sorted_by_y[1][1]) / 2) + offset_y)
            bottom_mid = (int((sorted_by_y[2][0] + sorted_by_y[3][0]) / 2) + offset_x, 
                          int((sorted_by_y[2][1] + sorted_by_y[3][1]) / 2) + offset_y)

            center_x = int((top_mid[0] + bottom_mid[0]) / 2)
            center_y = int((top_mid[1] + bottom_mid[1]) / 2)
            center_pt = (center_x, center_y)

            valid_centers.append(center_pt)

            box_original = box + [offset_x, offset_y]
            cv2.drawContours(draw_image, [box_original], 0, (0, 255, 0), 2)
            cv2.circle(draw_image, center_pt, 4, (0, 255, 255), -1) # 노란색 점

        #  흰테이프 중점끼리 징검다리 선 이어서 상대 각도 제어
        if len(valid_centers) >= 2: 
            # 로봇과 가장 가까운 순서대로 모든 점 정렬
            sorted_by_near_to_far = sorted(valid_centers, key=lambda p: p[1], reverse=True)
            
            path_color = (255, 100, 0) # 파란색 계열
            
            # 모든 테이프 중점을 순서대로 연결
            for i in range(len(sorted_by_near_to_far) - 1):
                p_start = sorted_by_near_to_far[i]
                p_end = sorted_by_near_to_far[i+1]
                cv2.line(draw_image, p_start, p_end, path_color, 2) 
                
                cv2.circle(draw_image, p_start, 6, path_color, -1)
                cv2.circle(draw_image, p_end, 6, path_color, -1)

            # 각도 구하기
            y_coords = [pt[1] for pt in valid_centers]
            x_coords = [pt[0] for pt in valid_centers]
            fit = np.polyfit(y_coords, x_coords, 1)
            m = fit[0] 
            c = fit[1] 

            target_y = int(orig_height * 0.6)
            target_x = int(m * target_y + c)
            target_pt = (target_x, target_y)

            # 빨간 선의 절대 각도 계산
            dx_steer = target_pt[0] - robot_pos[0]
            dy_steer = robot_pos[1] - target_pt[1]
            steer_abs_angle = math.degrees(math.atan2(dx_steer, dy_steer))

            # 가장 가까운 두 점으로 현재 트랙 방향 각도 구하기
            p1_closest = sorted_by_near_to_far[0] 
            p2_closest = sorted_by_near_to_far[1] 

            dx_path = p2_closest[0] - p1_closest[0]
            dy_path = p1_closest[1] - p2_closest[1] 
            path_abs_angle = math.degrees(math.atan2(dx_path, dy_path))

            # 최종 조향 오차 계산
            steering_error = path_abs_angle - steer_abs_angle

            cv2.line(draw_image, robot_pos, target_pt, (0, 0, 255), 4) 
            cv2.circle(draw_image, target_pt, 8, (0, 0, 255), -1)      

            # 텍스트 출력
            text_color = (0, 0, 255)
            cv2.putText(draw_image, f"Steer Error: {int(steering_error)} deg", (30, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)
            
            # 오차에 따른 방향 판별 (임계값 5도)
            if steering_error > 5:
                direction_cmd = ">> TURN RIGHT >>"
            elif steering_error < -5:
                direction_cmd = "<< TURN LEFT <<"
            else:
                direction_cmd = "^^ GO STRAIGHT ^^"
                
            cv2.putText(draw_image, direction_cmd, (30, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

        # 파란색 ROI 사각형 표시
        if self.use_roi:
            cv2.rectangle(draw_image, (self.roi_x_start, self.roi_y_start), 
                          (self.roi_x_end, self.roi_y_end), (255, 0, 0), 2)

        # 결과 이미지 화면에 띄우기
        cv2.imshow("Binary Threshold (ROI)", thresh_cv)
        cv2.imshow("Tape Center Detection", draw_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ImgSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()