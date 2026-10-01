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
        
        # 로봇의 현재 위치 (화면 맨 아래 정중앙)
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
        
        # 인식된 테이프들의 중심점을 모음
        valid_centers = []

        for certain_contour in contours:
            area = cv2.contourArea(certain_contour)
            if area < 500 or area > 10000:
                continue

            rect = cv2.minAreaRect(certain_contour)
            (x, y), (w, h), angle = rect
            if w > 0 and h > 0:
                aspect_ratio = max(w, h) / min(w, h)
                if aspect_ratio < 1.3 or aspect_ratio > 4.0:
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

            # 리스트에 현재 테이프의 중심점 저장
            valid_centers.append(center_pt)

            # 기존 시각화
            box_original = box + [offset_x, offset_y]
            cv2.drawContours(draw_image, [box_original], 0, (0, 255, 0), 2)
            cv2.circle(draw_image, center_pt, 4, (0, 255, 255), -1)

        # 최종 전진 방향 계산 및 시각화
        if valid_centers:
            # 인식된 모든 테이프 중심점들의 평균 좌표 구하기
            avg_x = int(np.mean([pt[0] for pt in valid_centers]))
            avg_y = int(np.mean([pt[1] for pt in valid_centers]))
            target_pt = (avg_x, avg_y)

            # 로봇 위치에서 목표점까지의 벡터 각도 계산
            dx_steer = target_pt[0] - robot_pos[0]
            dy_steer = robot_pos[1] - target_pt[1] # y축은 위로 갈수록 작아지므로 반전
            
            steering_angle = math.degrees(math.atan2(dx_steer, dy_steer))

            # 굵은 빨간색 선으로 가야 할 방향 그리기
            cv2.line(draw_image, robot_pos, target_pt, (0, 0, 255), 4) # 굵기 4
            cv2.circle(draw_image, target_pt, 8, (0, 0, 255), -1)      # 목표점 강조

            # 직관적인 텍스트 및 조향 방향 출력
            text_color = (0, 0, 255)
            cv2.putText(draw_image, f"Steer Angle: {int(steering_angle)} deg", (30, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)
            
            # 각도에 따른 방향 판별 (임계값 7도로 가정)
            if steering_angle > 7:
                direction_cmd = ">> TURN RIGHT >>"
            elif steering_angle < -7:
                direction_cmd = "<< TURN LEFT <<"
            else:
                direction_cmd = "^^ GO STRAIGHT ^^"
                
            cv2.putText(draw_image, direction_cmd, (30, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

        # 파란색 ROI 사각형 표시
        if self.use_roi:
            cv2.rectangle(draw_image, (self.roi_x_start, self.roi_y_start), 
                          (self.roi_x_end, self.roi_y_end), (255, 0, 0), 2)

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