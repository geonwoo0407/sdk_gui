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

        # ROI 설정 (True: 설정, False: 설정 해제)
        self.use_roi = True 
        ##########################################################

    def color_image_callback(self, msg):
        # ROS 이미지를 OpenCV 이미지로 변환 (원본 해상도 유지)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        draw_image = cv_image.copy() # 전체 화면
        
        #이미지의 실제 원본 해상도 가져오기
        orig_height, orig_width = cv_image.shape[:2]
        
        # ROI 모드에 따른 처리 이미지 결정 및 가변 ROI 계산
        if self.use_roi:
            # 비율 단위로 가변 ROI 영역 계산
            self.roi_y_start = int(orig_height * 0.15) 
            self.roi_y_end = int(orig_height * 0.90)   
            self.roi_x_start = int(orig_width * 0.30) 
            self.roi_x_end = int(orig_width * 0.70)   

            # 원본 기억하기
            processing_image = cv_image[self.roi_y_start:self.roi_y_end, 
                                        self.roi_x_start:self.roi_x_end]
            offset_x = self.roi_x_start
            offset_y = self.roi_y_start
        else:
            processing_image = cv_image
            offset_x = 0
            offset_y = 0

        # 영상 전처리
        gray_image = cv2.cvtColor(processing_image, cv2.COLOR_BGR2GRAY)
        
        # 흰색 테이프 인식을 위한 임계값 설정 (흑백으로 이진화)
        ret, thresh_cv = cv2.threshold(gray_image, 140, 255, cv2.THRESH_BINARY)
        
        # 모폴로지를 이용하여 미세한 점 노이즈 제거
        kernel = np.ones((3, 3), np.uint8)
        thresh_cv = cv2.morphologyEx(thresh_cv, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # 컨투어 이용하여 윤곽선 찾기
        contours, _ = cv2.findContours(thresh_cv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        for certain_contour in contours:
            area = cv2.contourArea(certain_contour)
            if area < 500 or area > 10000:
                continue

            # 장단비(Aspect Ratio) 필터링
            rect = cv2.minAreaRect(certain_contour)
            (x, y), (w, h), angle = rect
            if w > 0 and h > 0:
                aspect_ratio = max(w, h) / min(w, h)
                # 대회용 테이프 비율이 보통 1:2~3이므로 1.3~4.0 사이로 설정
                if aspect_ratio < 1.3 or aspect_ratio > 4.0:
                    continue
            else: continue

            # 근사화 (꼭짓점 4개 확인)
            peri = cv2.arcLength(certain_contour, True)
            approx = cv2.approxPolyDP(certain_contour, 0.08 * peri, True)
            if len(approx) != 4: continue

            # 사각형 꼭짓점 가져오기 (ROI 기준 좌표에서)
            box = cv2.boxPoints(rect)
            box = np.intp(box)

            # y좌표 기준 정렬 (상단/하단 점 분리)
            sorted_by_y = sorted(box, key=lambda p: p[1])
            
            # ROI 좌표에 오프셋(offset_x, offset_y)을 더해 원본 좌표로 복구
            top_mid = (int((sorted_by_y[0][0] + sorted_by_y[1][0]) / 2) + offset_x, 
                       int((sorted_by_y[0][1] + sorted_by_y[1][1]) / 2) + offset_y)
            bottom_mid = (int((sorted_by_y[2][0] + sorted_by_y[3][0]) / 2) + offset_x, 
                          int((sorted_by_y[2][1] + sorted_by_y[3][1]) / 2) + offset_y)

            # 라인 중심점
            center_x = int((top_mid[0] + bottom_mid[0]) / 2)
            center_y = int((top_mid[1] + bottom_mid[1]) / 2)
            center_pt = (center_x, center_y)

            # 라인 자체의 각도 (수직선 기준)
            dx_line = top_mid[0] - bottom_mid[0]
            dy_line = bottom_mid[1] - top_mid[1] 
            line_angle_deg = math.degrees(math.atan2(dx_line, dy_line)) 

            # 원본 해상도(orig_width, orig_height) 기준으로 로봇 위치 계산
            robot_pos = (int(orig_width / 2), orig_height)
            dx_vec1 = center_x - robot_pos[0]
            dy_vec1 = robot_pos[1] - center_y
            vec1_angle_deg = math.degrees(math.atan2(dx_vec1, dy_vec1))

            # 시각화, 모든 꼭짓점 좌표를 원본 좌표로 변환해서 그리기
            box_original = box + [offset_x, offset_y]
            cv2.drawContours(draw_image, [box_original], 0, (0, 255, 0), 2)
            cv2.circle(draw_image, center_pt, 5, (0, 255, 255), -1)
            cv2.line(draw_image, bottom_mid, top_mid, (0, 255, 255), 2) 
            cv2.line(draw_image, robot_pos, center_pt, (255, 0, 255), 1) 

            # 각도 텍스트 출력
            cv2.putText(draw_image, f"Angle: {int(line_angle_deg)}", (center_x + 10, center_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        # ROI 영역을 화면에 표시 (파란색 사각형)
        if self.use_roi:
            cv2.rectangle(draw_image, (self.roi_x_start, self.roi_y_start), 
                          (self.roi_x_end, self.roi_y_end), (255, 0, 0), 2)
            # 텍스트 정보 출력
            info_text = f"DYNAMIC ROI ({orig_width}x{orig_height})"
            cv2.putText(draw_image, info_text, (self.roi_x_start, self.roi_y_start - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # 결과 출력
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