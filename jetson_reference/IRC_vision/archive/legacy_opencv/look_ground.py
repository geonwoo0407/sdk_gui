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
            '/camera/camera/color/image_raw',  # 컬러 이미지 토픽
            self.color_image_callback, 10)        
        self.bridge = CvBridge()

    def color_image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 그레이스케일 변환 및 이진화
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret, thresh_cv = cv2.threshold(gray_image, 160, 255, cv2.THRESH_BINARY)
        
        # 모폴로지 연산으로 미세한 점 노이즈 제거
        # 침식 후 팽창을 통해 작은 흰색 점들을 삭제
        kernel = np.ones((3, 3), np.uint8)
        thresh_cv = cv2.morphologyEx(thresh_cv, cv2.MORPH_OPEN, kernel, iterations=1)
        
        draw_image = cv_image.copy()
        contours, _ = cv2.findContours(thresh_cv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        for certain_contour in contours:
            # 면적 필터링
            area = cv2.contourArea(certain_contour)
            if area < 300 or area > 2000:
                continue

            # 장단비(Aspect Ratio) 필터링
            # 테이프는 일정한 비율의 직사각형이어서 찌그러진 모양을 제거.
            rect = cv2.minAreaRect(certain_contour)
            (x, y), (w, h), angle = rect
            if w > 0 and h > 0:
                aspect_ratio = max(w, h) / min(w, h)
                # 대회용 테이프 비율이 보통 1:2~3이므로 1.3~4.0 사이로 설정
                if aspect_ratio < 1.3 or aspect_ratio > 4.0:
                    continue
            else:
                continue

            # 근사화 (꼭짓점 개수 확인)
            peri = cv2.arcLength(certain_contour, True)
            approx = cv2.approxPolyDP(certain_contour, 0.08 * peri, True)
            if len(approx) != 4:
                continue

            box = cv2.boxPoints(rect)
            box = np.intp(box)

            # 라인의 중심점 및 각도 계산
            # y좌표 기준으로 상단점과 하단점을 분리
            sorted_by_y = sorted(box, key=lambda p: p[1])
            top_points = sorted_by_y[:2]
            bottom_points = sorted_by_y[2:]

            top_mid = (int((top_points[0][0] + top_points[1][0]) / 2), int((top_points[0][1] + top_points[1][1]) / 2))
            bottom_mid = (int((bottom_points[0][0] + bottom_points[1][0]) / 2), int((bottom_points[0][1] + bottom_points[1][1]) / 2))

            center_x = int((top_mid[0] + bottom_mid[0]) / 2)
            center_y = int((top_mid[1] + bottom_mid[1]) / 2)
            center_pt = (center_x, center_y)

            # 라인의 각도
            dx_line = top_mid[0] - bottom_mid[0]
            dy_line = bottom_mid[1] - top_mid[1] 
            line_angle_deg = math.degrees(math.atan2(dx_line, dy_line)) 

            # 로봇 위치(화면 하단 중앙) 기준 벡터 각도
            height, width = cv_image.shape[:2]
            robot_pos = (int(width / 2), height)
            
            dx_vec1 = center_x - robot_pos[0]
            dy_vec1 = robot_pos[1] - center_y
            vec1_angle_deg = math.degrees(math.atan2(dx_vec1, dy_vec1))

            # 시각화
            cv2.drawContours(draw_image, [box], 0, (0, 255, 0), 2)
            cv2.circle(draw_image, center_pt, 5, (0, 255, 255), -1)
            cv2.circle(draw_image, robot_pos, 7, (255, 0, 255), -1)
            cv2.line(draw_image, bottom_mid, top_mid, (0, 255, 255), 2) 
            cv2.line(draw_image, robot_pos, center_pt, (255, 0, 255), 1) 

            text_x = center_x + 15
            cv2.putText(draw_image, f"Center: {center_x}, {center_y}", (text_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            cv2.putText(draw_image, f"Line Angle: {int(line_angle_deg)}", (text_x, center_y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            cv2.putText(draw_image, f"Vec1 Angle: {int(vec1_angle_deg)}", (text_x, center_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

        cv2.imshow("Binary Threshold (Morphology applied)", thresh_cv)
        cv2.imshow("Detected Tape Centers", draw_image)
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