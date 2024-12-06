import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import Counter
from geometry_msgs.msg import Twist

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        self.subscription = self.create_subscription(
            Image,
            '/simple_rover/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription 

        self.bridge = CvBridge()

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.slow_down = False  # 속도를 줄일지 여부를 추적하는 플래그


    def image_callback(self, msg):
        cv_imag = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = cv_imag.shape
        cv_image = cv_imag[2 * height // 3:, :]
        right_image = cv_imag[7 * height // 9:, 4 * width // 5:]

        # 그레이스케일로 변환
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        right_gray_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        # 이진화 (검정색 라인 추출)
        _, thresholded = cv2.threshold(gray_image, 100, 255, cv2.THRESH_BINARY_INV)
        _, right_thresholded = cv2.threshold(right_gray_image, 100, 255, cv2.THRESH_BINARY_INV)

        black_pixel_ratio_right = np.sum(right_thresholded >= 200) / (right_thresholded.size)

        # 라인 검출 (컨투어 찾기)
        contours, _ = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()
        
        if contours:
            # 가장 큰 컨투어(라인) 찾기
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            # 라인의 중심 계산
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                center_offset = -30  # 라인의 중심에서 20픽셀 오른쪽으로 이동
                target_position = cv_image.shape[1] // 2 + center_offset
                error = cx - target_position 
                
                # 이미지의 중앙과 라인의 중심을 비교
                # error = cx - cv_image.shape[1] // 2
                twist.angular.z = -float(error) /92.0  # 중심에 맞춰 회전
                twist.linear.x = 0.7  # 기본 속도

                if black_pixel_ratio_right >0.01:
                    # self.get_logger().info("Right line detected!")
                    # twist.angular.z = -2.0
                    twist.linear.x = 0.1  # 속도 줄이기



            # 명령을 로봇에 전달
            self.cmd_vel_pub.publish(twist)

        # 디버그용 이미지 출력
        # cv2.imshow("Line Detection", thresholded)
        # cv2.imshow("right_image", right_image)
        # cv2.waitKey(1)

        # # 윗부분을 제외한 관심 영역(ROI) 설정
        # height, width, _ = cv_image.shape
        # roi = cv_image[2 * height // 3:, :]  # 이미지의 하단 절반만 사용

        # filtered_pixels = [
        #     tuple(pixel) for pixel in roi.reshape((-1, 3))
        #     if not all(100 < channel < 200 for channel in pixel)
        # ]

        # # 필터링된 픽셀 카운트
        # color_counter = Counter(filtered_pixels)
        # if color_counter:  # 필터링 후 남은 값이 있을 경우
        #     dominant_color, count = color_counter.most_common(1)[0]
        #     self.get_logger().info(f"Dominant color: {dominant_color}, Count: {count}")
        # else:
        #     self.get_logger().info("No valid colors found after filtering.")

        # cv2.imshow('Camera Image', roi)
        # cv2.waitKey(1)
        
        
def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()

    try:
        rclpy.spin(line_follower)
    except KeyboardInterrupt:
        pass
    finally:
        line_follower.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
