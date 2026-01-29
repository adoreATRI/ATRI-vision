import cv2
import time
import random
import math
import numpy as np

COLOR_BGR = {
    0: (116, 5, 202),
    3: (167, 1, 98),
    4: (7, 237, 19),
    5: (23, 51, 215),
    6: (241, 132, 251),
    7: (17, 168, 214),
    8: (135, 199, 246),
    9: (221, 66, 76),
    10: (216, 199, 167),
    11: (85, 152, 55),
    12: (57, 244, 231),
    16: (23, 3, 23),
}

class BuffSimulator:
    def __init__(self, 
                 mode: str = "small"):
        self.mode = mode

        self.angle = 0
        self.start_time = time.time()
        self.direction = 1

        # large mode
        self.a = random.uniform(0.780, 1.045)
        self.omega = random.uniform(1.884, 2.000)
        self.b = 2.090 - self.a

        # 色块颜色
        self.block_color_id = []
        self.target_color_id = -1
        self.randomize_colors()

        self.img_w = 1280
        self.img_h = 720

    # 运行
    def run(self):
        cv2.namedWindow("Buff Simulator", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Buff Simulator", 1280, 720)

        last_time = time.time()

        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            self.update(dt)

            img = self.render()

            self.draw_info(img)

            cv2.imshow("Buff Simulator", img)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:  # Q or ESC
                break
            elif key == ord('m'):  # Toggle mode
                self.mode = 'large' if self.mode == 'small' else 'small'
                print(f"Mode changed to: {self.mode}")
            elif key == ord('d'):  # Toggle direction
                self.direction *= -1
                print(f"Direction changed to: {'CW' if self.direction > 0 else 'CCW'}")
            elif key == ord('r'):  # Randomize colors
                self.randomize_colors()
                print(f"Colors randomized. Target: {self.target_color_id}")
            elif key == ord('p'):  # Reset large buff params
                self.reset_large_params()
                print(f"Large buff params reset: a={self.a:.3f}, ω={self.omega:.3f}, b={self.b:.3f}")

        
        cv2.destroyAllWindows()
        
    # 更新状态
    def update(self, dt: float):
        t = time.time() - self.start_time  
        angular_velocity = self.get_angular_velocity(t)
        self.angle += angular_velocity * dt * self.direction

        self.angle = self.angle % (2 * math.pi)
    
    def get_angular_velocity(self, t: float) -> float:
        if self.mode == 'small':
            return math.pi / 3.0 
        else:
            return self.a * math.sin(self.omega * t) + self.b

    # 渲染当前帧
    def render(self) -> np.ndarray:
        img = np.full((self.img_h, self.img_w, 3), 255, dtype=np.uint8)

        for i in range(5):
            self.draw_block(img, i, self.angle, self.block_color_id[i])

        self.draw_center_ring(img)

        return img

    def draw_info(self, img):
        info_lines = [
         f"Angular Vel: {self.get_angular_velocity(time.time() - self.start_time):.3f} rad/s",
         f"a: {self.a:.3f}, omega: {self.omega:.3f}, b: {self.b:.3f}" if self.mode == 'large' else "Small Buff Mode",
        ]

        y_offset = 30
        for line in info_lines:
            cv2.putText(img, line, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            y_offset += 20

    # 单个色块的绘制
    def draw_block(self, img, block_idx, angle, block_color_id):
        angle_diff = 2 * math.pi / 5
        block_angle = block_idx * angle_diff + angle

        cx = self.img_w // 2
        cy = self.img_h // 2
        center_x = cx + 204.8 * math.cos(block_angle)
        center_y = cy + 204.8 * math.sin(block_angle)

        corners = self.get_block_corners(center_x, center_y, block_angle, 102.4)

        color = COLOR_BGR[block_color_id]
        cv2.fillPoly(img, [corners], color)

    def draw_center_ring(self, img):
        center = (self.img_w // 2, self.img_h // 2)

        target_color = COLOR_BGR[self.target_color_id]
        cv2.circle(img, center, 51, target_color, -1)

        cv2.circle(img, center, 16, (255, 255, 255), -1)


    def get_block_corners(self, center_x, center_y, block_angle, side) -> np.ndarray:
        half_diagonal = side * math.sqrt(2) / 2

        corners = []
        for i in range(4):
            corner_angle = block_angle + i * math.pi / 2
            corner_x = center_x + half_diagonal * math.cos(corner_angle)
            corner_y = center_y + half_diagonal * math.sin(corner_angle)
            corners.append([int(corner_x), int(corner_y)])

        return np.array(corners, dtype=np.int32)


    def randomize_colors(self):
        self.block_color_id = random.sample(list(COLOR_BGR.keys()), 5)
        self.target_color_id = random.choice(self.block_color_id)

    def reset_large_params(self):
        self.a = random.uniform(0.780, 1.045)
        self.omega = random.uniform(1.884, 2.000)
        self.b = 2.090 - self.a
        self.start_time = time.time()
        
def main():
    simulator = BuffSimulator(mode="small")
    simulator.run()

if __name__ == "__main__":
    main()


        