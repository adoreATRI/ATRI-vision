import math
import random
import time
from pathlib import Path

import cv2
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


class BuffSimulatorNew:
    def __init__(self, mode: str = "small"):
        self.mode = mode

        self.angle = 0
        self.start_time = time.time()
        self.direction = 1

        self.a = random.uniform(0.780, 1.045)
        self.omega = random.uniform(1.884, 2.000)
        self.b = 2.090 - self.a

        self.block_color_id = []
        self.target_color_id = -1
        self.randomize_colors()

        self.img_w = 1280
        self.img_h = 720

        self.pixels_per_cm = 12.8
        self.pixels_per_mm = 1.28

        self.block_side = 8.0 * self.pixels_per_cm
        self.block_distance = 16.0 * self.pixels_per_cm

        self.center_outer_radius = 4.0 * self.pixels_per_cm
        self.center_inner_radius = 0

        self.white_bg_inner_dist = 11.339 * self.pixels_per_cm

        # Load background image
        bg_path = Path(__file__).resolve().parent / "background" / "background.jpg"
        if bg_path.exists():
            self.bg_img = cv2.imread(str(bg_path))
            if self.bg_img is not None:
                self.bg_img = cv2.resize(self.bg_img, (self.img_w, self.img_h))
        else:
            self.bg_img = None

    def run(self):
        cv2.namedWindow("Buff Simulator New", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Buff Simulator New", 1280, 720)
        last_time = time.time()
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            self.update(dt)
            img = self.render()
            self.draw_info(img)
            cv2.imshow("Buff Simulator New", img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            elif key == ord("m"):
                self.mode = "large" if self.mode == "small" else "small"
            elif key == ord("d"):
                self.direction *= -1
            elif key == ord("r"):
                self.randomize_colors()
            elif key == ord("p"):
                self.reset_large_params()
        cv2.destroyAllWindows()

    def update(self, dt: float):
        t = time.time() - self.start_time
        angular_velocity = self.get_angular_velocity(t)
        self.angle += angular_velocity * dt * self.direction
        self.angle = self.angle % (2 * math.pi)

    def get_angular_velocity(self, t: float) -> float:
        if self.mode == "small":
            return math.pi / 3.0
        else:
            return self.a * math.sin(self.omega * t) + self.b

    def render(self) -> np.ndarray:
        if self.bg_img is not None:
            img = self.bg_img.copy()
        else:
            img = np.full((self.img_h, self.img_w, 3), 50, dtype=np.uint8)

        cx = self.img_w // 2
        cy = self.img_h // 2

        bg_points = []
        half_diagonal = self.block_side * math.sqrt(2) / 2
        # Margin is 3cm mapping, distance expanded diagonally is 3*sqrt(2)
        bg_hd = half_diagonal + 3.0 * math.sqrt(2) * self.pixels_per_cm

        for i in range(5):
            angle_diff = 2 * math.pi / 5
            block_angle = i * angle_diff + self.angle

            p_right = (
                int(
                    cx + self.block_distance * math.cos(block_angle) + bg_hd * math.sin(block_angle)
                ),
                int(
                    cy + self.block_distance * math.sin(block_angle) - bg_hd * math.cos(block_angle)
                ),
            )
            p_outer = (
                int(cx + (self.block_distance + bg_hd) * math.cos(block_angle)),
                int(cy + (self.block_distance + bg_hd) * math.sin(block_angle)),
            )
            p_left = (
                int(
                    cx + self.block_distance * math.cos(block_angle) - bg_hd * math.sin(block_angle)
                ),
                int(
                    cy + self.block_distance * math.sin(block_angle) + bg_hd * math.cos(block_angle)
                ),
            )

            bisector_angle = block_angle + math.pi / 5
            p_inner = (
                int(cx + self.white_bg_inner_dist * math.cos(bisector_angle)),
                int(cy + self.white_bg_inner_dist * math.sin(bisector_angle)),
            )

            bg_points.append(p_right)
            bg_points.append(p_outer)
            bg_points.append(p_left)
            bg_points.append(p_inner)

        bg_points = np.array(bg_points, dtype=np.int32)
        cv2.fillPoly(img, [bg_points], (255, 255, 255))

        for i in range(5):
            self.draw_block(img, i, self.angle, self.block_color_id[i])

        self.draw_center_ring(img)

        return img

    def draw_info(self, img):
        info_lines = [
            f"Angular Vel: {self.get_angular_velocity(time.time() - self.start_time):.3f} rad/s",
            (
                f"a: {self.a:.3f}, omega: {self.omega:.3f}, b: {self.b:.3f}"
                if self.mode == "large"
                else "Small Buff Mode"
            ),
        ]
        y_offset = 30
        for line in info_lines:
            cv2.putText(img, line, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(
                img, line, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
            )
            y_offset += 20

    def draw_block(self, img, block_idx, angle, block_color_id):
        angle_diff = 2 * math.pi / 5
        block_angle = block_idx * angle_diff + angle

        cx = self.img_w // 2
        cy = self.img_h // 2
        center_x = cx + self.block_distance * math.cos(block_angle)
        center_y = cy + self.block_distance * math.sin(block_angle)

        corners = self.get_block_corners(center_x, center_y, block_angle, self.block_side)

        color = COLOR_BGR[block_color_id]
        cv2.fillPoly(img, [corners], color)

    def draw_center_ring(self, img):
        center = (self.img_w // 2, self.img_h // 2)
        target_color = COLOR_BGR[self.target_color_id]
        cv2.circle(img, center, int(self.center_outer_radius), target_color, -1)

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
    simulator = BuffSimulatorNew(mode="small")
    simulator.run()


if __name__ == "__main__":
    main()
