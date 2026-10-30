import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

TRIGGER_M = 0.9
EMERGENCY_M = 0.65
SPEED_MPS = 0.08
TURN_RADPS = 0.5
TURN_SECONDS = 3.14
TOTAL_SECONDS = 65.0

def examples():
    data = []
    for left in (0.7, 1.0, 1.4, 1.8, 2.4, 3.0):
        for right in (0.7, 1.0, 1.4, 1.8, 2.4, 3.0):
            if abs(left - right) >= 0.15:
                label = 'LEFT' if left > right else 'RIGHT'
                data.append((left, right, label))
    return data

TRAIN = examples()

def knn(left, right, k=3):
    nearest = sorted(
        TRAIN,
        key=lambda e: (e[0]-left)**2 + (e[1]-right)**2
    )[:k]

    votes = {'LEFT': 0.0, 'RIGHT': 0.0}
    for l, r, label in nearest:
        votes[label] += 1 / (
            0.01 + (l-left)**2 + (r-right)**2
        )

    return max(votes, key=votes.get)

class Robot(Node):
    def __init__(self):
        super().__init__('pai_ai_long_run')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan, 10
        )
        self.front = None
        self.left = None
        self.right = None
        self.last_scan = None

    def sector(self, scan, center):
        values = []
        for i, d in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            diff = math.atan2(
                math.sin(angle-center),
                math.cos(angle-center)
            )
            if abs(diff) <= math.radians(15):
                if math.isfinite(d):
                    if scan.range_min <= d <= scan.range_max:
                        values.append(d)
        return min(values) if values else scan.range_max

    def on_scan(self, scan):
        self.front = self.sector(scan, 0)
        self.left = self.sector(scan, math.pi/2)
        self.right = self.sector(scan, -math.pi/2)
        self.last_scan = time.monotonic()

    def drive(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.drive()
            time.sleep(0.08)

    def run(self):
        print(f'AI: weighted k-NN, examples={len(TRAIN)}')
        print('TASK: AI avoidance and 65-second run')

        start = time.monotonic()
        phase = 'APPROACH'
        turn_end = 0.0
        turn_sign = 0
        last_print = 0.0

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                now = time.monotonic()
                elapsed = now - start

                if elapsed >= TOTAL_SECONDS:
                    print('TASK COMPLETE: time limit reached')
                    break

                if self.last_scan is None:
                    self.drive()
                    continue

                if now - self.last_scan > 1.0:
                    print('SAFE STOP: LiDAR timeout')
                    break

                if self.front is None:
                    self.drive()
                    continue

                if now - last_print >= 1.0:
                    print(
                        f'T={elapsed:.1f}s '
                        f'PHASE={phase} '
                        f'F={self.front:.2f} '
                        f'L={self.left:.2f} '
                        f'R={self.right:.2f}',
                        flush=True
                    )
                    last_print = now

                if phase == 'APPROACH':
                    if self.front <= TRIGGER_M:
                        self.drive()
                        decision = knn(self.left, self.right)
                        print(
                            f'AI decision: {decision}',
                            flush=True
                        )
                        turn_sign = 1 if decision == 'LEFT' else -1
                        turn_end = now + TURN_SECONDS
                        phase = 'TURN'
                    else:
                        self.drive(SPEED_MPS, 0.0)

                elif phase == 'TURN':
                    if now < turn_end:
                        self.drive(0.0, turn_sign * TURN_RADPS)
                    else:
                        self.drive()
                        phase = 'CRUISE'
                        print('TURN completed -> CRUISE')

                elif phase == 'CRUISE':
                    if self.front <= EMERGENCY_M:
                        print('SAFE STOP: obstacle ahead')
                        break
                    self.drive(SPEED_MPS, 0.0)

        finally:
            self.stop()

def main():
    rclpy.init()
    robot = Robot()
    try:
        robot.run()
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
