import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PygameTeleop(Node):
    def __init__(self):
        super().__init__('pygame_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        pygame.init()
        self.screen = pygame.display.set_mode((200, 200))
        pygame.display.set_caption("Pygame Teleop - Use WASD or Arrow Keys")
        self.clock = pygame.time.Clock()
        self.running = True

        # Set your speeds here
        self.linear_speed = 2.0
        self.angular_speed = 1.0

    def run(self):
        while self.running and rclpy.ok():
            linear = 0.0
            angular = 0.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            keys = pygame.key.get_pressed()
            # Forward/back
            if keys[pygame.K_w] or keys[pygame.K_UP]:
                linear += self.linear_speed
            if keys[pygame.K_s] or keys[pygame.K_DOWN]:
                linear -= self.linear_speed
            # Left/right
            if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                angular += self.angular_speed
            if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                angular -= self.angular_speed

            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.pub.publish(twist)

            self.screen.fill((30, 30, 30))
            font = pygame.font.SysFont(None, 24)
            msg = f"Linear: {linear:.2f}  Angular: {angular:.2f}"
            img = font.render(msg, True, (200, 200, 200))
            self.screen.blit(img, (10, 90))
            pygame.display.flip()

            self.clock.tick(40)  # 20 Hz

        pygame.quit()

def main():
    rclpy.init()
    node = PygameTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()