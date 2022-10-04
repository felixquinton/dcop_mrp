import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Int32


class MinimalObstacle(Node):

    def __init__(self, positions=None):
        super().__init__('minimal_obstacle')
        
        self.publisher_ = self.create_publisher(Int32, 'test_obstacle_sim_robots', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.positions = self.set_positions(positions)
        print(positions)

    def timer_callback(self):
        msg = Int32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Obstacle publishing: "%s"' % msg.data)
        self.i += 1
        
    def set_positions(self, positions):
        if positions == None:
            nb_points = np.random.randint(3, 6)
            return self.polygon_gen(nb_points)
        else:
        	return positions
            
    def polygon_gen(self, nb_points):
        last_phi = 0
        phis = []
        for i in range(nb_points):
            phis.append(np.random.uniform(2*np.pi*(i/nb_points),
								          2/(nb_points-i)*np.pi))    
            last_phi = phis[-1]
        rhos = np.random.uniform(size=nb_points)
        cart_coords = np.dstack((rhos*np.cos(phis), rhos*np.sin(phis)))
        return cart_coords.squeeze()


def main(args=None):
    rclpy.init(args=args)

    minimal_obstacle = MinimalObstacle()

    rclpy.spin(minimal_obstacle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_obstacle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
