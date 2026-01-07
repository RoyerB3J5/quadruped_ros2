import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point 
from visualization_msgs.msg import Marker 
from std_msgs.msg import ColorRGBA

class FootTrajectoryViz(Node):

  def __init__(self):
    super().__init__('foot_trajectory_viz')

    self.legs = ['front_left', 'front_right', 'rear_left', 'rear_right']
    self.colors = {
      'front_left': ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),   
      'front_right': ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  
      'rear_left': ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),    
      'rear_right': ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)    
    }

    self.paths = {leg: [] for leg in self.legs}

    self.marker_pub = self.create_publisher(Marker, '/foot_trajectories', 10)

    for leg in self.legs:
      self.create_subscription(
        Point, f'/foot_ref/{leg}',lambda msg, leg = leg: self.foot_callback(msg, leg), 10
      )
    
    self.timer = self.create_timer(0.05, self.publish_markers)
    self.get_logger().info("Foot Trajectory Visualizer Node has been started.")

  def foot_callback(self, msg, leg):
    p = Point()
    p.x = msg.x
    p.y = msg.y
    p.z = msg.z

    self.paths[leg].append(p)

    if len(self.paths[leg]) > 500:
      self.paths[leg].pop(0)
    
  def publish_markers(self):
    for i, leg in enumerate(self.legs):
      marker = Marker()
      marker.header.frame_id = 'base_link'
      marker.header.stamp = self.get_clock().now().to_msg()

      marker.ns = leg
      marker.id = i
      marker.type = Marker.LINE_STRIP
      marker.action = Marker.ADD

      marker.scale.x = 0.003
      marker.color = self.colors[leg]
      marker.points = self.paths[leg]

      marker.pose.orientation.w = 1.0
      marker.lifetime.sec = 0

      self.marker_pub.publish(marker)

def main():
  rclpy.init()
  node = FootTrajectoryViz()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()