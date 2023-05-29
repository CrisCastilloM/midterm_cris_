#!/usr/bin/env python3
import rclpy, grpc
from rclpy.node import Node 
from concurrent.futures import ThreadPoolExecutor
from std_msgs.msg import Int32MultiArray
from interfases import midterm_pb2_grpc, midterm_pb2

class Wrapper(Node):
  def __init__(self, service):
    super().__init__('wrapper')
    self.create_subscription(Int32MultiArray,
      '/coordinates', self.setCoordinates, 10)
    self.service = service
  
  def setCoordinates(self, msg):
    self.service.x = msg.data[0]
    self.service.y = msg.data[1]


class Servicer(midterm_pb2_grpc.ColorDetectorServicer):
  def __init__(self):
    super().__init__()
    self.x, self.y = -1, -1


  def getCoordinate(self, req, ctx):
    return midterm_pb2.Coordinate(x = self.x, y = self.y)


def run():
  rclpy.init()
  service = Servicer()
  node = Wrapper(service)

  server = grpc.server(ThreadPoolExecutor(max_workers = 10))
  server.add_insecure_port(f'[::]:50051')

  midterm_pb2_grpc.add_ColorDetectorServicer_to_server(
    service, server)

  try:
    server.start()
    print(f"Started grpc server on port 50051")
    rclpy.spin(node)
  finally:
    rclpy.shutdown()
    server.wait_for_termination()


if __name__ == '__main__':
  run()

