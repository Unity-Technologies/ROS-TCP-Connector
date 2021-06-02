#!/usr/bin/env python

import rclpy

from ros2_tcp_endpoint import TcpServer


def main(args=None):
    rclpy.init(args=args)
    tcp_server = TcpServer('TCPServer')

    tcp_server.start()

    tcp_server.setup_executor()

    tcp_server.destroy_nodes()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
