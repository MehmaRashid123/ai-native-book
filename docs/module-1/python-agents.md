---
sidebar_position: 2
---

# Python Agents with rclpy

In this section, we'll explore how to create intelligent agents using Python and the Robot Operating System 2 (ROS 2) through the `rclpy` library.

## What You'll Learn

- How to create ROS 2 nodes using Python
- Publisher and subscriber patterns in `rclpy`
- Service and action clients
- Best practices for Python-based robot control

## Prerequisites

- Understanding of ROS 2 concepts from the introduction
- Basic Python programming knowledge

## Creating Your First ROS 2 Node in Python

Let's start by creating a simple ROS 2 node using `rclpy`...

```python
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Simple node has been created')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This is the basic structure of a ROS 2 node in Python.