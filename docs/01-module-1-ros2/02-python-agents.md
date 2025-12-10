---
sidebar_position: 2
---

# Building Your First Agent with rclpy

## Introduction to Python Nodes

In ROS 2, a **node** is an executable that uses ROS 2 to communicate with other nodes. Think of it as an intelligent agent that performs a specific function within your robot's ecosystem. In this module, we'll build your first Python-based agents using the `rclpy` library, which is the Python client library for ROS 2.

## Step-by-Step Guide to Writing a Python Node

Let's start by understanding the basic structure of a ROS 2 Python node:

```python
import rclpy                    # ROS 2 Python client library
from rclpy.node import Node     # Base class for ROS 2 nodes

class MyNode(Node):             # Inherit from Node class
    def __init__(self):
        super().__init__('my_node_name')  # Initialize node with a name
        # Node-specific initialization code goes here

def main(args=None):
    rclpy.init(args=args)       # Initialize ROS 2 communications
    node = MyNode()             # Create an instance of your node

    try:
        rclpy.spin(node)        # Keep the node alive
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()     # Clean up resources
        rclpy.shutdown()        # Shutdown ROS 2 communications

if __name__ == '__main__':
    main()                      # Execute main function when run directly
```

## Minimal Publisher Example

Here's a complete minimal publisher that publishes messages to a topic:

```python linenums="1"
import rclpy                    # Import the ROS 2 Python client library
from rclpy.node import Node     # Import the base Node class
from std_msgs.msg import String # Import standard message type for strings

class MinimalPublisher(Node):   # Define a publisher class that inherits from Node
    """A minimal publisher node that sends messages to a topic."""

    def __init__(self):         # Constructor method
        super().__init__('minimal_publisher')  # Initialize the node with name 'minimal_publisher'

        # Create a publisher that publishes String messages to the topic 'topic'
        # The queue size (10) determines how many messages to buffer if the subscriber is slow
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create a timer that calls timer_callback every 0.5 seconds (500ms)
        timer_period = 0.5      # Timer period in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)  # Create timer

        self.i = 0              # Counter to track number of messages published

    def timer_callback(self):   # Callback function called by the timer
        """Callback function that executes every time the timer fires."""
        msg = String()          # Create a new String message
        msg.data = f'Hello World: {self.i}'  # Set the message data with counter value

        # Publish the message to the topic
        self.publisher_.publish(msg)

        # Log the published message to the console with timestamp
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.i += 1             # Increment the counter

def main(args=None):            # Main function
    """Main function to initialize and run the publisher node."""
    rclpy.init(args=args)       # Initialize the ROS 2 communication system

    minimal_publisher = MinimalPublisher()  # Create an instance of the publisher node

    try:
        rclpy.spin(minimal_publisher)  # Keep the node running and processing callbacks
    except KeyboardInterrupt:   # Handle Ctrl+C interruption
        pass
    finally:
        # Clean up resources when exiting
        minimal_publisher.destroy_node()  # Destroy the node
        rclpy.shutdown()        # Shutdown the ROS 2 communication system

if __name__ == '__main__':      # Entry point of the script
    main()                      # Call the main function
```

### Code Explanation Line by Line:

- **Line 1**: Import the ROS 2 Python client library
- **Line 2**: Import the base Node class that provides ROS 2 functionality
- **Line 3**: Import String message type for publishing text
- **Line 5**: Define a class that inherits from Node (the base ROS 2 node class)
- **Line 8**: Constructor that initializes the node with the name 'minimal_publisher'
- **Line 11**: Create a publisher that sends String messages to the 'topic' with a queue size of 10
- **Line 14**: Define timer period as 0.5 seconds
- **Line 15**: Create a timer that calls `timer_callback` every 0.5 seconds
- **Line 17**: Initialize a counter variable
- **Line 19**: Define the timer callback function that gets executed periodically
- **Line 22**: Create a new String message object
- **Line 23**: Set the message data with a formatted string including the counter
- **Line 26**: Publish the message to the topic
- **Line 29**: Log the published message to the console
- **Line 31**: Increment the message counter
- **Line 34**: Main function that serves as the entry point
- **Line 37**: Initialize the ROS 2 communication system
- **Line 39**: Create an instance of the publisher node
- **Line 42**: Keep the node running and processing callbacks
- **Line 43-48**: Handle cleanup when the node is interrupted

## Minimal Subscriber Example

Now let's look at a complete minimal subscriber that listens to messages from a topic:

```python linenums="1"
import rclpy                    # Import the ROS 2 Python client library
from rclpy.node import Node     # Import the base Node class
from std_msgs.msg import String # Import standard message type for strings

class MinimalSubscriber(Node):  # Define a subscriber class that inherits from Node
    """A minimal subscriber node that receives messages from a topic."""

    def __init__(self):         # Constructor method
        super().__init__('minimal_subscriber')  # Initialize the node with name 'minimal_subscriber'

        # Create a subscription to the 'topic' that expects String messages
        # When a message arrives, the listener_callback function will be called
        self.subscription = self.create_subscription(
            String,             # Message type
            'topic',            # Topic name
            self.listener_callback,  # Callback function
            10                  # Queue size
        )
        # This prevents unused variable warnings
        self.subscription  # Store the subscription reference

    def listener_callback(self, msg):  # Callback function for incoming messages
        """Callback function that executes when a message is received."""
        # Log the received message to the console
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):            # Main function
    """Main function to initialize and run the subscriber node."""
    rclpy.init(args=args)       # Initialize the ROS 2 communication system

    minimal_subscriber = MinimalSubscriber()  # Create an instance of the subscriber node

    try:
        rclpy.spin(minimal_subscriber)  # Keep the node running and processing callbacks
    except KeyboardInterrupt:   # Handle Ctrl+C interruption
        pass
    finally:
        # Clean up resources when exiting
        minimal_subscriber.destroy_node()  # Destroy the node
        rclpy.shutdown()        # Shutdown the ROS 2 communication system

if __name__ == '__main__':      # Entry point of the script
    main()                      # Call the main function
```

### Code Explanation Line by Line:

- **Line 1**: Import the ROS 2 Python client library
- **Line 2**: Import the base Node class that provides ROS 2 functionality
- **Line 3**: Import String message type for receiving text
- **Line 5**: Define a class that inherits from Node (the base ROS 2 node class)
- **Line 8**: Constructor that initializes the node with the name 'minimal_subscriber'
- **Line 11**: Start creating a subscription to receive messages
- **Line 12**: Specify the message type (String)
- **Line 13**: Specify the topic name ('topic')
- **Line 14**: Specify the callback function to execute when a message is received
- **Line 15**: Set the queue size for buffering messages
- **Line 18**: Store the subscription reference to prevent garbage collection
- **Line 20**: Define the callback function that handles incoming messages
- **Line 23**: Log the received message to the console
- **Line 26**: Main function that serves as the entry point
- **Line 29**: Initialize the ROS 2 communication system
- **Line 31**: Create an instance of the subscriber node
- **Line 34**: Keep the node running and processing callbacks
- **Line 35-40**: Handle cleanup when the node is interrupted

## Running the Nodes

To run these nodes:

1. **Terminal 1** (for the publisher):
   ```bash
   python3 minimal_publisher.py
   ```

2. **Terminal 2** (for the subscriber):
   ```bash
   python3 minimal_subscriber.py
   ```

You should see the publisher sending messages and the subscriber receiving them.

:::tip
**Best Practices for Node Development:**
- Always use descriptive node names that reflect their function
- Include proper error handling and resource cleanup
- Use appropriate queue sizes based on your application's needs
- Log important events for debugging and monitoring
:::

:::danger
**Important Notes:**
- Make sure to call `destroy_node()` and `rclpy.shutdown()` in the finally block to properly clean up resources
- Use try-except blocks to handle interruptions gracefully
- Be mindful of timer periods - too frequent timers can overload the system
:::

## Challenge Activity

**Modify the code to publish your name:**

Create a modified version of the publisher that publishes your name instead of "Hello World". Here's how you can do it:

```python linenums="1"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NamePublisher(Node):
    """A publisher that publishes your name."""

    def __init__(self):
        super().__init__('name_publisher')

        self.publisher_ = self.create_publisher(String, 'my_name_topic', 10)

        timer_period = 1.0      # Publish every second
        self.timer = self.create_timer(timer_period, self.publish_name)

        self.counter = 0        # Counter for publishing sequence

    def publish_name(self):     # Modified callback to publish your name
        """Publish your name to the topic."""
        msg = String()
        # TODO: Replace 'Your Name' with your actual name
        msg.data = f'My name is: Your Name (Message #{self.counter})'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    name_publisher = NamePublisher()

    try:
        rclpy.spin(name_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        name_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Your Challenge:**
1. Modify the code above to replace 'Your Name' with your actual name
2. Optionally, add more personal information to the message
3. Create a corresponding subscriber that receives and displays the name message
4. Experiment with different timer periods to control the frequency of publications

## Summary

In this module, you've learned how to create basic ROS 2 nodes using Python and the rclpy library. You now understand the publisher-subscriber pattern and can implement both publishers and subscribers. These foundational concepts will serve as the building blocks for more complex robot behaviors in future modules.