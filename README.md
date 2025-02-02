## Robot Waiter Node Documentation

### Overview

The **Robot Waiter** is a ROS 2 (Robot Operating System) node that simulates a robot moving from its home position to deliver food to various tables in a restaurant. The robot handles tasks such as receiving orders, delivering food, canceling orders, and returning to the kitchen if needed.

This documentation explains the code that enables the robot to perform these tasks step by step.

---

### Key Concepts

Before we dive into the details, let's cover some key terms used in the code:

- **Node**: In ROS 2, a node is a process that performs computation. The `RobotWaiter` is a node that handles order processing and movement.
- **Order**: An order is represented by the table number where the food needs to be delivered. The robot handles multiple orders at once.
- **Home Position**: The starting position where the robot begins its task.
- **Kitchen**: The location where the robot picks up the food before delivering it to the table.
- **Timer**: A timer is used to periodically call a function (in this case, to process orders).

---

### Main Functionality

The **RobotWaiter** node has several key functions:

1. **Receive Orders**: The robot receives a list of table orders and processes them.
2. **Move to Kitchen**: The robot goes to the kitchen to pick up the food.
3. **Deliver Food**: The robot moves to the specified table to deliver the food.
4. **Return to Home**: After delivering the food, the robot returns to its home position.
5. **Handle Cancellations**: If an order is canceled, the robot returns the food to the kitchen and either completes the delivery or goes home.

---

### Code Breakdown

#### 1. **Initializing the RobotWaiter Node**

```python
class RobotWaiter(Node):
    def __init__(self):
        super().__init__('robot_waiter')
        self.orders = []  # List of active orders
        self.canceled_orders = set()  # Store canceled orders
        self.timer = self.create_timer(1.0, self.process_orders)  # Timer to process orders every 1 second
```

- The `RobotWaiter` class inherits from `Node`, which means it’s a ROS 2 node.
- The `orders` list stores the active orders, and `canceled_orders` keeps track of canceled orders.
- The `create_timer` function calls the `process_orders` function every 1 second to handle the orders.

#### 2. **Receiving Orders**

```python
def receive_order(self, order_list):
    """Receive and queue new orders."""
    self.orders.extend(order_list)
    self.get_logger().info(f"Orders received: {self.orders}")
```

- The `receive_order` method adds a list of orders (table numbers) to the `orders` list.
- It also logs a message to confirm that the orders have been received.

#### 3. **Canceling Orders**

```python
def cancel_order(self, table):
    """Cancel an order and ensure food is returned to the kitchen."""
    if table in self.orders:
        self.get_logger().info(f"Order for {table} has been cancelled.")
        self.canceled_orders.add(table)  # Mark as canceled
    else:
        self.get_logger().info(f"Table {table} has no active order.")
```

- The `cancel_order` method cancels a specific table’s order.
- If the order is found in the `orders` list, it is moved to `canceled_orders`.

#### 4. **Returning to Kitchen**

```python
def return_to_kitchen(self, table):
    """Simulate returning the canceled food to the kitchen."""
    self.get_logger().info(f"Returning {table}'s food to the kitchen.")
    time.sleep(2)  # Simulating time taken to return food to the kitchen
```

- The `return_to_kitchen` function simulates the process of returning food to the kitchen if the order is canceled.

#### 5. **Processing Orders**

```python
def process_orders(self):
    """Processes orders, skips canceled tables, and ensures canceled food is returned first."""
    if not self.orders:
        return  # If no orders, do nothing

    table = self.orders.pop(0)  # Get the first order in queue
    self.get_logger().info(f"Moving to kitchen for {table}.")

    time.sleep(1)  # Simulate moving
    self.get_logger().info(f"Food is ready. Collecting food for {table}.")
    time.sleep(1)

    if table in self.canceled_orders:
        self.get_logger().info(f"Order for {table} was canceled! Returning food.")
        self.return_to_kitchen(table)
        self.canceled_orders.remove(table)  # Remove from canceled list
    else:
        self.get_logger().info(f"Delivering food to {table}.")
        time.sleep(1)
        self.get_logger().info(f"Food delivered to {table}.")

    self.get_logger().info("Returning to home position.")
    time.sleep(1)
    self.get_logger().info("Robot is home.")
```

- The `process_orders` method:
  - If there are no orders, it does nothing.
  - For each order, it simulates the robot moving to the kitchen, collecting food, and delivering it to the table.
  - If the order is canceled, it returns the food to the kitchen.
  - After completing the delivery, it returns to the home position.

#### 6. **Main Function**

```python
def main(args=None):
    rclpy.init(args=args)
    node = RobotWaiter()

    # Test: Receiving and canceling orders
    node.receive_order(['table1', 'table2', 'table3'])
    node.cancel_order('table2')  # Only cancel table2, others should be served
    node.process_orders()

    rclpy.spin(node)  # Keep the node running
    node.destroy_node()
    rclpy.shutdown()
```

- The `main` function initializes ROS, creates the `RobotWaiter` node, and tests the order and cancellation process.
- It simulates receiving orders, canceling one of them, and processing the remaining orders.

---

### Launch File 

The launch file is used to launch the `robot_waiter` node in ROS.

```python
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_waiter',
            executable='robot_waiter_node',
            name='robot_waiter',
            output='screen',
            parameters=[{'timeout': 5}],  # Set timeout parameter if needed
            remappings=[('/order_received', '/order_received')]  # Remap topic if needed
        )
    ])
```

- The launch file starts the `robot_waiter_node` with the necessary parameters and topic remappings.
- This is useful to run the node in a controlled environment with specific settings.

---

### How the Robot Waiter Node Handles Different Scenarios

Here’s how the robot handles various situations based on the conditions outlined:

1. **Order Reception**: The robot will start moving to the kitchen and then deliver food to each table in the order received.
2. **Order Cancellation**: If an order is canceled while the robot is on its way to the table or kitchen, it will return to the kitchen and home accordingly.
3. **Multiple Orders**: The robot will process multiple orders one after another, skipping canceled orders and delivering food to the remaining tables.

---

### Next Steps for Improvement

- **Timeout Handling**: You can add a timeout feature to make the robot wait for a confirmation from the table or kitchen before returning home.
- **Confirmation Mechanism**: You could implement a mechanism for the robot to wait for confirmation before delivering food, ensuring it knows when the task is complete.

---

### Conclusion

The `RobotWaiter` node simulates the behavior of a robot in a restaurant setting, processing orders, delivering food, and handling cancellations. By breaking down the problem into smaller steps (receiving orders, moving to the kitchen, delivering food, and handling cancellations), the code provides a simple and effective way to manage robot behavior in a restaurant scenario.

