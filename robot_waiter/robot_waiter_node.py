import rclpy
from rclpy.node import Node
import time

class RobotWaiter(Node):
    def __init__(self):
        super().__init__('robot_waiter')

        self.orders = []  # List of active orders
        self.canceled_orders = set()  # Store canceled orders
        self.timer = self.create_timer(1.0, self.process_orders)

    def receive_order(self, order_list):
        """Receive and queue new orders."""
        self.orders.extend(order_list)
        self.get_logger().info(f"Orders received: {self.orders}")

    def cancel_order(self, table):
        """Cancel an order and ensure food is returned to the kitchen."""
        if table in self.orders:
            self.get_logger().info(f"Order for {table} has been cancelled.")
            self.canceled_orders.add(table)  # Mark as canceled
        else:
            self.get_logger().info(f"Table {table} has no active order.")

    def return_to_kitchen(self, table):
        """Simulate returning the canceled food to the kitchen."""
        self.get_logger().info(f"Returning {table}'s food to the kitchen.")
        time.sleep(2)

    def process_orders(self):
        """Processes orders, skips canceled tables, and ensures canceled food is returned first."""
        if not self.orders:
            return

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

def main(args=None):
    rclpy.init(args=args)
    node = RobotWaiter()

    # Test: Receiving and canceling orders
    node.receive_order(['table1', 'table2', 'table3'])
    node.cancel_order('table2')  # Only cancel table2, others should be served
    node.process_orders()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
