# sample_publisher_locomotion.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, datetime, uuid

def now_iso():
    return datetime.datetime.utcnow().isoformat() + 'Z'

class LocoPub(Node):
    def __init__(self):
        super().__init__('loco_pub')
        self.pub = self.create_publisher(String, 'intent_raw/locomotion', 10)
        self.timer = self.create_timer(1.0, self.send_once)
        self.sent = False

    def send_once(self):
        if self.sent:
            return
        envelope = {
            'event_id': str(uuid.uuid4()),
            'timestamp': now_iso(),
            'source_module': 'Locomotion',
            'event_type': 'IntentRaw',
            'payload': {
                'raw_intents': [{'intent': 'move', 'confidence': 0.88}],
                'final_intent': 'move',
                'parameters': {'distance': '1m', 'target': 'kitchen'}
            },
            'fault_context': {'detected_fault': False},
            'telemetry': {'battery_level': '80%'}
        }
        self.pub.publish(String(data=json.dumps(envelope)))
        self.get_logger().info('Published locomotion intent')
        self.sent = True

def main():
    rclpy.init()
    node = LocoPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
