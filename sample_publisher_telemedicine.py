# sample_publisher_telemedicine.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, datetime, uuid

def now_iso():
    return datetime.datetime.utcnow().isoformat() + 'Z'

class TelePub(Node):
    def __init__(self):
        super().__init__('tele_pub')
        self.pub = self.create_publisher(String, 'intent_raw/telemedicine', 10)
        self.timer = self.create_timer(1.0, self.send_once)
        self.sent = False

    def send_once(self):
        if self.sent:
            return
        envelope = {
            'event_id': str(uuid.uuid4()),
            'timestamp': now_iso(),
            'source_module': 'Telemedicine',
            'event_type': 'IntentRaw',
            'payload': {
                'raw_intents': [{'intent': 'dispense_medicine', 'confidence': 0.95}],
                'final_intent': 'dispense_medicine',
                'parameters': {'medicine_id': 'med-101', 'dosage': '1 pill'}
            },
            'fault_context': {'detected_fault': False},
            'telemetry': {'battery_level': '78%'}
        }
        self.pub.publish(String(data=json.dumps(envelope)))
        self.get_logger().info('Published telemedicine intent')
        self.sent = True

def main():
    rclpy.init()
    node = TelePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
