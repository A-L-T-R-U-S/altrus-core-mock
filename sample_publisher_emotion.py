# sample_publisher_emotion.py
# Publishes a single emotion intent to intent_raw/emotion

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, datetime, uuid

def now_iso():
    return datetime.datetime.utcnow().isoformat() + 'Z'

class EmotionPub(Node):
    def __init__(self):
        super().__init__('emotion_pub')
        self.pub = self.create_publisher(String, 'intent_raw/emotion', 10)
        self.timer = self.create_timer(1.0, self.send_once)
        self.sent = False

    def send_once(self):
        if self.sent:
            return
        envelope = {
            'event_id': str(uuid.uuid4()),
            'timestamp': now_iso(),
            'source_module': 'EmotionFramework',
            'event_type': 'IntentRaw',
            'payload': {
                'raw_intents': [{'intent': 'play_meditation', 'confidence': 0.91}],
                'final_intent': 'play_meditation',
                'parameters': {'track': 'calm_song'}
            },
            'fault_context': {'detected_fault': False},
            'telemetry': {'battery_level': '85%'}
        }
        self.pub.publish(String(data=json.dumps(envelope)))
        self.get_logger().info('Published emotion intent')
        self.sent = True

def main():
    rclpy.init()
    node = EmotionPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
