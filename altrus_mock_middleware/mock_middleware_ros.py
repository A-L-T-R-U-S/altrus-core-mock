# mock_middleware_ros.py
# Simple ROS2 mock middleware node.
# Modes:
#   simulate - republishes to middleware/cmd/<service>
#   real     - republishes to the target subsystem's actual topic (configured)
#
# NOTE: This script expects ROS2 (rclpy) installed on the machine. See README.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import argparse
import json
import uuid
import datetime
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def now_iso():
    return datetime.datetime.utcnow().isoformat() + 'Z'


class MockMiddleware(Node):
    def __init__(self, mode='simulate', rules_file=None, fault_file=None, log_file='events.log'):
        super().__init__('mock_middleware')
        self.mode = mode
        self.get_logger().info(f'MockMiddleware starting in {mode} mode')

        # Get package share directory
        try:
            pkg_share = get_package_share_directory('altrus_mock_middleware')
        except Exception as e:
            self.get_logger().fatal(f'Package not found: {e}')
            raise

        # Set default paths if not provided
        rules_file = rules_file or os.path.join(pkg_share, 'intent_rules.yml')
        fault_file = fault_file or os.path.join(pkg_share, 'fault_manager.yml')
        self.log_file = log_file

        # Load rules and fault config
        try:
            with open(rules_file, 'r') as f:
                data = yaml.safe_load(f)
                self.rules = data.get('intents', {})
            self.get_logger().info(f'Loaded rules from {rules_file}')

            with open(fault_file, 'r') as f:
                data = yaml.safe_load(f)
                self.faults = data.get('resources', {})
            self.get_logger().info(f'Loaded faults from {fault_file}')
        except FileNotFoundError as e:
            self.get_logger().fatal(f'Config file not found: {e}')
            raise
        except PermissionError as e:
            self.get_logger().fatal(f'Permission denied reading config: {e}')
            raise
        except Exception as e:
            self.get_logger().fatal(f'Failed to load config: {e}')
            raise

        # Simple health map for resources (can be changed during runtime for tests)
        self.health = {r: True for r in self.faults.keys()}

        # Subscribers: raw intents from modules
        self.subs = []
        raw_topics = [
            ('intent_raw/emotion', 'emotion'),
            ('intent_raw/locomotion', 'locomotion'),
            ('intent_raw/telemedicine', 'telemedicine'),
        ]
        for topic, _ in raw_topics:
            sub = self.create_subscription(
                String,
                topic,
                self.cb_raw,
                10
            )
            self.subs.append(sub)

        # Publishers for middleware command topics (simulate)
        self.pub_cmd_music = self.create_publisher(String, 'middleware/cmd/music', 10)
        self.pub_cmd_loco = self.create_publisher(String, 'middleware/cmd/locomotion', 10)
        self.pub_cmd_tele = self.create_publisher(String, 'middleware/cmd/telemedicine', 10)

        # Events log topic
        self.pub_events = self.create_publisher(String, 'events', 10)

    def cb_raw(self, msg):
        try:
            envelope = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Invalid JSON received: {e}')
            return

        intent_block = envelope.get('payload', {})
        raw_intents = intent_block.get('raw_intents') or []
        final_intent = intent_block.get('final_intent')

        if not final_intent and raw_intents:
            final_intent = raw_intents[0].get('intent')
        if not final_intent:
            self.get_logger().warn('No intent found in message')
            return

        rule = self.rules.get(final_intent)
        event = {
            'event_id': str(uuid.uuid4()),
            'timestamp': now_iso(),
            'source_module': 'MockMiddleware',
            'event_type': 'IntentArbitrated',
            'payload': {
                'raw_intents': raw_intents,
                'final_intent': final_intent,
                'rule': final_intent
            },
            'fault_context': {'detected_fault': False},
            'telemetry': {'battery_level': 'unknown'},
        }

        # Fault/resource checking
        missing = []
        if rule:
            reqs = rule.get('required_resources', [])
            for r in reqs:
                if not self.health.get(r, False):
                    missing.append(r)

        if missing:
            event['fault_context'] = {'detected_fault': True, 'missing': missing}
            self.log_event(event)
            self.get_logger().warn(f'Resource(s) missing: {missing} -> publishing fault event')

            fault_msg = {
                'event_id': str(uuid.uuid4()),
                'timestamp': now_iso(),
                'source_module': 'MockMiddleware',
                'event_type': 'FaultDetected',
                'fault_context': {'missing': missing}
            }
            self.pub_events.publish(String(data=json.dumps(fault_msg)))
            return

        # Build command envelope
        cmd_env = {
            'event_id': str(uuid.uuid4()),
            'timestamp': now_iso(),
            'source_module': 'MockMiddleware',
            'event_type': 'CommandIssued',
            'command': {
                'command_id': rule.get('command') if rule else f'{final_intent}.execute',
                'command_params': intent_block.get('parameters', {}),
                'expected_ack': True
            }
        }

        # Republish according to mode
        target = rule.get('target') if rule else None
        if self.mode == 'simulate':
            if target == 'music':
                self.pub_cmd_music.publish(String(data=json.dumps(cmd_env)))
            elif target == 'locomotion':
                self.pub_cmd_loco.publish(String(data=json.dumps(cmd_env)))
            elif target == 'telemedicine':
                self.pub_cmd_tele.publish(String(data=json.dumps(cmd_env)))
            else:
                # fallback
                self.pub_events.publish(String(data=json.dumps(cmd_env)))
        else:
            # In 'real' mode, re-publish to the real topic name mapped by convention:
            # e.g. 'locomotion' -> '/locomotion/cmd'
            real_topic = f'/{target}/cmd' if target else '/unknown/cmd'
            pub = self.create_publisher(String, real_topic, 10)
            pub.publish(String(data=json.dumps(cmd_env)))

        # Log final event
        self.log_event(event)
        self.get_logger().info(f'Intent {final_intent} processed and command issued')

    def log_event(self, event):
        try:
            with open(self.log_file, 'a') as f:
                f.write(json.dumps(event) + '\n')
            # also publish on /events topic
            self.pub_events.publish(String(data=json.dumps(event)))
        except Exception as e:
            self.get_logger().warn(f'Failed to log event: {e}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['simulate', 'real'], default='simulate')
    parser.add_argument('--rules', default=None)  # Use None to trigger auto-path
    parser.add_argument('--faults', default=None)  # Use None to trigger auto-path
    args = parser.parse_args()

    rclpy.init()
    node = MockMiddleware(mode=args.mode, rules_file=args.rules, fault_file=args.faults)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()