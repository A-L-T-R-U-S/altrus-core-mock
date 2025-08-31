

````markdown
# ALTRUS Mock Middleware

**A ROS 2 Python middleware for intent arbitration, fault detection, and command simulation in modular assistive robots.**
Part of the **A.L.T.R.U.S. (Adaptive Life-support & Therapeutic Robotic Unit System)** project.

---

## **Table of Contents**

1. [Overview](#overview)
2. [Features](#features)
3. [Installation](#installation)
4. [Usage](#usage)
   - [Run Middleware](#run-middleware)
   - [Simulate Intents](#simulate-intents)
5. [Configuration](#configuration)
6. [Testing End-to-End](#testing-end-to-end)
7. [Extending Middleware](#extending-middleware)
8. [File Structure](#file-structure)
9. [License](#license)

---

## **Overview**

`altrus_mock_middleware` acts as the backbone of the modular A.L.T.R.U.S. robot by:

- Interpreting raw intents from different modules (emotion, locomotion, telemedicine)
- Checking resource health and detecting faults
- Issuing commands to simulated or real subsystems
- Logging all events (intent arbitration, commands, faults) in JSON format
- Supporting both **simulation mode** and **real mode** for subsystem testing

This framework allows each team member to test their module integration independently before full robot integration.

---

## **Features**

- **Intent Engine**: Arbitrates multiple raw intents into a final command.
- **Fault Manager**: Checks required resources before issuing commands.
- **Command Simulation**: Publishes commands to middleware topics (`middleware/cmd/<service>`) for safe testing.
- **Logging**: Events are logged to `events.log` and published on `/events`.
- **Extensible**: Add new intents, resources, or recovery routines easily via YAML configuration files.

---

## **Installation**

**Prerequisites**:

- Ubuntu 22.04 / ROS 2 Humble
- Python >= 3.10
- Required packages: `rclpy`, `std_msgs`, `pyyaml`, `ament_index_python`

**Steps**:

```bash
# Clone your repo
cd ~/ros2_ws/src
git clone <repo_url> altrus_mock_middleware

# Build the package
cd ~/ros2_ws
colcon build --packages-select altrus_mock_middleware

# Source setup files
source /opt/ros/humble/setup.bash
source install/setup.bash
````

---

## **Usage**

### **Run Middleware**

```bash
ros2 run altrus_mock_middleware middleware_node --mode simulate
```

* `--mode simulate` → sends commands to middleware topics for testing
* `--mode real` → sends commands to actual subsystem topics

You can optionally specify YAML configs:

```bash
ros2 run altrus_mock_middleware middleware_node \
    --rules path/to/intent_rules.yml \
    --faults path/to/fault_manager.yml
```

---

### **Simulate Intents**

Sample publishers exist for testing:

* **Emotion Module**

```bash
ros2 run altrus_mock_middleware sample_publisher_emotion
```

* **Locomotion Module**

```bash
ros2 run altrus_mock_middleware sample_publisher_locomotion
```

* **Telemedicine Module**

```bash
ros2 run altrus_mock_middleware sample_publisher_telemedicine
```

> These publishers send single test intents to `intent_raw/<module>` topics for end-to-end testing.

---

## **Configuration**

All rules and resource definitions are configurable in the YAML files inside the package:

* `intent_rules.yml` → defines intents, target subsystems, commands, priorities, and required resources
* `fault_manager.yml` → defines monitored resources, heartbeat topics, and recovery procedures

> Modify these files to simulate different scenarios or add new commands.

---

## **Testing End-to-End**

1. **Start the middleware** in simulate mode:

```bash
ros2 run altrus_mock_middleware middleware_node --mode simulate
```

2. **Publish raw intents** from sample publishers:

```bash
ros2 run altrus_mock_middleware sample_publisher_emotion
ros2 run altrus_mock_middleware sample_publisher_locomotion
ros2 run altrus_mock_middleware sample_publisher_telemedicine
```

3. **Observe command outputs** on middleware topics:

```bash
ros2 topic echo /middleware/cmd/music
ros2 topic echo /middleware/cmd/locomotion
ros2 topic echo /middleware/cmd/telemedicine
```

4. **Observe events log**:

```bash
tail -f events.log
```

5. **Simulate faults**:
   Edit `fault_manager.yml` and mark a resource as unhealthy in `self.health` inside `MockMiddleware` to see how the middleware reacts.

---

## **Extending Middleware**

* **Add new intents**: Update `intent_rules.yml` and define required resources.
* **Add new resources**: Update `fault_manager.yml` with new heartbeat topics and recovery procedures.
* **Custom publishers**: Team members can write their own `rclpy` nodes sending intents to `/intent_raw/<module>` topics.
* **Developer CLI**: Optional future CLI can inject intents and manipulate resources for testing.

---

## **File Structure**

```
altrus_mock_middleware/
├── altrus_mock_middleware/
│   ├── __init__.py
│   └── mock_middleware_ros.py
├── resource/
│   └── altrus_mock_middleware  # marker file
├── intent_rules.yml
├── fault_manager.yml
├── sample_publisher_emotion.py
├── sample_publisher_locomotion.py
├── sample_publisher_telemedicine.py
├── package.xml
├── setup.py
└── README.md
```

---

## **License**

MIT License. See [LICENSE](LICENSE) file for details.
