from setuptools import setup
import os

package_name = 'altrus_mock_middleware'

# Ensure resource directory exists
os.makedirs('resource', exist_ok=True)
with open('resource/altrus_mock_middleware', 'w') as f:
    pass  # touch file

# Data files to include
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/altrus_mock_middleware']),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name, ['LICENSE']),
    ('share/' + package_name, ['intent_rules.yml']),
    ('share/' + package_name, ['fault_manager.yml']),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vk',
    maintainer_email='vk@example.com',
    description='Mock middleware for ALTRUS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'middleware_node = altrus_mock_middleware.mock_middleware_ros:main',
        ],
    },
)