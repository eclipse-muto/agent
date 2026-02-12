from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'muto_agent'

setup(
    name=package_name,
    version='0.42.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Composive Team',
    maintainer_email='info@composiv.ai',
    description='Eclipse Muto Agent - pluggable communication gateway bridging ROS 2 nodes with cloud backends via MQTT/Ditto and Symphony protocols',
    license='Eclipse Public License v2.0',
    entry_points={
        'console_scripts': [
            'muto_agent = muto_agent.muto_agent:main',
            'mqtt = muto_agent.mqtt:main',
            'commands = muto_agent.commands:main',
            'symphony_provider = muto_agent.symphony.symphony_provider:main'
        ],
    },
    python_requires='>=3.10',
)
