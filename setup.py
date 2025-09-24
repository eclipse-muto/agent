from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'agent'

setup(
    name=package_name,
    version='2.0.0',
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
    description='Eclipse Muto Agent Package - Refactored for improved modularity and robustness',
    license='Eclipse Public License v2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'muto_agent = agent.muto_agent:main',
            'mqtt = agent.mqtt:main',
            'commands = agent.commands:main',
            'symphony_provider = agent.symphony.symphony_provider:main'
        ],
    },
    python_requires='>=3.8',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Eclipse Public License 2.0 (EPL-2.0)',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: Scientific/Engineering :: Interface Engine/Protocol Translator',
    ],
)
