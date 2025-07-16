from setuptools import find_packages, setup

package_name = 'bumperbot_python_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moose',
    maintainer_email='yacin.ha9@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = bumperbot_python_examples.simple_publisher:main',
            'simple_subscriber = bumperbot_python_examples.simple_subscriber:main',
            'simple_parameter = bumperbot_python_examples.simple_parameter:main',
            'turtlesim_kinematics = bumperbot_python_examples.turtlesim_kinematics:main', 
            'simple_tf_kinematics = bumperbot_python_examples.simple_tf_kinematics:main',
            "simple_service_server = bumperbot_python_examples.simple_service_server:main", 
            "simple_service_client = bumperbot_python_examples.simple_service_client:main", 
            "simple_action_client = bumperbot_python_examples.simple_action_client:main",
            "simple_lifecycle_node = bumperbot_python_examples.simple_lifecycle_node:main"
        ],
    },
)
