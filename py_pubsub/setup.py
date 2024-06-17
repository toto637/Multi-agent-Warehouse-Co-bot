from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='thomas_medhat',
    maintainer_email='thomas_medhat@todo.todo',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # to install the node and could run it using (ros2 run pkg_name excutable_name) 
                'talker = py_pubsub.publisher_member_function:main', # "excutable_name"= pkg_name.file_name:function i want to run
 		        'listener = py_pubsub.subscriber_member_function:main',        
        ],
 },
 
)
