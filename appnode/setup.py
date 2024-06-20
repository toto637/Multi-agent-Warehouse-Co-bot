from setuptools import find_packages, setup

package_name = 'appnode'

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
    maintainer_email='temi.medhat@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roshttp = appnode.roshttp:main',    # "excutable_name"= pkg_name.file_name(without extension):function i want to run
            'goal = appnode.goal:main', 
            'navhttp = appnode.navhttp:main', 
            'appnav = appnode.appnav:main', 
        ],
    },
)
