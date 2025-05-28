from setuptools import find_packages, setup

package_name = 'lecture'

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
    maintainer='ssafy',
    maintainer_email='ssafy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_prog = lecture.main_prog:main',
            'realsensewithYolov5 = lecture.realsensewithYolov5:main',
            'roboDK_client = lecture.roboDK_client:main',
            'rp5_client = lecture.rp5_client:main',
            'pickandplace=lecture.pickandplace:main',
            'dobot_homing_service=lecture.dobot_homing_service:main'
        ],
    },
)
