from setuptools import find_packages, setup

package_name = 'roboneo_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aqirito',
    maintainer_email='aqirito@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_state_pub = roboneo_bot.led_state_pub:main',
            'hello_sub = roboneo_bot.hello_sub:main',
            'ultrasonic_sub = roboneo_bot.ultrasonic_sub:main',
            'roboneo_bot = roboneo_bot.roboneo_bot:main',
            'roboneo_bot_sonic_twist = roboneo_bot.roboneo_bot_sonic_twist:main',
            'roboneo_bot_sonic_lidar_twist = roboneo_bot.roboneo_bot_sonic_lidar_twist:main',
            'test_roboneo_bot = roboneo_bot.test_roboneo_bot:main',
            'roboneo_test_lidar = roboneo_bot.roboneo_test_lidar:main',
            'roboneo_test_color = roboneo_bot.roboneo_test_color:main',
            'roboneo_bot_full = roboneo_bot.roboneo_bot_full:main',
        ],
    },
)
