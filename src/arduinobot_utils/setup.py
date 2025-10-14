from setuptools import setup

package_name = 'arduinobot_utils'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jashwanth',
    maintainer_email='jashwanth@example.com',
    description='ArduinoBot utility nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_markers_publisher = arduinobot_utils.world_markers_publisher:main',
        ],
    },
)
