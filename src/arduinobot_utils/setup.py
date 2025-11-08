from setuptools import setup

package_name = 'arduinobot_utils'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    # Install the top-level world_markers_publisher.py as a module
    py_modules=['world_markers_publisher'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jashwanth',
    maintainer_email='jashwanth@example.com',
    description='ArduinoBot utility nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Use the top-level module for markers (deleted duplicate inside package)
            'world_markers_publisher = world_markers_publisher:main',
            # Backward-compatible alias to match existing VS Code task name
            'world_markers_publisher.py = world_markers_publisher:main',
            # Keep other utilities inside the package
            'ee_markers_publisher = arduinobot_utils.ee_markers_publisher:main',
        ],
    },
)
