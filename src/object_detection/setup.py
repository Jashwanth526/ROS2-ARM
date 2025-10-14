from setuptools import find_packages, setup

package_name = 'object_detection'

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
    maintainer='jashwanth',
    maintainer_email='kjashwanthk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_detector = object_detection.object_detector:main',
            'enhanced_task_server = object_detection.enhanced_task_server:main',
            'simple_task_server = object_detection.simple_task_server:main',
            'manipulation_client = object_detection.manipulation_client:main',
        ],
    },
)
