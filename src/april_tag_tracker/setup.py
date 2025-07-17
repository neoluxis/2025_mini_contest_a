from setuptools import find_packages, setup
from glob import glob
package_name = 'april_tag_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neolux',
    maintainer_email='neolux_lee@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            f"videocap = {package_name}.videocap:main",
            f"april_tag_tracker = {package_name}.april_tag_tracker:main",
            f"serial = {package_name}.serial_sender:main"
        ],
    },
)
