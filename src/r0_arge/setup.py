from setuptools import find_packages, setup

package_name = 'r0_arge'

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
    maintainer='rag0n',
    maintainer_email='enes_trhn_8@outlook.com',
    description='TODO: Package description',
    license='Apache2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "stereo_example_rgbd_subscriber = r0_arge.stereo_example_rgbd_subscriber:main",
            "provider_rgbd_1 = r0_arge.provider_rgbd_1:main"
        ],
    },
)
