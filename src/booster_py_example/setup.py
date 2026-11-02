from setuptools import find_packages, setup

package_name = 'booster_py_example'

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
    maintainer='seonghwi',
    maintainer_email='ham9301@naver.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "custom_arm_example = booster_py_example.custom_arm_example:main",
            "keyboard = booster_py_example.keyboard:main",
            "joint_state_sub = booster_py_example.joint_state_sub:main",
        ],
    },
)
