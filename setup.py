from setuptools import find_packages, setup

package_name = 'ros2_project_sc22lv'

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
    maintainer='cscajb',
    maintainer_email='x.wang16@leeds.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'first_step = ros2_project_sc22lv.first_step:main',
            'second_step = ros2_project_sc22lv.second_step:main',
            'third_step = ros2_project_sc22lv.third_step:main',
            'fourth_step = ros2_project_sc22lv.fourth_step:main',
            'target_point = ros2_project_sc22lv.ros2025_targetpoint:main',
        ],
    },
)
