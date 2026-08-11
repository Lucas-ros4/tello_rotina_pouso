from setuptools import find_packages, setup

package_name = 'tello_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucas',
    maintainer_email='lucas@example.com',
    description='Pacote de controle do Tello com deteccao de ArUco',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_node = tello_control.tello_node:main',
            'tello_node_json = tello_control.tello_node_json:main',
            'tello_node_aruco = tello_control.tello_node_aruco:main',
        ],
    },
)
