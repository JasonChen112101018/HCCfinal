from setuptools import setup

package_name = 'tello_controller_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='Tello controller node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_controller_node_main = tello_controller_node.main:main',
        ],
    },
)
