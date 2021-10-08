from setuptools import setup

package_name = 'config_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samsung',
    maintainer_email='dannylowithor@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking_gui = config_gui.parking_gui_node:main',
            'control_gui = config_gui.control_gui_node:main'
        ],
    },
)
