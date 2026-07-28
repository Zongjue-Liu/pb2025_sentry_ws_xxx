from setuptools import setup

package_name = 'pb_control_panel'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robomaster',
    maintainer_email='todo@example.com',
    description='Qt control panel for simulating PB referee topics.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pb_control_panel = pb_control_panel.gui:main',
        ],
    },
)
