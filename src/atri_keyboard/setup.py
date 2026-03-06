from setuptools import setup

package_name = 'atri_keyboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    
    maintainer='adore',
    maintainer_email='makochishima043@gmail.com',
    description='A package to get keyboard input to control the actions.',
    license='MIT',

    entry_points={
        'console_scripts': [
            'keyboard_node = atri_keyboard.keyboard_node:main',
        ],
    },
)