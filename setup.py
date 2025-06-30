from setuptools import find_packages, setup

package_name = 'multi_topic_monitor'

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
    maintainer='robolab',
    maintainer_email='tedusar@tugraz.at',
    description='Multi-topic monitoring node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_topic_monitor = multi_topic_monitor.multi_topic_monitor:main'
        ],
    },
)
