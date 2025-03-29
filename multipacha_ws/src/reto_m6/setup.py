from setuptools import find_packages, setup

package_name = 'reto_m6'

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
    maintainer='saul',
    maintainer_email='sjnd1029@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reto_topic_pub = reto_m6.reto_topic_pub:main',
            'reto_topic_sub = reto_m6.reto_topic_sub:main',
        ],
    },
)
