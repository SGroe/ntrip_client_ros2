from setuptools import setup

package_name = 'ntrip_client'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name, 'resource/' + package_name + '.properties']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Groechenig @ Salzburg Research Forschungsgesellschaft mbH',
    maintainer_email='simon.groechenig@salzburgresearch.at',
    description='NTRIP client that sends NMEA-GGA messages to caster in return for RTCM correction messages.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ntrip_client = ntrip_client.ntripclient:main'
        ],
    },
)
