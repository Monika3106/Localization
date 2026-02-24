from setuptools import setup

package_name = 'localization_kf'

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
    maintainer='Monika Nagarajrao',
    maintainer_email='mon4565s@hs-coburg.de',
    description='Kalman filter based localization node using OptiTrack mocap data.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # this matches how you run it: `ros2 run localization_kf localization`
            'localization = localization_kf.localization:main',
        ],
    },
)

