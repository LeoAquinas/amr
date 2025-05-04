from setuptools import find_packages, setup

package_name = 'voice_control'

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
    maintainer='heisenburg',
    maintainer_email='wobblywulf@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_control = voice_control.voice_control:main',
            'whisper_voice_control = voice_control.whisper_voice_control:main',
            'gpt_voice_control = voice_control.gpt_voice_control:main',
        ],
    },
)
