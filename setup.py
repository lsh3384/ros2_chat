from setuptools import find_packages
from setuptools import setup

package_name = 'chat'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Lee',
    author_email='lsh3384@gmail.com',
    maintainer='Lee',
    maintainer_email='lsh3384@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS 2 chat',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chat_publisher = chat.chat_publisher:main',
            'chat_subscriber = chat.chat_subscriber:main',
        ],
    },
)
