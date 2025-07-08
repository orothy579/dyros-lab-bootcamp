from setuptools import setup

package_name = 'dyros_python'

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
    maintainer='dyros',
    maintainer_email='dyros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dyros_py_pub = dyros_python.dyros_py_pub:main',
            'dyros_py_sub = dyros_python.dyros_py_sub:main',
            'dyros_py_custom_pub = dyros_python.dyros_py_custom_pub:main',
            'dyros_py_custom_sub = dyros_python.dyros_py_custom_sub:main',
            'dyros_py_server = dyros_python.dyros_py_server:main',
            'dyros_py_client = dyros_python.dyros_py_client:main',
            'dyros_py_param = dyros_python.dyros_py_param:main',
        ],
    },
)
