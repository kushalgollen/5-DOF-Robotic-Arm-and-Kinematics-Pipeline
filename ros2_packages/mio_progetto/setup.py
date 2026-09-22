from setuptools import find_packages, setup

package_name = 'mio_progetto'

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
    maintainer='kgollen',
    maintainer_email='kgollen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'parlaparla = mio_progetto.mio_publisher:main',
            'ascoltamimf = mio_progetto.mio_subscriber:main',
            'test1 = mio_progetto.test1:main',
            'ikpy_node = mio_progetto.ikpy_to_rviz:main',
            'seedFisso = mio_progetto.test1_seedFix:main',
            'pinzaFissa = mio_progetto.test1_eeFix:main',
            'test_5dof = mio_progetto.test_moveit_5dof:main',
            'analytical = mio_progetto.analytical_sol:main',
        ],
    },
)
