from setuptools import setup

package_name = 'rosbag_to_csv'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/rosbag_to_csv']),
    ('share/rosbag_to_csv', ['package.xml']),
    ('share/rosbag_to_csv/launch', ['launch/rosbag_to_csv_gui.launch.py']),
    # ('share/rosbag_to_csv/launch', ['launch/rosbag_to_csv_plot.launch.py']),
],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Diego H. Macias',
    maintainer_email='diegohmacias@tamu.edu',
    description='Convert ROS 2 bags to per-topic CSV files.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rosbag_to_csv = rosbag_to_csv.cli:main',
            'rosbag_to_csv_gui = rosbag_to_csv.gui:main',
            # 'rosbag_to_csv_plot = rosbag_to_csv.plot_gui:main',
        ],
    },
)
