from setuptools import find_packages, setup
import os, glob

package_name = 'box_sorter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share', package_name, "launch"), glob.glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, "config"), glob.glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhj',
    maintainer_email='happyijun@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'GUI = box_sorter.GUI:main',
            'GUI02 = box_sorter.GUI02:main',
            'img_publisher = box_sorter.img_publisher:main',
            'job_publisher = box_sorter.GUI_uuuj:main',
            #'img_sub_GUI = box_sorter.img_sub_GUI:main',
        ],
    },
)
