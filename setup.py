from setuptools import setup, find_packages

setup(
    name="stepmill",
    version="0.1.0",
    description="Open-source CAM tool for CNC milling and turning",
    author="Your Name",
    license="GPLv3",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "pyqt5",
        "pyqtgraph"
    ],
    entry_points={
    'console_scripts': [
        'stepmill=stepmill.ui.qt_viewer:main'
    ]
    },
    include_package_data=True,
    zip_safe=False
)
