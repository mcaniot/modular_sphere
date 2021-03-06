import setuptools

setuptools.setup(
    name='modular_sphere',
    version='0.0.2',
    author="Maxime Caniot",
    author_email="maxime.caniot@gmail.com",
    description="Bullet-based simulation for a modular sphere robot",
    long_description_content_type="text/markdown",
    url="https://github.com/mcaniot/modular_sphere",
    packages=setuptools.find_packages(),
    install_requires=['pybullet', 'qibullet'],
    package_data={'modular_sphere': [
        "data/*.urdf",
        "data/*.mtl",
        "data/*.obj"]},
    keywords=[
        'physics simulation',
        'robotics',
        'modular sphere robot',
        'soft robot'],
    license='Creative Commons Attribution-Noncommercial-Share Alike license',
    long_description=open('README.md').read(),
    classifiers=[
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        'Intended Audience :: Science/Research',
        'Intended Audience :: Developers',
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS",
        "Operating System :: Microsoft",
        'Topic :: Games/Entertainment :: Simulation',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Framework :: Robot Framework :: Tool'
    ]
)
