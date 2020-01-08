# Modular Sphere project [![Build Status](https://travis-ci.com/mcaniot/modular_sphere.svg?branch=master)](https://travis-ci.com/mcaniot/modular_sphere)  [![codecov](https://codecov.io/gh/mcaniot/modular_sphere/branch/master/graph/badge.svg)](https://codecov.io/gh/mcaniot/modular_sphere)


The project is to create a modular soft robot able to adapt and collaborate in swarm.

## Install

Use the setup.py to install the libraries:

`sudo python setup.py install`

## Examples

Teleoperate the modular sphere with the keyboard arrow keys:

`python examples/teleop_sphere.py`

Train the sphere to navigate to a specific goal (need to install [stable_baselines](https://stable-baselines.readthedocs.io/en/master/guide/install.html) with specific version of Tensorflow (from 1.8.0 to 1.14.0)):

`python examples/train_sphere_baselines.py`

You can see the result with the models pretrained:

`python examples/enjoy_sphere_baselines.py`
