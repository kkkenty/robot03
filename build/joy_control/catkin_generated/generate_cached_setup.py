# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/melodic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/melodic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/opt/ros/melodic;/home/robot03/robot03/devel'.split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

<<<<<<< HEAD:build/custom_msgs/catkin_generated/generate_cached_setup.py
code = generate_environment_script('/home/robot03/robot03/devel/.private/custom_msgs/env.sh')

output_filename = '/home/robot03/robot03/build/custom_msgs/catkin_generated/setup_cached.sh'
=======
code = generate_environment_script('/home/kento/robot03/devel/.private/joy_control/env.sh')

output_filename = '/home/kento/robot03/build/joy_control/catkin_generated/setup_cached.sh'
>>>>>>> dev:build/joy_control/catkin_generated/generate_cached_setup.py
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
