import os
import sys

# The code under test lives outside an installed package: scripts/ (the identified model, as used by the
# notebooks and the simulator) and ackermann_nmpc in the ROS 2 workspace (the NMPC). Both are importable
# straight from the repository, no colcon build needed.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'ackermann_nmpc'))
