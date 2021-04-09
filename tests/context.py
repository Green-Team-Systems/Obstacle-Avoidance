# ===============================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: September 6th, 2020
# 
# Description: Adds the necessary classes via an import module on
#              the path so that you can import modules in individual
#              tests.
# ===============================================================
import os
import sys
# Taken from https://docs.python-guide.org/writing/structure/
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from comms import Comms
from drone import Drone
from perception import Perception