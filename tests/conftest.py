# ===============================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: September 18th, 2020
#
# Description: Fixture class.
# ===============================================================
import pytest
import json
import datetime
import numpy as np
from context import Drone, Comms, Perception

@pytest.fixture
def lidar_data():
    # Import JSON
    # putting this into some object
    return [1,2,3]