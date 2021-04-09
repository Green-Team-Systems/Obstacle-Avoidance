import pytest
from context import Perception

def test_lidar_data_is_a_array(lidar_data):
    assert isinstance(lidar_data, list)