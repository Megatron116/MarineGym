import torch

from marinegym.robots.robot import ASSET_PATH
from marinegym.robots.drone.underwaterVehicle import UnderwaterVehicle

class Spheroid(UnderwaterVehicle):

    usd_path: str = ASSET_PATH + "/usd/Spheroid/Spheroid.usd"
    param_path: str = ASSET_PATH + "/usd/Spheroid/Spheroid.yaml"
    fixed_usd_path: str = ASSET_PATH + "/usd/Spheroid/fixed_usd/Spheroid/Spheroid.usd"
