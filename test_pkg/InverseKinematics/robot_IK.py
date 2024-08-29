#!/usr/bin/env python3
#Author: lnotspotl
import numpy as np
from math import sqrt, atan2, sin, cos, pi
from RobotUtilities.Transformations import homog_transform, homog_transform_inverse

class InverseKinematics(object):
    def __init__(self, bodyDimensions, legDimensions):

        self.body
