"""
Code authored by Keegan Kelly
"""
from tracker import Tracker
import requests
import pandas as pd
import numpy as np
from math import ceil
import time

tracker = Tracker(marker_width=0.09, aruco_type="DICT_4X4_1000")
# starts threads for reading frames, outputing frames, processing frames, and sending data to the server
tracker.startThreads()
