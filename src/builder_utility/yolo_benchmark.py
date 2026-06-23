import numpy as np
from tqdm import tqdm
import time
from enum import Enum
import psutil
import os

import matplotlib
matplotlib.use('Agg')
from ultralytics import YOLO

class TestingCases(Enum):
    PREDICT = 'Predicting'
    TRACKING = 'Tracking'
    
# ###### Change those ##############
TESTING = TestingCases.TRACKING
N_TRIALS = 200
N_WARMUP = 10
MODEL_PATH = "../skywarden_ai/static/models/yolo26n_640_int8_openvino_model"
TRACKER_CONFIG_PATH = "./tracker_config.yaml"
# ###### End Changable #############

process = psutil.Process(os.getpid())

before_ram = process.memory_info().rss / 1024**2
model = YOLO(MODEL_PATH)

dummy = np.zeros((640, 640, 3))

print("Warming up...")
for _ in tqdm(range(N_WARMUP)):
    model(dummy, verbose=False)
    
    
print("Starting benchmark...")

times = [] # seconds
for _ in tqdm(range(N_TRIALS)):
    start_t = time.perf_counter()
    
    if TESTING ==TestingCases.PREDICT:
        model(dummy, verbose=False)
    elif TESTING ==TestingCases.TRACKING:
        model.track(dummy,
            persist=True,
            tracker=TRACKER_CONFIG_PATH,
            verbose=False
        )
    
    times.append(time.perf_counter() - start_t)
    
times = np.array(times)
after_ram = process.memory_info().rss / 1024**2

print(f"Results of evaluating {TESTING}")
print("Times in seconds")
print(f"min: {times.min()}s")
print(f"max: {times.max()}s")
print(f"mean: {times.mean()}s")

print("FPS Conversion")
print(f"min: {1/times.max()} hz")
print(f"max: {1/times.min()} hz")
print(f"mean: {1/times.mean()} hz")

print("Model Footprint")
print(f"Before: {before_ram:.2f} MB")
print(f"After: {after_ram:.2f} MB")
print(f"Requires: {(after_ram - before_ram):.2f} MB")