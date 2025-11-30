# Run this to generate all figs
import os
TASKS_PATH = os.path.abspath("interactiveSim")

div = "-"*25

task1_path = os.path.join(TASKS_PATH, "Task1.py")
print(f"Task 1{div}")
os.system(f"C:/Users/myspa/anaconda3/envs/autonomy/python.exe {task1_path}")
