#!/usr/bin/python3

from dataclasses import dataclass
import pandas as pd
import matplotlib.pyplot as plt
from cmath import pi
import os


FILE="elbowAnglesRecording.txt"
FOLDER="experiment_data"

"""
NEW RECORDING: limite min ps
-----------------------------------
timestamp: 
  secs: 1674377074
  nsecs: 483919879
fe: 1.5200933198919662
ps: -0.8767175805114412
beta: 0.15799361512424623
"""

rec_types = ('FE_MAX','FE_MIN','PS_MAX','PS_MIN')
current_rec_type = 0

@dataclass
class Recordig:
    recType:str = 0
    t_secs:int = 0
    t_nsecs:int = 0
    fe:float = 0.0
    ps:float = 0.0
    beta:float = 0.0

rec_list = []

with open(FILE,"r") as f:
    rec = None
    for line in f.readlines(): 
        if line.startswith("timestamp"):
            continue
        elif line.strip().startswith("secs"):
            rec.t_secs = int(line.split(': ')[1].strip())
        elif line.strip().startswith("nsecs"):
            rec.t_nsecs = int(line.split(': ')[1].strip())
        elif line.strip().startswith("fe"):
            rec.fe = float(line.split(': ')[1].strip())*180/pi
        elif line.strip().startswith("ps"):
            rec.ps = float(line.split(': ')[1].strip())*180/pi
        elif line.strip().startswith("beta"):
            rec.beta = float(line.split(': ')[1].strip())*180/pi
        elif line.startswith("NEW"):
            if rec is None:
                rec = Recordig(recType=rec_types[0])
                current_rec_type = 0
            else: 
                rec_list.append(rec)
                current_rec_type = (current_rec_type + 1) % 4
                rec = Recordig(recType=rec_types[current_rec_type])
    rec_list.append(rec)

# print(rec_list)
t_init = rec_list[0].t_secs

file = f"{FOLDER}/data_{t_init}"
if not os.path.exists(file):
    os.makedirs(file)


df = pd.DataFrame(rec_list)
print(df)
df.to_csv(f"{file}/data_{t_init}.csv")

fe_max = {}
fe_min = {}
ps_max = {}
ps_min = {}

for rec in rec_list:
    if      rec.recType == 'FE_MAX': fe_max.update({rec.t_secs-t_init:rec.fe})
    elif    rec.recType == 'FE_MIN': fe_min.update({rec.t_secs-t_init:rec.fe})
    elif    rec.recType == 'PS_MAX': ps_max.update({rec.t_secs-t_init:rec.ps})
    elif    rec.recType == 'PS_MIN': ps_min.update({rec.t_secs-t_init:rec.ps})
    else: print("Unknown rec type")

plt.plot(fe_max.keys(),fe_max.values(), fe_min.keys(), fe_min.values())
plt.grid(True)
plt.title("Flexion-Extension limits over time")
plt.xlabel("time (s)")
plt.ylabel("angle (º)")
plt.legend(("fe_min","fe_max"))
plt.savefig(f"{file}/fe_{t_init}.png")
plt.close()

plt.plot(ps_max.keys(),ps_max.values(), ps_min.keys(), ps_min.values())
plt.grid(True)
plt.title("Pronation-Supination limits over time")
plt.xlabel("time (s)")
plt.ylabel("angle (rad)")
plt.legend(("ps_min","ps_max"))
plt.savefig(f"{file}/ps_{t_init}.png")
plt.close()
