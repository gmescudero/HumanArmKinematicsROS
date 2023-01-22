from datetime import datetime
from dataclasses import dataclass
import pandas as pd
from enum import IntEnum

FILE="elbowAnglesRecording.txt"

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

import subprocess as sb
#rosservice call /elbowAngles >> $FILE

# result = sb.run(['rosservice','call','/elbowAngles'],capture_output=True)

# print(result.stdout.split('\n'))

with open(FILE,"r") as f:
    rec = None
    for line in f.readlines(): 
        if line.startswith("timestamp"):
            continue
        elif line.strip().startswith("secs"):
            rec.t_secs = line.split(': ')[1].strip()
        elif line.strip().startswith("nsecs"):
            rec.t_nsecs = line.split(': ')[1].strip()
        elif line.strip().startswith("fe"):
            rec.fe = line.split(': ')[1].strip()
        elif line.strip().startswith("ps"):
            rec.ps = line.split(': ')[1].strip()
        elif line.strip().startswith("beta"):
            rec.beta = line.split(': ')[1].strip()
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
print(pd.DataFrame(rec_list))
df = pd.DataFrame(rec_list)
# df.to_csv("pollo.csv")