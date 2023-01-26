import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

FILE = "data_1674663174_gme_static/data_1674663174.csv"

def estimate_coef(x, y):
    # number of observations/points
    n = np.size(x)
  
    # mean of x and y vector
    m_x = np.mean(x)
    m_y = np.mean(y)
  
    # calculating cross-deviation and deviation about x
    SS_xy = np.sum(y*x) - n*m_y*m_x
    SS_xx = np.sum(x*x) - n*m_x*m_x
  
    # calculating regression coefficients
    b_1 = SS_xy / SS_xx
    b_0 = m_y - b_1*m_x
  
    return (b_0, b_1)

def plotWithLinReg(x,y,dataName,title):
    regression = estimate_coef(x,y)
    print(f"{title} linear regression:")
    print(f"\tRegression: {regression}")
    print(f"\tTendency in degrees per min: {regression[1]*60}")
    print(f"\tStandard deviation: {np.std(y)}")

    plt.scatter(x,y)
    plt.plot( (x[0], x[-1]), (regression[0]+regression[1]*x[0],regression[0]+regression[1]*x[-1]),'r')
    plt.grid(True)
    plt.title(f"{title} limits over time")
    plt.xlabel("time (s)")
    plt.ylabel("angle (º)")
    plt.legend((dataName,"regression"))
    plt.savefig(f"{dataName}_lin_reg.png")
    # plt.show()
    plt.close()

df = pd.read_csv(FILE)
print(df)
t0 = df.t_secs[0]

df_aux = df[(df['recType']=="FE_MAX")]
x = np.array([t-t0 for t in df_aux['t_secs']])
y = np.array(df_aux["fe"])
regression = plotWithLinReg(x,y,"fe_max","Flexion-Extension")
df_aux = None

df_aux = df[(df['recType']=="FE_MIN")]
x = np.array([t-t0 for t in df_aux['t_secs']])
y = np.array(df_aux["fe"])
regression = plotWithLinReg(x,y,"fe_min","Flexion-Extension")
df_aux = None

df_aux = df[(df['recType']=="PS_MAX")]
x = np.array([t-t0 for t in df_aux['t_secs']])
y = np.array(df_aux["ps"])
regression = plotWithLinReg(x,y,"ps_max","Pronation-Supination")
df_aux = None

df_aux = df[(df['recType']=="PS_MIN")]
x = np.array([t-t0 for t in df_aux['t_secs']])
y = np.array(df_aux["ps"])
regression = plotWithLinReg(x,y,"ps_min","Pronation-Supination")
df_aux = None

# x = np.array([t-df['t_secs'][0] for t in df['t_secs']])
# y = np.array([val for val in df['ps']])



# regression = estimate_coef(x,y)
# print(f"Pronation-Supination linear regression:\t{regression}")
# print(f"\tRegression: {regression}")
# print(f"\tTendency in degrees per min: {regression[1]*60}")
# print(f"\tStandard deviation: {np.std(y)}")