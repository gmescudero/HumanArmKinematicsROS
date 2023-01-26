import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# FILE = "data_1674663174_gme_static/data_1674663174.csv"
FILE = "data_1674713370_gme_corrected/data_1674713370.csv"

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

def coefficient_of_determination(x,y,b0,b1):
    y_mean = np.mean(y)
    ssr = np.sum([(y_i - (b0 + b1*x_i))**2 for x_i,y_i in zip(x,y)])
    sst = np.sum([(y_i - y_mean)**2 for y_i in y])
    return 1 - (ssr/sst)


def plotWithLinReg(x,y,dataName,title,xlimits=None, ylimits=None):
    regression = estimate_coef(x,y)
    print(f"{title} linear regression:")
    print(f"\tRegression: {regression}")
    r2 = coefficient_of_determination(x,y,*regression)
    print(f"\tCoefficient of determination: {r2*100:.3f} %")
    print(f"\tTendency in degrees per min: {regression[1]*60:.3f}")
    print(f"\tStandard deviation: {np.std(y):.3f}")
    print(f"\tTotal deviation: {regression[1]*(x[0]-x[-1]):.3f}")

    plt.scatter(x,y)
    plt.plot( (x[0], x[-1]), (regression[0]+regression[1]*x[0],regression[0]+regression[1]*x[-1]),'r')
    plt.grid(True)
    plt.title(f"{title} limit over time")
    plt.xlabel("time (s)")
    plt.ylabel("angle (º)")
    if xlimits is not None: plt.xlim(xlimits)
    if ylimits is not None: plt.ylim(ylimits)
    plt.legend((f"regression (R2: {r2:.2f})",dataName))
    plt.savefig(f"{dataName}_lin_reg.png")
    # plt.show()
    plt.close()

df = pd.read_csv(FILE)
print(df)
t0 = df.t_secs[0]

df_aux = df[(df['recType']=="FE_MAX")]
x = np.array([t-t0 for t in df_aux['t_secs']])
y = np.array(df_aux["fe"])
regression = plotWithLinReg(x,y,"fe_max","Flexion-Extension",ylimits=(40,165))
df_aux = None

df_aux = df[(df['recType']=="FE_MIN")]
x = np.array([t-t0 for t in df_aux['t_secs']])
y = np.array(df_aux["fe"])
regression = plotWithLinReg(x,y,"fe_min","Flexion-Extension",ylimits=(10,-130))
df_aux = None

df_aux = df[(df['recType']=="PS_MAX")]
x = np.array([t-t0 for t in df_aux['t_secs']])
y = np.array(df_aux["ps"])
regression = plotWithLinReg(x,y,"ps_max","Pronation-Supination",ylimits=(-85,-20))
df_aux = None

df_aux = df[(df['recType']=="PS_MIN")]
x = np.array([t-t0 for t in df_aux['t_secs']])
y = np.array(df_aux["ps"])
regression = plotWithLinReg(x,y,"ps_min","Pronation-Supination",ylimits=(60,110))
df_aux = None

# x = np.array([t-df['t_secs'][0] for t in df['t_secs']])
# y = np.array([val for val in df['ps']])



# regression = estimate_coef(x,y)
# print(f"Pronation-Supination linear regression:\t{regression}")
# print(f"\tRegression: {regression}")
# print(f"\tTendency in degrees per min: {regression[1]*60}")
# print(f"\tStandard deviation: {np.std(y)}")