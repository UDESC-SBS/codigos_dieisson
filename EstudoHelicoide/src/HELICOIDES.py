
import numpy as np


def FK3RHSMTH(theta, s, s0):
    T = np.eye(4)
    for i in range(3):
      
        a11 = np.cos(theta[i])
        a12 = -np.sin(theta[i])
        a13 = 0
        a21 = np.sin(theta[i])
        a22 = np.cos(theta[i])
        a23 = 0
        a31 = 0
        a32 = 0
        a33 = 1
        
        a14 = -s0[1,i]*a11-1
        a24 = -s0[1,i]*a21
        a34 = 0
        
        a41 = 0
        a42 = 0
        a43 = 0
        a44 = 1
        H = np.array([
            [a11, a12, a13, a14],
            [a21,a22, a23, a24],
            [a31, a32, a33, a34],
            [a41, a42, a43, a44]
        ])
        print(-s0[1,i])
        T = T @ H
    return T

theta = [np.pi/2, 0.0, 0.0] # angulos em radianos
s = np.array([
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
        ])

s0 = np.array([
            [0, 0, 0],
            [-30, 0, 100],
            [-30, 0, 200],
        ])
T=FK3RHSMTH(theta,s,s0)
print(T)