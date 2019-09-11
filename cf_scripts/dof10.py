# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg as lin
import os

os.system('clear')

n = 10
wn = np.zeros([n,1],float)
print(wn)
phi = np.zeros([n,1],float)
A = np.zeros([n,1],float)
sze = 1001;
hi = 20;
t = np.linspace(0,hi,sze)
print(t)
P = np.zeros([n,sze],float)
x = np.zeros([n,sze],float)

k = 1e2
print(k)
m = 2
x0 = np.zeros([n,1],float)
x0[0] = 3;
print(x0)
v0 = np.zeros([n,1]);

M = m*np.eye(n);
L = lin.sqrtm(M);
print(M)
print(L)
K = 2*k*np.eye(n)
K[n-1][n-1] = k

for i in range(0,n-1):
    K[i+1][i] = -1*k;
    K[i][i+1] = -1*k;
    
print(K)
Linv = lin.inv(L);
Ktilde = Linv@K@Linv
print(Ktilde)
D,V = lin.eig(Ktilde)
print(D)
print(V)


u = Linv@V;
print(u)
uinv = lin.inv(u)
p0 = uinv@x0
print(p0.shape)
pdot0 = uinv@v0

Mnew = np.transpose(u)@M@u
Knew = np.transpose(u)@K@u

for i in range(0,n):
    wn[i] = np.sqrt(Knew[i][i]/Mnew[i][i])
    phi[i] = np.arctan(p0[i]*wn[i]/pdot0[i])
    A[i] = np.sqrt(p0[i]**2 + (pdot0[i]/wn[i])**2)
    P[i][:] = A[i]*np.sin(np.multiply(wn[i],t) + phi[i])
print(phi)

x = u@P
for i in range(0,n):
    plt.plot(t,x[i][:])
    
plt.xlabel('Time (s)')
plt.ylabel('Displacement (m)')
plt.title('10 DOF System')
plt.show()
