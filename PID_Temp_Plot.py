#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 12 17:58:15 2023

@author: user1
"""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pylab import *
import pandas as pd

plt.style.use('fivethirtyeight')

x = []
data = []
counter = 0.
Dataname = "TempData_20-22,5_0,5K_P0,3_I0,02_D8_0,5s.csv"

def update(t):
    counter = 0
    for row in open(Dataname):
        counter += 1
    print(counter)
    #if(counter < 602):
    data = pd.read_csv(Dataname, skiprows = [x for x in range(1, 10)])
    x = data['t_values']
    y1 = data['T']
    #elif(counter > 601):
        #data = pd.read_csv(Dataname, skiprows = [x for x in range(1, counter -600)])
        #x = data['t_values']
        #y1 = data['T']
        
    plt.cla()
    #plt.tight_layout()
    plt.plot(x, y1)

    plt.xlabel("Regelschritte")
    plt.ylabel("Temperatur [°C]")


ani = FuncAnimation(plt.gcf(), update, interval=500)


plt.show()