#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun  6 16:39:37 2023

@author: user1
"""

import csv
import numpy as np
#from pylab import *
import Adafruit_MCP4725
import time
import sys
sys.path.append("/home/user1/Adafruit_Python_ADS1x15")
import Adafruit_ADS1x15


dac = Adafruit_MCP4725.MCP4725()
adc = Adafruit_ADS1x15.ADS1115()

class PID:
    dt = 0.
    kp = 0.
    kd = 0.
    ki = 0.
    err = 0.
    dT = 0.
    count = 0.
    Int = 0.
    
    def __init__(self, dt, kp, kd, ki, dT, Offset):   
        self.dt = dt                                 # dt = Zeit in s
        self.kp = kp                                 # kp = Proportionalkonstante
        self.kf = kd                                 # kd = Differentialkonstante
        self.ki = ki                                 # ki = Integralkonstante
        self.dT = dT                                 # dT = Integrationszeit in s
        self.Offset = Offset
        self.err = 0
        
    def correct(self, set, act, range1, range2, range3, range4, Out_max, Out_min):
        e = set - act
        
        P = self.kp * e
        
        if(range2 < act) and (act < range1):
            print("in Range (Int)")
            self.Int += self.err
            I = self.ki * self.dt * self.Int
        else:
            self.Int = 0
            I = 0.
            
        if(range4 < act < range3):
            print("in Range (Diff)")
            D = -self.kd * ((e-self.err)/(self.dt))
        else:
            D = 0.
            
        out = P+I+D+self.Offset
        if(out > Out_max):
            o = Out_max
        elif(out<Out_min):
            o = Out_min
        else:
            o = out
        
        print("P")
        print(P)
        print("I")
        print(I)
        print("D")
        print(D)
        
        self.err = e
        return o 
    
def read(Gain):
    # Gain-Regeln
    # 2/3 = +/-6.144V
    # 1 = +/-4.096V
    # 2 = +/-2.048V
    # 4 = +/-1.024V
    # 8 = +/-0.512V
    # 16 = +/-0.256V
    #
    #unit = " V"
    #if GAIN == 4 or GAIN == 8 or GAIN == 16:
        #unit = "mV"
    GAIN = Gain
    values = [0]*4
    voltages = [0]*4
    for i in range(4):
        values[i] = adc.read_adc(i, gain=GAIN)

    
    for i in range(4):
        if GAIN == 2/3:
            voltages[i] = ((values[i]/32767)*6144)/1000
        elif GAIN == 1:
            voltages[i] = ((values[i]/32767)*4096)/1000
        elif GAIN == 2:
            voltages[i] = ((values[i]/32767)*2048)/1000
        elif GAIN == 4:
            voltages[i] = (values[i]/32767)*1024*0.001
        elif GAIN == 8:
            voltages[i] = (values[i]/32767)*512*0.001
        elif GAIN == 16:
            voltages[i] = (values[i]/32767)*256*0.001
    
    return voltages

def write(factor, vorfactor): #Gibt Spannung auf den OPA
    f = factor - vorfactor    #Differenz aus Faktor und Vorfaktor um Flanken zu glätten bei Spannungsänderung
    b = 200                   #Glättiterationen
    r = 1/b
    for i in range(b):
        dac.set_voltage(int((vorfactor + i * r * f)*4096))
    
def main():
    Dataname = "Test.csv"
    P = 0.2
    I = 0.01
    D = 8
    Leitungswiderstandsabfall = 0
    sweep = True
    EinfachSweep = False     # Von T1 bis T2
    MehrfachSweep = True  # Von T1 nach T2 bis T1
    EinfachIterationen = 1  # Sweepiterationen (Sollte int Wert sein)
    MehrfachIterationen = 1
    fieldnames = ["t_values", "T"]
    Termaltime = 60.
    Kschritte = 5.                     #Kelvinschritte
    t_value = 0.
    factor = 0.
    vorfactor = 0.
    Gain = 8
    volts_in =  read(Gain)[0]/2 - Leitungswiderstandsabfall   #Leitungswiderstand = 0.004V
    V_Start = 2.5                           #Anfänglicher Ausgangswert
    V_max = 3.                           #maximaler Ausgangswert
    V_min = 2.                            #minimaler Ausgangswert
    print("V_Start:")
    print(V_Start)
    T1 = 15.                          #sweepwert1 (wird behandelt wie Tsoll, wenn sweep aus ist)
    T2 = 35.                          #sweepwert2 (Tritt nur in Kraft, wenn sweep an ist)
    RangeInt = 4.
    RangeDiff = 0.5
    RangeInt1 = T1 +RangeInt                         #Temperaturrange in welcher Int und Diff geschaltet wird
    RangeInt2 = T1 -RangeInt
    RangeDiff1 = T1 +RangeDiff
    RangeDiff2 = T1 -RangeDiff
        
        
    #factorgoal = (Voltgoal/5)
    timestep = 0.5
    Integrationtime = 10 * timestep
    u = PID(timestep, P , D, I, Integrationtime, V_Start)
    A = 3.9083 * 10 **(-3)                   # Konstanten entspringen aus Linearisierung vom PT00 und 
    Aover2 = 1.95415*10**(-3)                # aus der Lösung von quadratischen Gleichungen oder 
    Aover2squared = Aover2**2                # Gleichungen 4ten Grades
    OneoverB = -0.17316*10**(7)
    B = -5.775*10**(-7)
    C = -4.183*10**(-12)
    minusBoverfourA = 10/4
    p = B/C - (3/8)*10**4
    q = -(1/8)*10**6 + (100/8)*(B/C)+A/C
    
    if (volts_in >= 0.1):
        T = OneoverB * (-Aover2+np.sqrt(Aover2squared-B+(10*volts_in * B)))
    else:
        T = (10*volts_in -1)/0.003851
        
    write(factor, vorfactor)
        
    
    
    with open(Dataname, "w") as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames = fieldnames)
        csv_writer.writeheader()
    
    if(T < RangeInt2) or (T > RangeInt1):
        print("T:")
        print(T)
        print("Fahre zu T1")
        write(1/2, 1/2)
        time.sleep(10)
    while(T < RangeInt2) or (T > RangeInt1):
        with open(Dataname, 'a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames= fieldnames)
            
            info = {
                't_values': t_value,
                'T': T
                }
            
            csv_writer.writerow(info)
            
            
        volts_in =  read(Gain)[0]/2 - Leitungswiderstandsabfall    #Leitungswiderstand
        
        if (volts_in >= 0.1):
            T = OneoverB * (-Aover2+np.sqrt(Aover2squared-B+(10*volts_in * B)))
        else:
            T = (10*volts_in -1)/0.003851
            
        t_value += timestep
        
        print("---------------------")
        print("T")
        print(T)
        print("T_soll")
        print(T1)
        V_Out = u.correct(T1, T, RangeInt1, RangeInt2, RangeDiff1, RangeDiff2, V_max, V_min)
        
        #if(V_Out < Voltgoal):
        vorfactor = factor
        factor = V_Out/5
            
            
        print("V_Out:")
        print(V_Out)
        print("factor:")
        print(factor)
        print("---------------------")
                                                    # Peltierschutz
        if(factor <= 1/2 and vorfactor >= 1/2):     # factor > 1/2 = positiver Strom und factor <1/2 = negativer Strom
            write(1/2, 1/2)                         # Strom auf 0
            time.sleep(1)                           # 1 Sekunde warten
        elif(factor >= 1/2 and vorfactor <= 1/2):
            write(1/2, 1/2)
            time.sleep(1)
        
        
        write(factor, vorfactor)
        
        time.sleep(timestep)
    
    if (sweep == False):
        RangeInt1 = T1 +RangeInt                          #Temperaturrange in welcher Int und Diff geschaltet wird
        RangeInt2 = T1 -RangeInt
        RangeDiff1 = T1 +RangeDiff
        RangeDiff2 = T1 -RangeDiff
            
        print("T_soll")
        print(T1)
        
        while True:
            
            with open(Dataname, 'a') as csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames= fieldnames)
                
                info = {
                    't_values': t_value,
                    'T': T
                    }
                
                csv_writer.writerow(info)
                
                
            volts_in =  read(Gain)[0]/2 - Leitungswiderstandsabfall    #Leitungswiderstand
            
            if (volts_in >= 0.1):
                T = OneoverB * (-Aover2+np.sqrt(Aover2squared-B+(10*volts_in * B)))
            else:
                T = (10*volts_in -1)/0.003851
                
            t_value += timestep
            
            print("---------------------")
            print("T")
            print(T)
            print("T_soll")
            print(T1)
            V_Out = u.correct(T1, T, RangeInt1, RangeInt2, RangeDiff1, RangeDiff2, V_max, V_min)
            
            #if(V_Out < Voltgoal):
            vorfactor = factor
            factor = V_Out/5
            #else:
                #vorfactor = factor
                #factor = 
                
                
            print("V_Out:")
            print(V_Out)
            print("factor:")
            print(factor)
            print("---------------------")
                                                        # Peltierschutz
            if(factor <= 1/2 and vorfactor >= 1/2):     # factor > 1/2 = positiver Strom und factor <1/2 = negativer Strom
                write(1/2, 1/2)                         # Strom auf 0
                time.sleep(1)                           # 1 Sekunde warten
            elif(factor >= 1/2 and vorfactor <= 1/2):
                write(1/2, 1/2)
                time.sleep(1)
            
            
            write(factor, vorfactor)
            
            time.sleep(timestep)
            
    if(sweep == True):
        if(T2-T1 < 0):
            i = int(T1-T2)
        else:
            i = int(T2-T1)
            
        if(EinfachSweep == True):
            for I in range(EinfachIterationen):
                for p in range(int(i/Kschritte)+1):
                    Tsoll = T1 + Kschritte * i * (p/i)
                    print('Tsoll')
                    print(Tsoll)
                    
                    
                    RangeInt1 = Tsoll +RangeInt                          #Temperaturrange in welcher Int und Diff geschaltet wird
                    RangeInt2 = Tsoll -RangeInt
                    RangeDiff1 = Tsoll +RangeDiff
                    RangeDiff2 = Tsoll -RangeDiff
                        
                    endtime = time.time() + Termaltime
                    
                    while(time.time() < endtime):
                        with open(Dataname, 'a') as csv_file:
                            csv_writer = csv.DictWriter(csv_file, fieldnames= fieldnames)
                            
                            info = {
                                't_values': t_value,
                                'T': T
                                }
                            
                            csv_writer.writerow(info)
                            
                            
                        volts_in =  read(Gain)[0]/2 - Leitungswiderstandsabfall    #Leitungswiderstand
                        
                        if (volts_in >= 0.1):
                            T = OneoverB * (-Aover2+np.sqrt(Aover2squared-B+(10*volts_in * B)))
                        else:
                            T = (10*volts_in -1)/0.003851
                            
                        t_value += timestep
                        
                        print("---------------------")
                        print("T")
                        print(T)
                        print("T_soll")
                        print(Tsoll)
                        V_Out = u.correct(Tsoll, T, RangeInt1, RangeInt2, RangeDiff1, RangeDiff2, V_max, V_min)
                        
                        #if(V_Out < Voltgoal):
                        vorfactor = factor
                        factor = V_Out/5
                        #else:
                            #vorfactor = factor
                            #factor = 
                            
                            
                        print("V_Out:")
                        print(V_Out)
                        print("factor:")
                        print(factor)
                        print("---------------------")
                                                                    # Peltierschutz
                        if(factor < 1/2 and vorfactor > 1/2):         # factor > 1/2 = positiver Strom und factor <1/2 = negativer Strom
                            write(1/2, 1/2)                         # Strom auf 0
                            time.sleep(1)                           # 1 Sekunde warten
                        elif(factor > 1/2 and vorfactor < 1/2):
                            write(1/2, 1/2)
                            time.sleep(1)
                        
                        
                        write(factor, vorfactor)
                        
                        time.sleep(timestep)
            
                    
        if(MehrfachSweep == True):
            for I in range(MehrfachIterationen):
                for p in range(0,int(i/Kschritte)+1):
                    Tsoll = T1 + Kschritte * i * (p/i)
                    print('Tsoll')
                    print(Tsoll)
                    
                    
                    RangeInt1 = Tsoll +RangeInt                          #Temperaturrange in welcher Int und Diff geschaltet wird
                    RangeInt2 = Tsoll -RangeInt
                    RangeDiff1 = Tsoll +RangeDiff
                    RangeDiff2 = Tsoll -RangeDiff
                        
                    endtime = time.time() + Termaltime
                    
                    while(time.time() < endtime):
                        with open(Dataname, 'a') as csv_file:
                            csv_writer = csv.DictWriter(csv_file, fieldnames= fieldnames)
                            
                            info = {
                                't_values': t_value,
                                'T': T
                                }
                            
                            csv_writer.writerow(info)
                            
                            
                        volts_in =  read(Gain)[0]/2 - Leitungswiderstandsabfall    #Leitungswiderstand
                        
                        if (volts_in >= 0.1):
                            T = OneoverB * (-Aover2+np.sqrt(Aover2squared-B+(10*volts_in * B)))
                        else:
                            T = (10*volts_in -1)/0.003851
                            
                        t_value += timestep
                        
                        print("---------------------")
                        print("T")
                        print(T)
                        print("T_soll")
                        print(Tsoll)
                        V_Out = u.correct(Tsoll, T, RangeInt1, RangeInt2, RangeDiff1, RangeDiff2, V_max, V_min)
                        
                        #if(V_Out < Voltgoal):
                        vorfactor = factor
                        factor = V_Out/5
                        #else:
                            #vorfactor = factor
                            #factor = 
                            
                            
                        print("V_Out:")
                        print(V_Out)
                        print("factor:")
                        print(factor)
                        print("---------------------")
                                                                    # Peltierschutz
                        if(factor < 1/2 and vorfactor > 1/2):         # factor > 1/2 = positiver Strom und factor <1/2 = negativer Strom
                            write(1/2, 1/2)                         # Strom auf 0
                            time.sleep(1)                           # 1 Sekunde warten
                        elif(factor > 1/2 and vorfactor < 1/2):
                            write(1/2, 1/2)
                            time.sleep(1)
                        
                        
                        write(factor, vorfactor)
                        
                        time.sleep(timestep)
                        
                for p in reversed(range(0,int(i/Kschritte))):
                    Tsoll = T1 + Kschritte * i * (p/i)
                    print('Tsoll')
                    print(Tsoll)
                    
                    
                    RangeInt1 = Tsoll +RangeInt                          #Temperaturrange in welcher Int und Diff geschaltet wird
                    RangeInt2 = Tsoll -RangeInt
                    RangeDiff1 = Tsoll +RangeDiff
                    RangeDiff2 = Tsoll -RangeDiff
                        
                    endtime = time.time() + Termaltime
                    
                    while(time.time() < endtime):
                        with open(Dataname, 'a') as csv_file:
                            csv_writer = csv.DictWriter(csv_file, fieldnames= fieldnames)
                            
                            info = {
                                't_values': t_value,
                                'T': T
                                }
                            
                            csv_writer.writerow(info)
                            
                            
                        volts_in =  read(Gain)[0]/2 - Leitungswiderstandsabfall    #Leitungswiderstand
                        
                        if (volts_in >= 0.1):
                            T = OneoverB * (-Aover2+np.sqrt(Aover2squared-B+(10*volts_in * B)))
                        else:
                            T = (10*volts_in -1)/0.003851
                            
                        t_value += timestep
                        
                        print("---------------------")
                        print("T")
                        print(T)
                        print("T_soll")
                        print(Tsoll)
                        V_Out = u.correct(Tsoll, T, RangeInt1, RangeInt2, RangeDiff1, RangeDiff2, V_max, V_min)
                        
                        #if(V_Out < Voltgoal):
                        vorfactor = factor
                        factor = V_Out/5
                        #else:
                            #vorfactor = factor
                            #factor = 
                            
                            
                        print("V_Out:")
                        print(V_Out)
                        print("factor:")
                        print(factor)
                        print("---------------------")
                                                                    # Peltierschutz
                        if(factor < 1/2 and vorfactor > 1/2):         # factor > 1/2 = positiver Strom und factor <1/2 = negativer Strom
                            write(1/2, 1/2)                         # Strom auf 0
                            time.sleep(1)                           # 1 Sekunde warten
                        elif(factor > 1/2 and vorfactor < 1/2):
                            write(1/2, 1/2)
                            time.sleep(1)
                        
                        
                        write(factor, vorfactor)
                        
                        time.sleep(timestep)
    vorfactor = factor
    factor = 1/2
    
    if(factor <= 1/2 and vorfactor >= 1/2):     # factor > 1/2 = positiver Strom und factor <1/2 = negativer Strom
        write(1/2, 1/2)                         # Strom auf 0
        time.sleep(1)                           # 1 Sekunde warten
    elif(factor >= 1/2 and vorfactor <= 1/2):
        write(1/2, 1/2)
        time.sleep(1)
            
    write (1/2, 1/2)        
        
                
    
        
    

main()