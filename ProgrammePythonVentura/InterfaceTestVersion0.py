# -*- coding: utf-8 -*-
"""
Test Interface SpaceDog
Andrea Ventura 6/2/2020
"""

from Tkinter import *
import sys
import glob
import serial
import time

class App:
    def __init__(self, master, ser1, ser2, ser3, ser4):
        
        self.master=master
        self.ser1 = ser1
        self.ser2 = ser2
        self.ser3 = ser3
        self.ser4 = ser4
        self.ser1Buffer = ""
        self.ser2Buffer = ""
        self.ser3Buffer = ""
        self.ser4Buffer = ""
        
        self.menu()
        
    def menu(self):
        
        self.FramePrincipale=Frame(self.master,
                                      width=400,
                                      height=400
                                      )
        self.FramePrincipale.grid(row=0,column=0,rowspan=3,columnspan=3)
       
        self.boutQuitter = Button(self.FramePrincipale, 
                                  text="QUITTER",
                                  fg="red",
                                  bg="grey",
                                  command=self.quitter)
        self.boutQuitter.place(x=200,y=350,anchor=CENTER)
        
        self.boutonExtendMotorA = Button(self.FramePrincipale, 
                                           text="Extend Motor A",
                                           command=self.extendA)
        self.boutonExtendMotorA.place(x=100,y=100)
        
        self.boutonRetractMotorA = Button(self.FramePrincipale, 
                                           text="Retract Motor A",
                                           command=self.retractA)
        self.boutonRetractMotorA.place(x=200,y=100)
       
        
    def quitter(self):
        root.destroy()
        
    def extendA(self):
        self.transmission(1,1,0,255,0,2000)
        
    def retractA(self):
        self.transmission(1,0,0,255,0,2000)
    
    def transmission(self,flag,val1,val2,val3,val4,val5,val6=0):
        print("<%1d,%1d,%1d,%1d,%1d,%1d,%1d>" %(flag,val1,val2,val3,val4,val5,val6))
        self.ser.write("<%1d,%1d,%1d,%1d,%1d,%1d,%1d>" %(flag,val1,val2,val3,val4,val5,val6))
     
    def reception(self):
        global root
        flagRecept = 0
        while True:
            c = self.ser.read()
            
            if len(c) == 0:
                break
            
            elif (c == ":" and flagRecept == 0):
                #print c
                flagRecept = 1
            elif flagRecept == 1:
                #print c
                if c == ";":
                    flagRecept = 0  
                    print self.ser1Buffer
                    if (self.ser1Buffer[0] == "0"):
                        "rien"
                    elif (self.ser1Buffer[0] == "1"):
                        "rien"
                    elif (self.ser1Buffer[0] == "2"):
                        "rien"
                    elif (self.ser1Buffer[0] == "3"):
                        "rien"
                    elif (self.ser1Buffer[0] == "4"):
                        "rien"
                    self.ser1Buffer = ""
                    break
                else:
                    self.ser1Buffer += c
            else:
                "rien"
                
        root.after(100, self.reception)



def main():
    global root
    ser = serial.Serial()
    ser.port = '/dev/ttyACM1'
    ser.baudrate = 9600
    ser.timeout = 0
    # open port if not already open
    
    if ser.isOpen() == False:
        ser.open()
    
    root = Tk()
    root.title("Projet SpaceDog, The SPACE, Stage Andrea Ventura")
    app = App(root,ser1,ser2,ser3,ser4)
    root.after(10, app.reception)
    root.mainloop()
    
     
if __name__ == '__main__':
    main()
