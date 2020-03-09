"""
Test Interface I2C SpaceDog
Andrea Ventura 7/2/2020
"""
from Tkinter import *

import serial
import time
import sys

class App:
    def __init__(self, master):
        
        self.master=master

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
        
        self.boutonExtendMotorA1 = Button(self.FramePrincipale, 
                                           text="Extend Motor A1",
                                           command=lambda: self.incrAngle(0,'A'))
        self.boutonExtendMotorA1.place(x=30,y=30)
        
        self.boutonRetractMotorA1 = Button(self.FramePrincipale, 
                                           text="Retract Motor A1",
                                           command=lambda: self.decrAngle(0,'A'))
        self.boutonRetractMotorA1.place(x=170,y=30)

        self.boutonExtendMotorB1 = Button(self.FramePrincipale, 
                                           text="Extend Motor B1",
                                           command=lambda: self.decrAngle(0,'B'))
        self.boutonExtendMotorB1.place(x=30,y=60)
        
        self.boutonRetractMotorB1 = Button(self.FramePrincipale, 
                                           text="Retract Motor B1",
                                           command=lambda: self.incrAngle(0,'B'))
        self.boutonRetractMotorB1.place(x=170,y=60)

        self.boutonExtendMotorA2 = Button(self.FramePrincipale, 
                                           text="Extend Motor A2",
                                           command=lambda: self.decrAngle(1,'A'))
        self.boutonExtendMotorA2.place(x=30,y=90)
        
        self.boutonRetractMotorA2 = Button(self.FramePrincipale, 
                                           text="Retract Motor A2",
                                           command=lambda: self.incrAngle(1,'A'))
        self.boutonRetractMotorA2.place(x=170,y=90)

        self.boutonExtendMotorB2 = Button(self.FramePrincipale, 
                                           text="Extend Motor B2",
                                           command=lambda: self.incrAngle(1,'B'))
        self.boutonExtendMotorB2.place(x=30,y=120)
        
        self.boutonRetractMotorB2 = Button(self.FramePrincipale, 
                                           text="Retract Motor B2",
                                           command=lambda: self.decrAngle(1,'B'))
        self.boutonRetractMotorB2.place(x=170,y=120) 

        self.boutonExtendMotorA3 = Button(self.FramePrincipale, 
                                           text="Extend Motor A3",
                                           command=lambda: self.decrAngle(2,'A'))
        self.boutonExtendMotorA3.place(x=30,y=150)
        
        self.boutonRetractMotorA3 = Button(self.FramePrincipale, 
                                           text="Retract Motor A3",
                                           command=lambda: self.incrAngle(2,'A'))
        self.boutonRetractMotorA3.place(x=170,y=150)

        self.boutonExtendMotorB3 = Button(self.FramePrincipale, 
                                           text="Extend Motor B3",
                                           command=lambda: self.incrAngle(2,'B'))
        self.boutonExtendMotorB3.place(x=30,y=180)
        
        self.boutonRetractMotorB3 = Button(self.FramePrincipale, 
                                           text="Retract Motor B3",
                                           command=lambda: self.decrAngle(2,'B'))
        self.boutonRetractMotorB3.place(x=170,y=180)

        self.boutonExtendMotorA4 = Button(self.FramePrincipale, 
                                           text="Extend Motor A4",
                                           command=lambda: self.incrAngle(3,'A'))
        self.boutonExtendMotorA4.place(x=30,y=210)
        
        self.boutonRetractMotorA4 = Button(self.FramePrincipale, 
                                           text="Retract Motor A4",
                                           command=lambda: self.decrAngle(3,'A'))
        self.boutonRetractMotorA4.place(x=170,y=210)

        self.boutonExtendMotorB4 = Button(self.FramePrincipale, 
                                           text="Extend Motor B4",
                                           command=lambda: self.decrAngle(3,'B'))
        self.boutonExtendMotorB4.place(x=30,y=240)
        
        self.boutonRetractMotorB4 = Button(self.FramePrincipale, 
                                           text="Retract Motor B4",
                                           command=lambda: self.incrAngle(3,'B'))
        self.boutonRetractMotorB4.place(x=170,y=240)

        self.boutonUp = Button(self.FramePrincipale, 
                                           text="All B+",
                                           command=self.getUp)
        self.boutonUp.place(x=30,y=270)

        self.boutonDown = Button(self.FramePrincipale, 
                                           text="All B-",
                                           command=self.getDown)
        self.boutonDown.place(x=170,y=270)
        
        self.boutonFrontUp = Button(self.FramePrincipale, 
                                           text="Front Up",
                                           command=self.frontUp)
        self.boutonFrontUp.place(x=170,y=310)
        
        self.setZero = Button(self.FramePrincipale, 
                                           text="Set Zero",
                                           command=self.setZero)
        self.setZero.place(x=300,y=330)
       
    def quitter(self):
        global openPort
        
        for i in range(openPort):
            print "Closing port " + port[i]
            ser[i].close()
        
        openPort = 0
        root.destroy()
        
    def incrAngle(self,leg,motor):
        if motor == 'A':
            self.transmission(leg,1,0,20)
        elif motor == 'B':
            self.transmission(leg,1,1,20)
            
    def decrAngle(self,leg,motor):
        if motor == 'A':
            self.transmission(leg,1,0,-20)
        elif motor == 'B':
            self.transmission(leg,1,1,-20)
            
    def frontUp(self):
        self.transmission(0,1,1,-20)
        self.transmission(1,1,1,20)
    
    def getUp(self):
        self.transmission(0,1,1,-20)
        self.transmission(1,1,1,20)
        time.sleep(1)
        self.transmission(2,1,1,20)
        self.transmission(3,1,1,-20)

    def getDown(self):
        self.transmission(3,1,1,20)
        self.transmission(2,1,1,-20)
        time.sleep(1)
        self.transmission(1,1,1,-20)
        self.transmission(0,1,1,20)
    
    def setZero(self):
        self.transmission(0,0)
        self.transmission(1,0)
        self.transmission(2,0)
        self.transmission(3,0)
    
    def transmission(self,adress,val1=0,val2=0,val3=0):
        if adress < openPort:
            print("Sending: <%1d,%1d,%1d,%1d>" %(adress,val1,val2,val3))
            ser[adress].write("<%1d,%1d,%1d>" %(val1,val2,val3))

     
    def reception(self):
        #global root
        flagRecept = 0
        serBuffer = ""
        for i in range(openPort):
            while True:
                c = ser[i].read()
                if len(c) == 0:
                    break
                
                elif (c == ":" and flagRecept == 0):
                    #print c
                    flagRecept = 1
                elif flagRecept == 1:
                    #print c
                    if c == ";":
                        flagRecept = 0  
                        print "Rcvd from leg", i,":", serBuffer
                        if (serBuffer[0] == "0"):
                            "rien"
                        elif (serBuffer[0] == "1"):
                            "rien"
                        elif (serBuffer[0] == "2"):
                            "rien"
                        elif (serBuffer[0] == "3"):
                            "rien"
                        serBuffer = ""
                        break
                    else:
                        serBuffer += c
                else:
                    "rien"
                
        root.after(200, self.reception)
    
    
def main():
    global root
    global ser
    global port
    global openPort
    openPort = 0
    ser = [None] * 4
    # maybe 'COM8', 'COM6', 'COM10', 'COM9'
    #'/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3'
    port = ['COM8', 'COM6', 'COM10', 'COM9'] 
    for i in range(0,4):
        try:
            ser[i] = serial.Serial(port[i], 9600, timeout = 0)
            if ser[i].isOpen() == False:
                ser[i].open()
            print "The port " + port[i] + " is available"
            openPort += 1

        except serial.serialutil.SerialException:
            print "The port " + port[i] + " is not available"

    root=Tk()
    root.title("Projet SpaceDog, The SPACE, Stage Andrea Ventura")
    app = App(root)
    root.after(10, app.reception)
    root.mainloop()
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print 'Interrupted'
        gpio.cleanup()
        sys.exit(0)
