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
                                           command=lambda: self.extend(0,'A'))
        self.boutonExtendMotorA1.place(x=30,y=30)
        
        self.boutonRetractMotorA1 = Button(self.FramePrincipale, 
                                           text="Retract Motor A1",
                                           command=lambda: self.retract(0,'A'))
        self.boutonRetractMotorA1.place(x=170,y=30)

        self.boutonExtendMotorB1 = Button(self.FramePrincipale, 
                                           text="Extend Motor B1",
                                           command=lambda: self.extend(0,'B'))
        self.boutonExtendMotorB1.place(x=30,y=60)
        
        self.boutonRetractMotorB1 = Button(self.FramePrincipale, 
                                           text="Retract Motor B1",
                                           command=lambda: self.retract(0,'B'))
        self.boutonRetractMotorB1.place(x=170,y=60)

        self.boutonExtendMotorA2 = Button(self.FramePrincipale, 
                                           text="Extend Motor A2",
                                           command=lambda: self.extend(1,'A'))
        self.boutonExtendMotorA2.place(x=30,y=90)
        
        self.boutonRetractMotorA2 = Button(self.FramePrincipale, 
                                           text="Retract Motor A2",
                                           command=lambda: self.retract(1,'A'))
        self.boutonRetractMotorA2.place(x=170,y=90)

        self.boutonExtendMotorB2 = Button(self.FramePrincipale, 
                                           text="Extend Motor B2",
                                           command=lambda: self.extend(1,'B'))
        self.boutonExtendMotorB2.place(x=30,y=120)
        
        self.boutonRetractMotorB2 = Button(self.FramePrincipale, 
                                           text="Retract Motor B2",
                                           command=lambda: self.retract(1,'B'))
        self.boutonRetractMotorB2.place(x=170,y=120) 

        self.boutonExtendMotorA3 = Button(self.FramePrincipale, 
                                           text="Extend Motor A3",
                                           command=lambda: self.extend(2,'A'))
        self.boutonExtendMotorA3.place(x=30,y=150)
        
        self.boutonRetractMotorA3 = Button(self.FramePrincipale, 
                                           text="Retract Motor A3",
                                           command=lambda: self.retract(2,'A'))
        self.boutonRetractMotorA3.place(x=170,y=150)

        self.boutonExtendMotorB3 = Button(self.FramePrincipale, 
                                           text="Extend Motor B3",
                                           command=lambda: self.extend(2,'B'))
        self.boutonExtendMotorB3.place(x=30,y=180)
        
        self.boutonRetractMotorB3 = Button(self.FramePrincipale, 
                                           text="Retract Motor B3",
                                           command=lambda: self.retract(2,'B'))
        self.boutonRetractMotorB3.place(x=170,y=180)

        self.boutonExtendMotorA4 = Button(self.FramePrincipale, 
                                           text="Extend Motor A4",
                                           command=lambda: self.extend(3,'A'))
        self.boutonExtendMotorA4.place(x=30,y=210)
        
        self.boutonRetractMotorA4 = Button(self.FramePrincipale, 
                                           text="Retract Motor A4",
                                           command=lambda: self.retract(3,'A'))
        self.boutonRetractMotorA4.place(x=170,y=210)

        self.boutonExtendMotorB4 = Button(self.FramePrincipale, 
                                           text="Extend Motor B4",
                                           command=lambda: self.extend(3,'B'))
        self.boutonExtendMotorB4.place(x=30,y=240)
        
        self.boutonRetractMotorB4 = Button(self.FramePrincipale, 
                                           text="Retract Motor B4",
                                           command=lambda: self.retract(3,'B'))
        self.boutonRetractMotorB4.place(x=170,y=240)
        """
        self.boutonOneStep = Button(self.FramePrincipale, 
                                           text="1 step leg 1",
                                           command=self.oneStep)
        self.boutonOneStep.place(x=30,y=270)

        self.boutonThePas = Button(self.FramePrincipale, 
                                           text="the Pas",
                                           command=self.thePas)
        self.boutonThePas.place(x=30,y=310)

        self.boutonUp = Button(self.FramePrincipale, 
                                           text="All B+",
                                           command=self.getUp)
        self.boutonUp.place(x=170,y=310)

        self.boutonDown = Button(self.FramePrincipale, 
                                           text="All B-",
                                           command=self.getDown)
        self.boutonDown.place(x=250,y=310)
        """
        self.setZero = Button(self.FramePrincipale, 
                                           text="Set Zero",
                                           command=self.setZero)
        self.setZero.place(x=300,y=330)
       
    def quitter(self):
        """
        for i in range(0,4):
            print "Closing port " + port[i]
            ser[i].close()
        """   
        root.destroy()
        
    def extend(self,leg,motor):
        if motor == 'A':
            self.transmission(leg,1,0,1)
        elif motor == 'B':
            self.transmission(leg,1,1,1)
            
    def retract(self,leg,motor):
        if motor == 'A':
            self.transmission(leg,1,0,0)
        elif motor == 'B':
            self.transmission(leg,1,1,0)
    """
    def oneStep(self):
        self.transmission(self.address1,1)
        self.transmission(self.address1,9)
        self.transmission(self.address2,3)
        self.transmission(self.address2,11)

    def thePas(self):
        self.transmission(self.address1,5)
        time.sleep(1)
        self.transmission(self.address1,3)
        time.sleep(1)
        self.transmission(self.address1,7)
        time.sleep(1)

        self.transmission(self.address2,13)
        time.sleep(1)
        self.transmission(self.address2,9)
        time.sleep(1)
        self.transmission(self.address2,15)
        time.sleep(1)

        self.transmission(self.address1,13)
        time.sleep(1)
        self.transmission(self.address1,11)
        time.sleep(1)
        self.transmission(self.address1,15)
        time.sleep(1)

        self.transmission(self.address2,5)
        time.sleep(1)
        self.transmission(self.address2,1)
        time.sleep(1)
        self.transmission(self.address2,7)
        time.sleep(1)

        self.transmission(self.address1,1)
        self.transmission(self.address1,9)
        self.transmission(self.address2,3)
        self.transmission(self.address2,11)

    def getUp(self):
        self.transmission(self.address1,7)
        self.transmission(self.address1,15)
        time.sleep(1)
        self.transmission(self.address2,15)
        self.transmission(self.address2,7)

    def getDown(self):
        self.transmission(self.address2,5)
        self.transmission(self.address2,13)
        time.sleep(1)
        self.transmission(self.address1,13)
        self.transmission(self.address1,5)
    """
    def setZero(self):
        self.transmission(0,0)
        self.transmission(1,0)
        self.transmission(2,0)
        self.transmission(3,0)
    
    def transmission(self,adress,val1=0,val2=0,val3=0):
        print("<%1d,%1d,%1d,%1d>" %(adress,val1,val2,val3))
        ser[adress].write("<%1d,%1d,%1d>" %(val1,val2,val3))

     
    def reception(self):
        global root
        flagRecept = 0
        serBuffer = ""
        for i in range(0,4):
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
                        print i+1, serBuffer
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
                
        root.after(500, self.reception)
    
    
def main():
    global root
    global ser
    global port
    ser = [None] * 4
    # maybe 'COM8', 'COM6', 'COM10', 'COM9'
    port = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3'] 
    for i in range(0,4):
        try:
            ser[i] = serial.Serial(port[i], 9600, timeout = 0)
            if ser[i].isOpen() == False:
                ser[i].open()
            print "The port " + port[i] + " is available"

        except serial.serialutil.SerialException:
            print "The port " + port[i] + " is at use"

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
