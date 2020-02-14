"""
Test Interface I2C SpaceDog
Andrea Ventura 7/2/2020
"""
from Tkinter import *
import RPi.GPIO as gpio
import smbus
import time
import sys

class App:
    def __init__(self, master):
        
        self.master=master
        
        self.bus = smbus.SMBus(1)
        self.address1 = 0x04
        self.address2 = 0x05

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
                                           command=lambda: self.extend(1,'A'))
        self.boutonExtendMotorA1.place(x=30,y=30)
        
        self.boutonRetractMotorA1 = Button(self.FramePrincipale, 
                                           text="Retract Motor A1",
                                           command=lambda: self.retract(1,'A'))
        self.boutonRetractMotorA1.place(x=170,y=30)

        self.boutonExtendMotorB1 = Button(self.FramePrincipale, 
                                           text="Extend Motor B1",
                                           command=lambda: self.extend(1,'B'))
        self.boutonExtendMotorB1.place(x=30,y=60)
        
        self.boutonRetractMotorB1 = Button(self.FramePrincipale, 
                                           text="Retract Motor B1",
                                           command=lambda: self.retract(1,'B'))
        self.boutonRetractMotorB1.place(x=170,y=60)

        self.boutonExtendMotorA2 = Button(self.FramePrincipale, 
                                           text="Extend Motor A2",
                                           command=lambda: self.extend(2,'A'))
        self.boutonExtendMotorA2.place(x=30,y=90)
        
        self.boutonRetractMotorA2 = Button(self.FramePrincipale, 
                                           text="Retract Motor A2",
                                           command=lambda: self.retract(2,'A'))
        self.boutonRetractMotorA2.place(x=170,y=90)

        self.boutonExtendMotorB2 = Button(self.FramePrincipale, 
                                           text="Extend Motor B2",
                                           command=lambda: self.extend(2,'B'))
        self.boutonExtendMotorB2.place(x=30,y=120)
        
        self.boutonRetractMotorB2 = Button(self.FramePrincipale, 
                                           text="Retract Motor B2",
                                           command=lambda: self.retract(2,'B'))
        self.boutonRetractMotorB2.place(x=170,y=120) 

        self.boutonExtendMotorA3 = Button(self.FramePrincipale, 
                                           text="Extend Motor A3",
                                           command=lambda: self.extend(3,'A'))
        self.boutonExtendMotorA3.place(x=30,y=150)
        
        self.boutonRetractMotorA3 = Button(self.FramePrincipale, 
                                           text="Retract Motor A3",
                                           command=lambda: self.retract(3,'A'))
        self.boutonRetractMotorA3.place(x=170,y=150)

        self.boutonExtendMotorB3 = Button(self.FramePrincipale, 
                                           text="Extend Motor B3",
                                           command=lambda: self.extend(3,'B'))
        self.boutonExtendMotorB3.place(x=30,y=180)
        
        self.boutonRetractMotorB3 = Button(self.FramePrincipale, 
                                           text="Retract Motor B3",
                                           command=lambda: self.retract(3,'B'))
        self.boutonRetractMotorB3.place(x=170,y=180)

        self.boutonExtendMotorA4 = Button(self.FramePrincipale, 
                                           text="Extend Motor A4",
                                           command=lambda: self.extend(4,'A'))
        self.boutonExtendMotorA4.place(x=30,y=210)
        
        self.boutonRetractMotorA4 = Button(self.FramePrincipale, 
                                           text="Retract Motor A4",
                                           command=lambda: self.retract(4,'A'))
        self.boutonRetractMotorA4.place(x=170,y=210)

        self.boutonExtendMotorB4 = Button(self.FramePrincipale, 
                                           text="Extend Motor B4",
                                           command=lambda: self.extend(4,'B'))
        self.boutonExtendMotorB4.place(x=30,y=240)
        
        self.boutonRetractMotorB4 = Button(self.FramePrincipale, 
                                           text="Retract Motor B4",
                                           command=lambda: self.retract(4,'B'))
        self.boutonRetractMotorB4.place(x=170,y=240)

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

        self.setZero = Button(self.FramePrincipale, 
                                           text="Set Zero",
                                           command=self.setZero)
        self.setZero.place(x=300,y=330)
        
    def quitter(self):
        root.destroy()
        
    def extend(self,leg,motor):
        if leg == 1:
            if motor == 'A':
                self.transmission(self.address1,3)
            elif motor == 'B':
                self.transmission(self.address1,7)
        elif leg == 2:
            if motor == 'A':
                self.transmission(self.address1,11)
            elif motor == 'B':
                self.transmission(self.address1,15)
        elif leg == 3:
            if motor == 'A':
                self.transmission(self.address2,3)
            elif motor == 'B':
                self.transmission(self.address2,7)
        elif leg == 4:
            if motor == 'A':
                self.transmission(self.address2,11)
            elif motor == 'B':
                self.transmission(self.address2,15)
            
    def retract(self,leg,motor):
        if leg == 1:
            if motor == 'A':
                self.transmission(self.address1,1)
            elif motor == 'B':
                self.transmission(self.address1,5)
        elif leg == 2:
            if motor == 'A':
                self.transmission(self.address1,9)
            elif motor == 'B':
                self.transmission(self.address1,13)
        elif leg == 3:
            if motor == 'A':
                self.transmission(self.address2,1)
            elif motor == 'B':
                self.transmission(self.address2,5)
        elif leg == 4:
            if motor == 'A':
                self.transmission(self.address2,9)
            elif motor == 'B':
                self.transmission(self.address2,13)

    def transmission(self,address,value):
        self.bus.write_byte(address,value)
        self.reception(address)

    def reception(self,address):
        print "Arduino answer to RPI:", self.bus.read_byte(address)

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

    def setZero(self):
        self.transmission(self.address1,16)
        self.transmission(self.address2,16)
        
def main():
    global root
    gpio.setmode(gpio.BCM)
    root=Tk()
    root.title("Projet SpaceDog, The SPACE, Stage Andrea Ventura")
    app = App(root)
    time.sleep(1)
    root.mainloop()
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print 'Interrupted'
        gpio.cleanup()
        sys.exit(0)
