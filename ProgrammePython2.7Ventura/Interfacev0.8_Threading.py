"""
Test Interface I2C SpaceDog
Andrea Ventura 7/2/2020
"""
import Tkinter as tk
#//import RPi.GPIO as gpio    #only used with Rasp-pi
import serial
import time
import sys
import logging
import threading

class App:
    def __init__(self, master):
        
        self.master=master

        self.menu()

    def menu(self):
        self.FramePrincipale=tk.Frame(self.master,
                                      width=400,
                                      height=400
                                      )
        self.FramePrincipale.grid(row=0,column=0,rowspan=3,columnspan=3)
       
        self.boutQuitter = tk.Button(self.FramePrincipale, 
                                  text="QUITTER",
                                  fg="red",
                                  bg="grey",
                                  command=self.quitter)
        self.boutQuitter.place(x=200,y=350,anchor=tk.CENTER)
        
        self.boutonExtendMotorA1 = tk.Button(self.FramePrincipale, 
                                           text="Extend Motor A1",
                                           command=lambda: self.incrAngle(0,'A'))
        self.boutonExtendMotorA1.place(x=30,y=30)
        
        self.boutonRetractMotorA1 = tk.Button(self.FramePrincipale, 
                                           text="Retract Motor A1",
                                           command=lambda: self.decrAngle(0,'A'))
        self.boutonRetractMotorA1.place(x=170,y=30)

        self.boutonExtendMotorB1 = tk.Button(self.FramePrincipale, 
                                           text="Extend Motor B1",
                                           command=lambda: self.decrAngle(0,'B'))
        self.boutonExtendMotorB1.place(x=30,y=60)
        
        self.boutonRetractMotorB1 = tk.Button(self.FramePrincipale, 
                                           text="Retract Motor B1",
                                           command=lambda: self.incrAngle(0,'B'))
        self.boutonRetractMotorB1.place(x=170,y=60)

        self.boutonExtendMotorA2 = tk.Button(self.FramePrincipale, 
                                           text="Extend Motor A2",
                                           command=lambda: self.decrAngle(1,'A'))
        self.boutonExtendMotorA2.place(x=30,y=90)
        
        self.boutonRetractMotorA2 = tk.Button(self.FramePrincipale, 
                                           text="Retract Motor A2",
                                           command=lambda: self.incrAngle(1,'A'))
        self.boutonRetractMotorA2.place(x=170,y=90)

        self.boutonExtendMotorB2 = tk.Button(self.FramePrincipale, 
                                           text="Extend Motor B2",
                                           command=lambda: self.incrAngle(1,'B'))
        self.boutonExtendMotorB2.place(x=30,y=120)
        
        self.boutonRetractMotorB2 = tk.Button(self.FramePrincipale, 
                                           text="Retract Motor B2",
                                           command=lambda: self.decrAngle(1,'B'))
        self.boutonRetractMotorB2.place(x=170,y=120) 

        self.boutonExtendMotorA3 = tk.Button(self.FramePrincipale, 
                                           text="Extend Motor A3",
                                           command=lambda: self.decrAngle(2,'A'))
        self.boutonExtendMotorA3.place(x=30,y=150)
        
        self.boutonRetractMotorA3 = tk.Button(self.FramePrincipale, 
                                           text="Retract Motor A3",
                                           command=lambda: self.incrAngle(2,'A'))
        self.boutonRetractMotorA3.place(x=170,y=150)

        self.boutonExtendMotorB3 = tk.Button(self.FramePrincipale, 
                                           text="Extend Motor B3",
                                           command=lambda: self.incrAngle(2,'B'))
        self.boutonExtendMotorB3.place(x=30,y=180)
        
        self.boutonRetractMotorB3 = tk.Button(self.FramePrincipale, 
                                           text="Retract Motor B3",
                                           command=lambda: self.decrAngle(2,'B'))
        self.boutonRetractMotorB3.place(x=170,y=180)

        self.boutonExtendMotorA4 = tk.Button(self.FramePrincipale, 
                                           text="Extend Motor A4",
                                           command=lambda: self.incrAngle(3,'A'))
        self.boutonExtendMotorA4.place(x=30,y=210)
        
        self.boutonRetractMotorA4 = tk.Button(self.FramePrincipale, 
                                           text="Retract Motor A4",
                                           command=lambda: self.decrAngle(3,'A'))
        self.boutonRetractMotorA4.place(x=170,y=210)

        self.boutonExtendMotorB4 = tk.Button(self.FramePrincipale, 
                                           text="Extend Motor B4",
                                           command=lambda: self.decrAngle(3,'B'))
        self.boutonExtendMotorB4.place(x=30,y=240)
        
        self.boutonRetractMotorB4 = tk.Button(self.FramePrincipale, 
                                           text="Retract Motor B4",
                                           command=lambda: self.incrAngle(3,'B'))
        self.boutonRetractMotorB4.place(x=170,y=240)

        self.boutonUp = tk.Button(self.FramePrincipale, 
                                           text="All B+",
                                           command=self.getUp)
        self.boutonUp.place(x=30,y=270)

        self.boutonDown = tk.Button(self.FramePrincipale, 
                                           text="All B-",
                                           command=self.getDown)
        self.boutonDown.place(x=170,y=270)
        
        self.boutonFrontUp = tk.Button(self.FramePrincipale, 
                                           text="Front Up",
                                           command=self.frontUp)
        self.boutonFrontUp.place(x=170,y=310)
        
        self.setZero = tk.Button(self.FramePrincipale, 
                                           text="Set Zero",
                                           command=self.setZero)
        self.setZero.place(x=300,y=330)
        
        self.butThread = tk.Button(self.FramePrincipale, 
                                           text="Thread Check",
                                           command=check_thread)
        self.butThread.place(x=200,y=300)
       
    def quitter(self):
        for i in threadList:
            i.stop()
            i.join()
        
        root.destroy()
        
    def incrAngle(self,leg,motor):
        if motor == 'A':
            self.transmission(leg,1,0,5)
        elif motor == 'B':
            self.transmission(leg,1,1,5)
            
    def decrAngle(self,leg,motor):
        if motor == 'A':
            self.transmission(leg,1,0,-5)
        elif motor == 'B':
            self.transmission(leg,1,1,-5)
            
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
    """
    def transmission(self,adress,val1=0,val2=0,val3=0):
        if adress < 0:
            print("Sending: <%1d,%1d,%1d,%1d>" %(adress,val1,val2,val3))
            ser[adress].write("<%1d,%1d,%1d>" %(val1,val2,val3))

     
    def reception(self):
        #global root
        flagRecept = 0
        serBuffer = ""
        for i in range(0):
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
"""
class MyStoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, name, port):
        super(MyStoppableThread, self).__init__()
        self.name=name
        self.port=port
        self._stop_event = threading.Event()
        self._transmit_event = threading.Event()
        self.flagRecept = 0
        self.serBuffer = ''
    
    def run(self):
        logging.debug('Thread %s connecting to port %s',self.name, self.port)
        try:
            self.ser = serial.Serial(self.port, 9600, timeout = 0)
            if self.ser.isOpen() == False:
                self.ser.open()
            logging.debug("The port %s is available",self.port)
        except serial.serialutil.SerialException:
            logging.debug("Error: The port %s is not available",self.port)
            return
        time.sleep(1)
        #self.ser.write("<%1d,%1d,%1d>" %(0,0,0))
        
        while True:
            if self.checkIfStopped():
                logging.debug('Exiting')
                self.ser.close()
                break
            self.reception()
        
    def stop(self):
        self._stop_event.set()

    def checkIfStopped(self):
        return self._stop_event.is_set()
    
    def checkIfTransmit(self):
        return self._transmit_event.is_set()
    
    def transmission(self,val1,val2,val3):
        logging.debug("Sending: <%1d,%1d,%1d>" %(val1,val2,val3))
        self.ser.write("<%1d,%1d,%1d>" %(val1,val2,val3))

        
        
    def reception(self):
        #flagRecept = 0
        #serBuffer = ""
        while True:
            c = self.ser.read()
            #logging.debug("Received char %s", c)
            if len(c) == 0:
                break
            elif (c == ":" and self.flagRecept == 0):
                self.flagRecept = 1
            elif self.flagRecept == 1:
                if c == ";":
                    self.flagRecept = 0  
                    logging.debug("Received: %s", self.serBuffer)
                    if (self.serBuffer[0] == "0"):
                        "rien"
                    elif (self.serBuffer[0] == "1"):
                        "rien"
                    self.serBuffer = ""
                    break
                else:
                    self.serBuffer += c
            
    
        
        
"""  
def t_test(port,num):
    logging.debug('Connecting to port %s',port)
    try:
        ser = serial.Serial(port, 9600, timeout = 0)
        if ser.isOpen() == False:
            ser.open()
        logging.debug("The port %s is available",port)
    except serial.serialutil.SerialException:
        logging.debug("Error: The port %s is not available",port)
        return
    time.sleep(1)
    ser.write("<%1d,%1d,%1d>" %(0,0,0))
    flagRecept = 0
    serBuffer = ""
    while True:
        while True:
            c = ser.read()
            #logging.debug("Received char %s", c)
            if len(c) == 0:
                break
            elif (c == ":" and flagRecept == 0):
                flagRecept = 1
            elif flagRecept == 1:
                if c == ";":
                    flagRecept = 0  
                    logging.debug("Received string %s", serBuffer)
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
            t1.stopped()
    logging.debug('Exiting')
    ser.close()
"""    
def check_thread():
    for i in threading.enumerate():
        logging.debug('%s', i.getName())
        
def main():
    global app
    global root
    global threadList
    
    logging.basicConfig(level=logging.DEBUG,
                        format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                        )
    

    threadList = []
    #'COM8', 'COM6', 'COM10', 'COM9'
    #'/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3' with pi
    portList = ['COM8', 'COM6', 'COM10', 'COM9']
    for i in portList:
        t = MyStoppableThread(name='thread_leg'+str(portList.index(i)), port=i) 
        t.setDaemon(True)
        threadList.append(t)
        t.start()
    
    root=tk.Tk()
    root.title("Projet SpaceDog, The SPACE, Stage Andrea Ventura")
    app = App(root)
    root.mainloop()
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print 'Interrupted'
        #gpio.cleanup()     only for rasp-pi
        sys.exit(0)
