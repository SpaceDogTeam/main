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
import Queue

class App:
    def __init__(self, master):
        
        self.master=master
        self._mainEventArray=[]
        for i in range(4):
            self._mainEventArray.append(threading.Event())
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
                                           command=self.check_thread)
        self.butThread.place(x=200,y=300)
        
        self.boutondetout = tk.Button(self.FramePrincipale, 
                                           text="Un peu de tout",
                                           command=self.unPeudeTout)
        self.boutondetout.place(x=30,y=300)
        
        self.boutongoToZero = tk.Button(self.FramePrincipale, 
                                           text="Go To Zero",
                                           command=self.goTo_zero)
        self.boutongoToZero.place(x=30,y=330)
       
    def quitter(self):
        for i in threadList:
            i.stop()
            i.join()
        
        root.destroy()
        
    def incrAngle(self,leg,motor):
        if motor == 'A':
            self.maint_transmission(leg,1,0,5)
        elif motor == 'B':
            self.maint_transmission(leg,1,1,5)
            
    def decrAngle(self,leg,motor):
        if motor == 'A':
            self.maint_transmission(leg,1,0,-5)
        elif motor == 'B':
            self.maint_transmission(leg,1,1,-5)
            
    def frontUp(self):
        self.maint_transmission(0,1,1,-20)
        self.maint_transmission(1,1,1,20)
    
    def getUp(self):
        self.maint_transmission(0,1,1,-20)
        self.maint_transmission(1,1,1,20)
        time.sleep(1)
        self.maint_transmission(2,1,1,20)
        self.maint_transmission(3,1,1,-20)

    def getDown(self):
        self.maint_transmission(3,1,1,20)
        self.maint_transmission(2,1,1,-20)
        time.sleep(1)
        self.maint_transmission(1,1,1,-20)
        self.maint_transmission(0,1,1,20)
        
    def unPeudeTout(self):
        self.maint_transmission(0,2,1,-10)
        self.maint_transmission(1,2,1,10)
        time.sleep(1)
        self.maint_transmission(0,2,0,5)
        self.maint_transmission(1,2,0,-5)
        time.sleep(1)
        self.maint_transmission(2,2,1,10)
        self.maint_transmission(3,2,1,-10)
        time.sleep(1)
        self.maint_transmission(2,2,0,-5)
        self.maint_transmission(3,2,0,5)
    
    def goTo_zero(self):
        self.maint_transmission(0,2,1,0)
        self.maint_transmission(1,2,1,0)
        time.sleep(1)
        self.maint_transmission(0,2,0,0)
        self.maint_transmission(1,2,0,0)
        time.sleep(1)
        self.maint_transmission(2,2,1,0)
        self.maint_transmission(3,2,1,0)
        time.sleep(1)
        self.maint_transmission(2,2,0,0)
        self.maint_transmission(3,2,0,0)
    
    def setZero(self):
        self.maint_transmission(0,0)
        self.maint_transmission(1,0)
        self.maint_transmission(2,0)
        self.maint_transmission(3,0)
        
    def check_thread(self):
        for i in threading.enumerate():
            logging.debug('%s', i.getName())
            
    def check_event_thread(self,event):
        return self._mainEventArray[event].is_set()
    
    def maint_transmission(self,adress,val1=0,val2=0,val3=0):
        #try:
        logging.debug("Sending to %s: <%1d,%1d,%1d>" %(threadList[adress],val1,val2,val3))
        threadList[adress].queue.put("<%1d,%1d,%1d>" %(val1,val2,val3))
        threadList[adress].set_event_interface()
        threadList[adress]._transmit_event.set()
        threadList[adress].queue.join()
    def maint_reception(self): 
        for i in self._mainEventArray:
            if i.is_set():
                t=threadList[self._mainEventArray.index(i)]
                self.unBuffer = t.queue.get()
                logging.debug("rcvd from %s: %s",t,self.unBuffer)
                t.queue.task_done()
                i.clear()
        root.after(10, self.maint_reception)

class MyStoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition."""

    def __init__(self, name, port, queue, event):
        super(MyStoppableThread, self).__init__()
        self.name=name
        self.port=port
        self.queue=queue
        self._stop_event = threading.Event()
        self._transmit_event = event
        self.flagRecept = 0
        self.serBuffer = ''
        self.intrfBuffer = ''
        self.val1=0
        self.val2=0
        self.val3=0
        self.flagGotSmthgFromLeg = 0
    
    def run(self):
        logging.debug('Connecting to port %s',self.port)
        try:
            self.ser = serial.Serial(self.port, 9600, timeout = 0)
            if self.ser.isOpen() == False:
                self.ser.open()
            logging.debug("Connected to %s",self.port)
        except serial.serialutil.SerialException:
            logging.debug("Error: The port %s is not available",self.port)
            return
        time.sleep(1)

        while True:
            if self.checkIfStopped():
                logging.debug('Exiting')
                self.ser.close() 
                break
            if self.check_event_interface():
                self.recept_from_interface()
                self.transmit_to_leg()
                self._transmit_event.clear()
            else:
                self.recept_from_leg()
                if (self.flagGotSmthgFromLeg == 0):
                    """rien"""
                else:
                    self.transmit_to_interface()
                    self.flagGotSmthgFromLeg = 0
        
    def stop(self):
        self._stop_event.set()

    def checkIfStopped(self):
        return self._stop_event.is_set()
    
    def set_event_interface(self):
        self._transmit_event.set()
        
    def check_event_interface(self):
        return self._transmit_event.is_set()
    
    def recept_from_interface(self):
        self.intrfBuffer = self.queue.get()
        self.queue.task_done()
        
    def transmit_to_interface(self):
        self.queue.put(self.serBuffer)
        app._mainEventArray[threadList.index(self)].set()
        self.queue.join()
        self.serBuffer = ''
        
    def recept_from_leg(self):
        while True:
            c = self.ser.read()
            #logging.debug("Received char %s", c)
            if len(c) == 0:
                break
            elif (c == ":" and self.flagRecept == 0):
                self.flagRecept = 1
            elif self.flagRecept == 1:
                if c == ";":
                    logging.debug("Received: %s", self.serBuffer)
                    self.flagGotSmthgFromLeg = 1
                    self.flagRecept = 0
                    break
                else:
                    self.serBuffer += c
                    
    def transmit_to_leg(self):
        logging.debug("Sending: %s" %(self.intrfBuffer))
        self.ser.write("%s" %(self.intrfBuffer))
        
        self.intrfBuffer = ''
        
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
        q=Queue.Queue()
        e=threading.Event()
        t = MyStoppableThread(name='thread_leg'+str(portList.index(i)), port=i, queue=q, event=e) 
        t.setDaemon(True)
        threadList.append(t)
        t.start()
    
    
    
    root=tk.Tk()
    root.title("Projet SpaceDog, The SPACE, Stage Andrea Ventura")
    app = App(root)
    root.after(10, app.maint_reception)
    root.mainloop()
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print 'Interrupted'
        #gpio.cleanup()     only for rasp-pi
        sys.exit(0)
