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
from PIL import Image, ImageTk


class App:
    def __init__(self, master):
        
        self.master=master
        self.mainTransmitList = []
        self.threadList = []
        #'COM8', 'COM6', 'COM10', 'COM9'
        #'/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3' with pi
        self.portList = ['COM8', 'COM6', 'COM10', 'COM9']
        self._mainEventArray=[]
        
        self.h_Interface = 400
        self.l_Interface = 600
        
        self.target_time = time.time()
        self.current_time = time.time()
        
    
        self.image = Image.open("TheSpace.png")
        self.image.thumbnail((self.l_Interface*2/3,self.h_Interface*2/3))
        self.im_titre = ImageTk.PhotoImage(self.image)
        
        #self.im_titre = self.im_titre.subsample(x=2,y=2)
      
        
        for i in self.portList:
            q=Queue.Queue()
            e=threading.Event()
            t = MyStoppableThread(name='thread_leg'+str(self.portList.index(i)), port=i, queue=q, event=e) 
            t.setDaemon(True)
            self.threadList.append(t)
            t.start()
        
        for i in range(4):
            self._mainEventArray.append(threading.Event())
        
        self.menu()

    def menu(self):
        
        
        self.FramePrincipale=GradientFrame(self.master,
                                      width=self.l_Interface,
                                      height=self.h_Interface,
                                      color1='#e31440',
                                      color2='#5416c5',
                                      color3='#1e0036'
                                      )
        self.FramePrincipale.grid(row=0,column=0,rowspan=3,columnspan=3)

        self.lab_image_titre = tk.Label(self.FramePrincipale,
                                        bg='white',
                                        image=self.im_titre)
        self.lab_image_titre.place(x=0,y=0)
       
        self.boutQuitter = tk.Button(self.FramePrincipale, 
                                  text="QUITTER",
                                  fg="red",
                                  bg="grey",
                                  command=self.quitter)
        self.boutQuitter.place(x=self.l_Interface/2,y=self.h_Interface-20,anchor=tk.CENTER)
        
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
        
        self.boutonMove = tk.Button(self.FramePrincipale, 
                                           text="Fait un pas",
                                           command=self.unPas)
        self.boutonMove.place(x=30,y=360)
        
        self.boutonMiChemin = tk.Button(self.FramePrincipale, 
                                           text="Position neutre",
                                           command=self.positionNeutre)
        self.boutonMiChemin.place(x=170,y=360)

            
            
    def unPas(self):
        self.maint_transmission(2,2,1,10)
        self.maint_transmission(3,2,1,-10)
        self.delay_transmission(3)
        self.maint_transmission(0,2,1,-15)
        self.delay_transmission(1)
        self.maint_transmission(0,2,0,16)
        self.delay_transmission(1)
        self.maint_transmission(0,2,1,-31.18)
        self.delay_transmission(2)
        self.maint_transmission(2,2,1,31.18)
        self.maint_transmission(3,2,1,-31.18)

    def positionNeutre(self):
        self.maint_transmission(0,2,0,8.93)
        self.maint_transmission(1,2,0,-8.93)
        self.delay_transmission(4)
        self.maint_transmission(0,2,1,-31.18)
        self.maint_transmission(1,2,1,31.18)
        self.delay_transmission(4)
        self.maint_transmission(2,2,0,-8.93)
        self.maint_transmission(3,2,0,8.93)
        self.delay_transmission(4)
        self.maint_transmission(2,2,1,31.18)
        self.maint_transmission(3,2,1,-31.18)
    
    def frontUp(self):
        self.maint_transmission(0,1,1,-20)
        self.maint_transmission(1,1,1,20)
    
    def getUp(self):
        self.maint_transmission(0,1,1,-20)
        self.maint_transmission(1,1,1,20)
        self.delay_transmission(1)
        self.maint_transmission(2,1,1,20)
        self.maint_transmission(3,1,1,-20)

    def getDown(self):
        self.maint_transmission(3,1,1,20)
        self.maint_transmission(2,1,1,-20)
        self.delay_transmission(1)
        self.maint_transmission(1,1,1,-20)
        self.maint_transmission(0,1,1,20)
        
    def unPeudeTout(self):
        self.maint_transmission(0,2,1,-10)
        self.maint_transmission(1,2,1,10)
        self.delay_transmission(1)
        self.maint_transmission(0,2,0,5)
        self.maint_transmission(1,2,0,-5)
        self.delay_transmission(1)
        self.maint_transmission(2,2,1,10)
        self.maint_transmission(3,2,1,-10)
        self.delay_transmission(1)
        self.maint_transmission(2,2,0,-5)
        self.maint_transmission(3,2,0,5)
    
    def goTo_zero(self):
        self.maint_transmission(0,2,1,0)
        self.maint_transmission(1,2,1,0)
        self.delay_transmission(1)
        self.maint_transmission(0,2,0,0)
        self.maint_transmission(1,2,0,0)
        self.delay_transmission(1)
        self.maint_transmission(2,2,1,0)
        self.maint_transmission(3,2,1,0)
        self.delay_transmission(1)
        self.maint_transmission(2,2,0,0)
        self.maint_transmission(3,2,0,0)
    
    def setZero(self):
        self.maint_transmission(0,0)
        self.maint_transmission(1,0)
        self.maint_transmission(2,0)
        self.maint_transmission(3,0)
            
    def quitter(self):
        for i in self.threadList:
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
        
    def check_thread(self):
        for i in threading.enumerate():
            logging.debug('%s', i.getName())
            
    def check_event_thread(self,event):
        return self._mainEventArray[event].is_set()
    
    def maint_transmission(self,adress,val1=0,val2=0,val3=0):
        self.mainTransmitList.append([adress,val1,val2,val3])
    
    def delay_transmission(self,waitingTime):
        self.mainTransmitList.append(waitingTime)
        
    def tryTransmit(self):
        if (len(self.mainTransmitList) > 0):
            self.current_time = time.time()
            if self.current_time >= self.target_time:
                if isinstance(self.mainTransmitList[0],list):
                    a = self.mainTransmitList.pop(0)
                    logging.debug("Sending to %s: <%1d,%1d,%1d>" %(self.threadList[a[0]].name,a[1],a[2],a[3]))
                    self.threadList[a[0]].queue.put("<%1d,%1d,%1d>" %(a[1],a[2],a[3]))
                    self.threadList[a[0]]._transmit_event.set() 
                    self.threadList[a[0]].queue.join()
                else :
                    self.target_time = self.current_time+self.mainTransmitList.pop(0)
        
    def maint_reception(self): 
        for i in self._mainEventArray:
            if i.is_set():
                t=self.threadList[self._mainEventArray.index(i)]
                self.unBuffer = t.queue.get()
                t.queue.task_done()
                i.clear()
                logging.debug("rcvd from %s: %s",t.name,self.unBuffer)             
        
    def main_loop(self):
        self.tryTransmit()
        self.maint_reception()
        root.after(2, self.main_loop)

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
            if self._transmit_event.is_set():
                self.recept_from_interface()
                self.transmit_to_leg()
                
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
        
    def recept_from_interface(self):
        self.intrfBuffer = self.queue.get()
        self.queue.task_done()
        self._transmit_event.clear()
        
    def transmit_to_interface(self):
        self.queue.put(self.serBuffer)
        app._mainEventArray[app.threadList.index(self)].set()
        self.queue.join()
        self.serBuffer = ''
        
    def recept_from_leg(self):
        while True:
            c = self.ser.read()
            
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
                    #logging.debug("Received char %s", c)
                    self.serBuffer += c
                    
    def transmit_to_leg(self):
        logging.debug("Sending: %s" %(self.intrfBuffer))
        self.ser.write("%s" %(self.intrfBuffer))
        self.intrfBuffer = ''
        
class GradientFrame(tk.Canvas):
    '''A gradient frame which uses a canvas to draw the background'''
    def __init__(self, parent, color1="red", color2="black", color3="white", **kwargs):
        tk.Canvas.__init__(self, parent, **kwargs)
        self._color1 = color1
        self._color2 = color2
        self._color3 = color3
        self.bind("<Configure>", self._draw_gradient)

    def _draw_gradient(self, event=None):
        '''Draw the gradient'''
        self.delete("gradient")
        width = self.winfo_width()
        height = self.winfo_height()
        limit = width
        (r1,g1,b1) = self.winfo_rgb(self._color1)
        (r2,g2,b2) = self.winfo_rgb(self._color2)
        (r3,g3,b3) = self.winfo_rgb(self._color3)
        r12_ratio = float(r2-r1) / limit
        g12_ratio = float(g2-g1) / limit
        b12_ratio = float(b2-b1) / limit
        r23_ratio = float(r3-r2) / limit
        g23_ratio = float(g3-g2) / limit
        b23_ratio = float(b3-b2) / limit

        for i in range(width):
            nr = int(r1 + (r12_ratio * i))
            ng = int(g1 + (g12_ratio * i))
            nb = int(b1 + (b12_ratio * i))
            color = "#%4.4x%4.4x%4.4x" % (nr,ng,nb)
            #self.create_line(i,height,0,height-i, tags=("gradient",), fill=color)
            self.create_line(0,height-i*(height*1.0/(width*1.0)),i,height, tags=("gradient",), fill=color)
        
        for i in range(width):
            nr = int(r2 + (r23_ratio * i))
            ng = int(g2 + (g23_ratio * i))
            nb = int(b2 + (b23_ratio * i))
            color = "#%4.4x%4.4x%4.4x" % (nr,ng,nb)
            #self.create_line(i,height,0,height-i, tags=("gradient",), fill=color)
            self.create_line(width,height-i,i,0, tags=("gradient",), fill=color)
       
        self.lower("gradient")
        
def main():
    global app
    global root
    
    logging.basicConfig(level=logging.DEBUG,
                        format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                        )

    root=tk.Tk()
    root.title("Projet SpaceDog, The SPACE, Stage Andrea Ventura")
    app = App(root)
    root.after(10, app.main_loop)
    root.mainloop()
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print 'Interrupted'
        #gpio.cleanup()     only for rasp-pi
        sys.exit(0)
