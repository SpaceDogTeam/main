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
        
        self.w_Interface = 900
        self.h_Interface = self.w_Interface*2/3
        
        
        self.target_time = time.time()
        
    
        with Image.open("TheSpace.png") as self.image:
            self.image.thumbnail((self.w_Interface*2/3,self.h_Interface*2/3))
            self.im_titre = ImageTk.PhotoImage(self.image)
        
        with Image.open("SchemaChien.png") as self.image:
            self.image.thumbnail((self.w_Interface*1/3,self.h_Interface*3/5))
            self.im_schChien = ImageTk.PhotoImage(self.image)
            
        with Image.open("Dog.png") as self.image:
            self.image.thumbnail((self.w_Interface*1/5,self.h_Interface*1/12))
            self.im_titChien = ImageTk.PhotoImage(self.image)
            
        with Image.open("Doggy.png") as self.image:
            self.image.thumbnail((self.w_Interface*1/4,self.h_Interface*1/5))
            self.im_desChien = ImageTk.PhotoImage(self.image)
        
        
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
        
        offset=self.w_Interface/30

        self.FramePrincipale=GradientFrame(self.master,
                                      width=self.w_Interface,
                                      height=self.h_Interface,
                                      color1='#e31440',
                                      color2='#5416c5',
                                      color3='#1e0036'
                                      )
        self.FramePrincipale.grid(row=0,column=0,rowspan=3,columnspan=3)
        
        self.FramePrincipale.create_rectangle(offset,offset,
                                      self.w_Interface-offset,
                                      self.h_Interface-offset,
                                      fill='white',
                                      outline='black',
                                      width=2)
       
        self.FramePrincipale.create_image(self.w_Interface/2,offset+5,
                                          anchor=tk.N, image=self.im_titre)
        
        self.FramePrincipale.create_image(self.w_Interface-offset*3/2,self.h_Interface-offset*3/2,
                                          anchor=tk.SE, image=self.im_schChien)
        
        self.FramePrincipale.create_image(self.w_Interface-offset,2*offset,
                                          anchor=tk.NE, image=self.im_desChien)
        
        self.FramePrincipale.create_image(self.w_Interface-offset*26/5,offset,
                                          anchor=tk.NW, image=self.im_titChien)
        
        self.FrameEntry = tk.Frame(self.FramePrincipale,
                                     width=400,
                                     height=150,
                                     bg='green'
                                     )
        self.FrameEntry.place(x=2*offset,y=5*offset)
        
        self.EntryCommand = tk.Entry(self.FrameEntry,width=60)
        self.EntryCommand.place(x=5 ,y=5)
        
        self.buttonSend = tk.Button(self.FrameEntry,text = 'Send',
                                    command = self.sendEntry)
        self.buttonSend.place(x=5,y=25)
        
        self.FrameBoutons1 = tk.Frame(self.FramePrincipale,
                                     width=100,
                                     height=218,
                                     bg='black'
                                     )
        self.FrameBoutons1.place(x=self.w_Interface*5/9-offset*3/2,y=self.h_Interface*2/7)
        
        for i in range(16):
            jambe = i//4
            moteur = i//2
            moteur = moteur%2
            direction = i%2
            direction != direction 
            if moteur == 0:
                moteurL = 'A'
            else:
                moteurL = 'B'   
            if direction == 0:
                signe = '-'
            else:
                signe = '+'
            tk.Button(self.FrameBoutons1,
                                   text='%s%d%s'%(moteurL,jambe,signe),
                                   width = 5,
                                   command=lambda jambe=jambe,moteur=moteur,direction=direction: self.changeAngle(jambe,moteur,direction)
                      
                        ).place(x=(1-direction)*(offset*3/2)+5,y=(offset-4)*(i//2)+5)

        self.FrameBoutons2 = tk.Frame(self.FramePrincipale,
                                     width=230,
                                     height=100,
                                     bg='red'
                                     )
        self.FrameBoutons2.place(x=self.w_Interface*1/3,y=self.h_Interface*2/3)
        
        self.boutonUp = tk.Button(self.FrameBoutons2, 
                                           text="All B+",
                                           command=self.getUp)
        self.boutonUp.place(x=0,y=26)

        self.boutonDown = tk.Button(self.FrameBoutons2, 
                                           text="All B-",
                                           command=self.getDown)
        self.boutonDown.place(x=43,y=26)
        
        self.boutonFrontUp = tk.Button(self.FrameBoutons2, 
                                           text="Front Up",
                                           command=self.frontUp)
        self.boutonFrontUp.place(x=83,y=26)
        
        self.setZero = tk.Button(self.FrameBoutons2, 
                                           text="Set Zero",
                                           command=self.setZero)
        self.setZero.place(x=0,y=0)
        
        self.butThread = tk.Button(self.FrameBoutons2, 
                                           text="Thread Check",
                                           command=self.check_thread)
        self.butThread.place(x=124,y=0)
        
        self.boutongoToZero = tk.Button(self.FrameBoutons2, 
                                           text="Go To Zero",
                                           command=self.goTo_zero)
        self.boutongoToZero.place(x=54,y=0)
        
        self.boutonMiChemin = tk.Button(self.FrameBoutons2, 
                                           text="Position neutre",
                                           command=self.positionNeutre)
        self.boutonMiChemin.place(x=0,y=52)
        
        
        
        self.boutQuitter = tk.Button(self.FrameBoutons2, 
                                  text="QUITTER",
                                  fg="red",
                                  bg="grey",
                                  command=self.quitter)
        self.boutQuitter.place(x=100,y=70)
        
    def sendEntry(self):
        val = ['','','','']
        flagNewTram = 0
        entryBuffer = self.EntryCommand.get()
        for i in range(len(entryBuffer)):
            if (entryBuffer[i] == '<' and flagNewTram == 0):
                flagNewTram = 1
                n = 0
            elif flagNewTram == 1:
                if entryBuffer[i] == '>':
                    val[n] = float(val[n])
                    #print ("<%s,%s,%s,%s>" %(val[0],val[1],val[2],val[3]))
                    self.maint_transmission(val[0],val[1],val[2],val[3])
                    val = ['','','','']
                    flagNewTram = 0
                elif (entryBuffer[i] == ',') :
                    #print val[n]
                    val[n] = int(val[n])
                    n += 1
                else :
                    val[n] += entryBuffer[i]
                    
    
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
    
    def changeAngle(self,leg,motor,direction):
        if ((leg == 0 or leg == 3) and motor == 1 or (leg == 1 or leg == 2) and motor == 0):
            direction = 1 - direction
        if (direction == 0):
            self.maint_transmission(leg,1,motor,-5)
        else:
            self.maint_transmission(leg,1,motor,5)
        
    def check_thread(self):
        for i in threading.enumerate():
            logging.debug('%s', i.getName())
            
    def check_event_thread(self,event):
        return self._mainEventArray[event].is_set()
    
    def maint_transmission(self,adress,val1=0,val2=0,val3=0):
        self.mainTransmitList.append([adress,str(val1),str(val2),str(val3)])
    
    def delay_transmission(self,waitingTime):
        self.mainTransmitList.append(waitingTime)
        
    def tryTransmit(self):
        if (len(self.mainTransmitList) > 0):
            current_time = time.time()
            if current_time >= self.target_time:
                if isinstance(self.mainTransmitList[0],list):
                    a = self.mainTransmitList.pop(0)
                    logging.debug("Sending to %s: <%s,%s,%s>" %(self.threadList[a[0]].name,a[1],a[2],a[3]))
                    self.threadList[a[0]].queue.put("<%s,%s,%s>" %(a[1],a[2],a[3]))
                    self.threadList[a[0]]._transmit_event.set() 
                    self.threadList[a[0]].queue.join()
                else :
                    self.target_time = current_time+self.mainTransmitList.pop(0)
        
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
