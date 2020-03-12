# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 16:43:59 2020

@author: Chevron
"""
#---Livrairies---
import tkinter as tk
import logging
from tkinter import *
from PIL import Image, ImageTk

#---Class interface--- // main frame
class interface(tk.Frame):
    
    #var
    w_Interface = 900
    h_Interface = 2/3*w_Interface
    master = None
    
    #var commande robot
    legSelect = [0,0,0,0]
    
    
    #construct
    def __init__(self, master):
        tk.Frame.__init__(self,master)
        self.config()
        self.widget()
        
    #config application (size...)
    def config(self):
        self.w_Interface = 900
        self.h_Interface = 2/3*self.w_Interface
        
        self.widget()        
        
    #widget creation
    def widget(self):
        #main frame
        self.FramePrincipale=GradientFrame(self.master)
        self.FramePrincipale.grid(row=0,column=0,rowspan=3,columnspan=3)
        
        self.defineTab()
        
        self.onglet1()
        self.onglet2()
        self.onglet3()
        
       #open & place img 
#        with Image.open("TheSpace.png") as self.image:
#            self.image.thumbnail((self.w_Interface*2/3,self.h_Interface*2/3))
#            self.im_titre = ImageTk.PhotoImage(self.image)
#self.FramePrincipale.create_image(self.w_Interface/2,50, anchor=tk.N, image=self.im_titre)
        
    # Tab creation
    def defineTab(self):    
        self.tabs = ttk.Notebook(self.FramePrincipale,width=int(self.w_Interface), height=int(self.h_Interface))
        
        self.Onglet1 = tk.Frame(self.master)
        self.Onglet2 = tk.Frame(self.master)
        self.Onglet3 = tk.Frame(self.master)
        #Affichage QUALI
        #GradientFrame(self.master,
#                        color1='#e31440',
#                        color2='#5416c5',
#                        color3='#1e0036'
#                        )
        #tk.Label(Onglet1, wraplength='4i', justify=tk.LEFT, anchor=tk.N, text="Yo !").pack()
        
        self.tabs.add(self.Onglet1,text='Main')
        self.tabs.add(self.Onglet2,text='Preconfig')
        self.tabs.add(self.Onglet3,text='Debug')
        self.tabs.pack()
        
    def onglet1(self):
        #Port management - check function defPortCom
        self.buttonCOM = tk.Button(self.Onglet1, text = "Port COM",command=self.defPortCom)
        self.buttonCOM.grid(row = 0, column = 0, padx = 1/20*self.w_Interface,pady = 1/20*self.h_Interface)
        
        #List of command
        self.listCommande = tk.Listbox(self.Onglet1)
        self.listCommande.insert(1,"Neutre")
        self.listCommande.insert(2,"Assis")
        self.listCommande.insert(3,"couche")
        self.listCommande.grid(row = 0, column = 1)

    def defPortCom(self):
        t = tk.Toplevel(self.master,width=int(self.w_Interface)/3, height=int(self.h_Interface)*2/3)
        t.title("Gestion des ports COM")
        msg = tk.Label(t,text = "Port COM utilisé : ")
        msg.grid(row=0,column=0)
        leaveButton = tk.Button(t, text = "quit", command = t.destroy)
        leaveButton.grid(row=3,column=0)
        
        
    def onglet2(self):
        #Button legs choice (List ?)
        self.buttonLeg0 = tk.Button(self.Onglet2, text="leg0", command=self.selectLeg0)
        self.buttonLeg1 = tk.Button(self.Onglet2, text="leg1", command=self.selectLeg1)
        self.buttonLeg2 = tk.Button(self.Onglet2, text="leg2", command=self.selectLeg2)
        self.buttonLeg3 = tk.Button(self.Onglet2, text="leg3", command=self.selectLeg3)
        self.buttonLeg0.grid(row = 0, column = 0, padx = 1/80*self.w_Interface,pady = 1/80*self.h_Interface)
        self.buttonLeg1.grid(row = 1, column = 0, padx = 1/80*self.w_Interface,pady = 1/80*self.h_Interface)
        self.buttonLeg2.grid(row = 2, column = 0, padx = 1/80*self.w_Interface,pady = 1/80*self.h_Interface)
        self.buttonLeg3.grid(row = 3, column = 0, padx = 1/80*self.w_Interface,pady = 1/80*self.h_Interface)
    
        #Action list
        self.listAction = tk.Listbox(self.Onglet2)
        self.listAction.insert(0, "On/Off")
        self.listAction.insert(1, "Manuel increase")
        self.listAction.insert(2, "Manuel decrease")
        self.listAction.insert(3, "Get pos")
        self.listAction.insert(4, "DMG")
        self.listAction.insert(5, "IGM")
        self.listAction.grid(row = 0, column = 2, rowspan = 4, padx = 1/80*self.w_Interface,pady = 1/80*self.h_Interface)
        
        #attach listener
        self.listAction.bind("<<ListboxSelect>>",lambda x : self.majFrameSecondaire())
        
        #Motor choice
        self.listMotor = tk.Listbox(self.Onglet2)
        self.listMotor.insert(1,"Motor A")
        self.listMotor.insert(2,"Motor B")
        self.listMotor.insert(3,"Motor C")
        self.listMotor.grid(row = 0, column = 3, rowspan = 4, padx = 1/80*self.w_Interface,pady = 1/80*self.h_Interface)
        
        #Angle choice
        self.labelAngle = tk.Label(self.Onglet2, text = "Angle en °").grid(row = 0, column =4) 
        self.entryAngle = tk.Entry(self.Onglet2)
        self.entryAngle.grid(row = 1, column = 4)
        self.entryAngle.setvar('0')
        
        #Frame secondaire
        self.frameSecondaire = tk.Frame(self.Onglet2, width = int(self.w_Interface)/3, height=int(self.h_Interface)/3 )
        self.frameSecondaire.grid(sticky = "e")
        
        
        #Hard debug
        self.labelHardCommand = tk.Label(self.Onglet2, text = "Hard com").grid(row = 6, column = 0) 
        self.entryCommand = tk.Entry(self.Onglet2)
        self.entryCommand.grid(row = 7, column = 0)
        
        self.buttonSendCommand = tk.Button(self.Onglet2, text="Send hard com", command=self.sendHardCommand)
        self.buttonSendCommand.grid(row = 7, column = 1)
        
        
    def sendHardCommand(self):
        #send hardCommand
        return 0
        
    
    #--- Activ legs ---#
    def selectLeg0(self):
        if self.legSelect[0]==0:
            self.legSelect[0]=1
            self.buttonLeg0.config(fg = "red")
        else:
            self.legSelect[0]=0
            self.buttonLeg0.config(fg = "black")
        print(self.legSelect)
        
    def selectLeg1(self):
        if self.legSelect[1]==0:
            self.legSelect[1]=1
            self.buttonLeg1.config(fg = "red")
        else:
            self.legSelect[1]=0
            self.buttonLeg1.config(fg = "black")
        print(self.legSelect)
        
    def selectLeg2(self):
        if self.legSelect[2]==0:
            self.legSelect[2]=1
            self.buttonLeg2.config(fg = "red")
        else:
            self.legSelect[2]=0
            self.buttonLeg2.config(fg = "black")
        print(self.legSelect)
        
    def selectLeg3(self):
        if self.legSelect[3]==0:
            self.legSelect[3]=1
            self.buttonLeg3.config(fg = "red")
        else:
            self.legSelect[3]=0
            self.buttonLeg3.config(fg = "black")
        print(self.legSelect)
    #--- Fin select legs ---# 
    
    def majFrameSecondaire(self):
        print("event detect")
        print(self.listAction.curselection()[0])   #get current index of selected item
    
    def onglet3(self):
        return 0
    
    
     
 
        
        
#special class gradient frame
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


#---MAIN---
root = tk.Tk()
app = interface(master=root)
app.mainloop()


