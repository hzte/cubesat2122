# -*- coding: utf-8 -*-
"""
Created on Fri Jul  1 15:50:48 2022

@author: Ali23
"""
from numpy import *
from math import *

Theta = 90

#Mass
Servo_1G=
Servo_2G=
Servo_3G=
Servo_4G=
Short_BracketG=
Custom_BracketG=
Long_BracketG=
Mechanical_GripperG=

Servo_SG90_G=
Lead_Weights_G=
Servo_mount_G=
CG_Pole_G=
Lead_Weights2_G=

#Distance From Datum
Servo_1_Datum1=
Servo_2_Datum1=
Servo_3_Datum1=
Servo_4_Datum1=
Short_Bracket_Datum1=
Custom_Bracket_Datum1=
Long_Bracket_Datum1=
Mechanical_Gripper_Datum1=

Servo_SG90_Datum2=
Lead_Weights_Datum2=
Servo_mount_Datum2=
CG_Pole_Datum2=
Lead_Weights2_Datum2=

#Weights 
W1 = Servo_1G*9.81
W2 = Servo_2G*9.81
W3 = Servo_3G*9.81
W4 = Servo_4G*9.81
W5 = Short_BracketG*9.81
W6 = Custom_BracketG*9.81
W7 = Long_BracketG*9.81
W8 = Mechanical_GripperG*9.81
WT_1 = W1+W2+W3+W4+W5+W6+W7+W8

W_1=Servo_SG90_G*9.81
W_2=Lead_Weights_G*9.81
W_3=Servo_mount_G*9.81
W_4=CG_Pole_G*9.81
W_5=Lead_Weights2_G
WT_2 = W_1+W_2+W_3+W_4+W_5

#Moment calculations
M1=W1*Servo_1_Datum1
M2=W2*Servo_2_Datum1
M3=W3*Servo_3_Datum1
M4=W4*Servo_4_Datum1
M5=W5*Short_Bracket_Datum1
M6=W6*Custom_Bracket_Datum1
M7=W7*Long_Bracket_Datum1
M8=W8*Mechanical_Gripper_Datum1
MT_1=M1+M2+M3+M4+M5+M6+M7+

M_1=W_1*Servo_SG90_Datum2
M_2=W_2*Lead_Weights_Datum2
M_3=W_3*Servo_mount_Datum2
M_4=W_4*CG_Pole_Datum2
M_5=W_5*Lead_Weights2_Datum2
MT_2 = M_1+M_2+M_3+M_4+M_5

#CG values
CG_arm=
CG_Mechanism=

#If CG values do not equal
if CG_arm!=CG_Mechanism :
    Theta = Theta-1
    
Height_of_CM_arm = 
Height_of_CM_Weights = 

Lead_Weights_Datum2 = Height_of_CM_Weights*cos(Theta)
CG_Pole_Datum2 = Height_of_CM_arm*cos(Theta)

Theta = 0 if Theta < 0 else Theta