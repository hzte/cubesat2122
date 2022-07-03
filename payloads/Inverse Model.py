from math import *
from numpy import *

# Length of links in cm
A1=5.3
A2=6.5
A3=4.4
#Desired position of end effector 
Px = 10
Py = 10
phi = 0
phi = deg2rad(phi)
# Equations for Inverse kinematics 
Wx = Px - A3*cos(phi)
Wy = Py - A3*sin(phi)

Delta = Wx**2 + Wy**2
C2 = (Delta -A1**2-A2**2)/(2*A1*A2)
S2 = sqrt(1-C2**2)
Theta_2 = arctan2(S2,C2)

S1 = ((A1+A2+C2)*Wy-A2*S2*Wx)/Delta
C1 = ((A1+A2+C2)*Wx+S2*A2*Wx)/Delta
Theta_1 = arctan2(S1,C1)
Theta_3 = phi-Theta_1-Theta_2

Theta_1=rad2deg(Theta_1)
Theta_2=rad2deg(Theta_2)
Theta_3=rad2deg(Theta_3)

print(Theta_1)
print(Theta_2)
print(Theta_3)
#Boundary Conditions:
    ##Certain condtions may only be truly known during testing
    
#Segment 1
Theta_1 = 0 if Theta_1 <0 else Theta_1
Theta_1 =  if Theta_1 > else Theta_1

Delta = Wx**2 + Wy**2
C2 = (Delta -A1**2-A2**2)/(2*A1*A2)
S2 = sqrt(1-C2**2)
Theta_2 = arctan2(S2,C2)

Theta_1 = Theta_1

Theta_3 = phi-Theta_1-Theta_2

Theta_1=rad2deg(Theta_1)
Theta_2=rad2deg(Theta_2)
Theta_3=rad2deg(Theta_3)
#Segment 2
Theta_2 =  if Theta_2 < else Theta_2
Theta_2 =  if Theta_2 > else Theta_2

Delta = Wx**2 + Wy**2
C2 = (Delta -A1**2-A2**2)/(2*A1*A2)
S2 = sqrt(1-C2**2)
Theta_2 = Theta_2

S1 = ((A1+A2+C2)*Wy-A2*S2*Wx)/Delta
C1 = ((A1+A2+C2)*Wx+S2*A2*Wx)/Delta
Theta_1 = arctan2(S1,C1)
Theta_3 = phi-Theta_1-Theta_2

Theta_1=rad2deg(Theta_1)
Theta_2=rad2deg(Theta_2)
Theta_3=rad2deg(Theta_3)
#Segment 3
Theta_3 =  if Theta_3 <  else Theta_3
Theta_3 =  if Theta_3 <  else Theta_3



Theta_1=rad2deg(Theta_1)
Theta_2=rad2deg(Theta_2)
Theta_3=rad2deg(Theta_3)

print(Theta_1)
print(Theta_2)
print(Theta_3)