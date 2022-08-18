from controller import Robot

robot = Robot()
timestep = 2

lm=robot.getDevice("wheel2")
rm=robot.getDevice("wheel1")
lm2=robot.getDevice("wheel4")
rm2=robot.getDevice("wheel3")

lm.setPosition(float('inf'))
lm.setVelocity(0.0)

rm.setPosition(float('inf'))
rm.setVelocity(0.0)

lm2.setPosition(float('inf'))
lm2.setVelocity(0.0)

rm2.setPosition(float('inf'))
rm2.setVelocity(0.0)

sensors=[]
names=["ir1","ir2","ir3","ir4","ir5","ir6"]
reading=[0,0,0,0,0,0]

previous_error=0.0
kp=2.5
kd=0.5
ki=0.0
Integral=0.0


for i in range (0,6):
    sensors.append(robot.getDistanceSensor(names[i]))
    sensors[i].enable(timestep)

def getReading():
    for i in range (0,6):
        if int(sensors[i].getValue())>900:
            reading[i]=1
        else:
            reading[i]=0
    
def PID():
    error=0
    coefficient=[-3000,-2000,-1000,1000,2000,3000]
    
    for i in range(0,6):
        error+=coefficient[i]*reading[i]
        
    P=kp*error
    I=Integral+(ki*error)
    D=kd*(error-previous_error)
    
    
    correction=(P+I+D)/1000
    
    l_speed=8.5+correction
    r_speed=8.5-correction
    
    
    if l_speed<0.0  : l_speed=0
    if l_speed>8.5 : l_speed=8.5
    if r_speed<0.0  : r_speed=0
    if r_speed>8.5 : r_speed=8.5
    
    
    lm.setVelocity(-l_speed)
    rm.setVelocity(-r_speed)
    lm2.setVelocity(-l_speed)
    rm2.setVelocity(-r_speed)
    
    
    print("Left-wheel-speed:",l_speed)
    print("Right-wheel-speed:",r_speed)
    print("IR-Reading:",reading)
    
    return I,error

while (robot.step(timestep) != -1):
    getReading()
    print(kp, kd, ki)
    Integral,previous_error=PID()
    pass