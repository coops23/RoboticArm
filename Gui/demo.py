import serial, time, math
import matplotlib.pylab as plt
import numpy as np

def SetJointAngles(ser, joint0, joint1, joint2, joint3, speedMs):
    message = str(joint0) + ',' + str(joint1) + ',' + str(joint2) + ',' + str(joint3) + ',' + str(speedMs)
    print(message)
    message += '\n'
    ser.write(message.encode())
    print(ser.readline())
    
def WaitUntilDone(ser):
    status = 0
    while(status == 0):
        msg = 'status' + '\n'
        ser.write(msg.encode())
        status = int(ser.readline())
        
ser = serial.Serial(
    port='COM7',
    baudrate=9600,
    timeout=1
)

def InverseKinematicsGeometric(x, y):
    try:
        Q1_MIN = 0
        Q1_MAX = 135
        Q2_MIN = -135
        Q2_MAX = 135
        l1 = 120
        l2 = 240

        tau = math.atan2(y, x)
        beta = math.acos((math.pow(l1,2) + math.pow(l2,2) - math.pow(x,2) - math.pow(y,2)) / (2 * l1 * l2))
        alpha = math.acos((math.pow(l1,2) - math.pow(l2,2) + math.pow(x,2) + math.pow(y,2)) / (2 * l1 * math.sqrt(math.pow(x,2) + math.pow(y,2))))

        righty_q1 = int(math.degrees(tau - alpha))
        righty_q2 = int(math.degrees(math.pi - beta))
        lefty_q1 = int(math.degrees(tau + alpha))
        lefty_q2 = int(math.degrees(beta - math.pi))

        if (righty_q1 > Q1_MIN and righty_q1 < Q1_MAX and righty_q2 > Q2_MIN and righty_q2 < Q2_MAX):
            q1 = righty_q1
            q2 = righty_q2
        elif (lefty_q1 > Q1_MIN and lefty_q1 < Q1_MAX and lefty_q2 > Q2_MIN and lefty_q2 < Q2_MAX):
            q1 = lefty_q1
            q2 = lefty_q2
        else:
            raise ValueError

    except ZeroDivisionError:
        #print("Invalid position")
        q1 = None
        q2 = None

    except ValueError:
        #print("Invalid position")
        q1 = None
        q2 = None

    finally:
        return [q1, q2]

def onclick(event):
    q3 = 70
    q4 = 0
    speed = 20

    if (ser.isOpen()):
        x = int(event.xdata)
        y = int(event.ydata)
        debugMsg = str(x) + "," + str(y)
        print(debugMsg)
        [q1, q2] = InverseKinematicsGeometric(x, y)
        if (q1 is not None and q2 is not None):
            SetJointAngles(ser, q1, q2, q3, q4, speed)
            WaitUntilDone(ser)

x_range = [0, 360]
y_range = [0, 360]
fig,ax = plt.subplots()
x = []
y = []
for i in range(x_range[0], x_range[1]):
    for j in range(y_range[0], y_range[1]):
        [q1, q2] = InverseKinematicsGeometric(i, j)
        if (q1 is not None and q2 is not None):
            x.append(i)
            y.append(j)

xvalues = np.array(x)
yvalues = np.array(y)
#xx,yy = np.meshgrid(xvalues,yvalues)
ax.plot(xvalues,yvalues, marker='.', color='k', linestyle='none')
fig.canvas.mpl_connect('button_press_event', onclick)
x1,x2,y1,y2 = plt.axis()
plt.axis((x_range[0], x_range[1], y_range[0], y_range[1]))
plt.show()

ser.close()

'''
if(ser.isOpen()):
    while True:
        user = input(">> ")
        if user == 'exit':
            ser.close()
            exit()
        elif user == 'open':
            q4 = 150
            SetJointAngles(ser, q1, q2, q3, q4, speed)
            WaitUntilDone(ser)
        elif user == 'close':
            q4 = 0
            SetJointAngles(ser, q1, q2, q3, q4, speed)
            WaitUntilDone(ser)
        elif user == 'reset':
            q1 = 90
            q2 = 0
            q3 = 70
            q4 = 0
            speed = 20
            SetJointAngles(ser, q1, q2, q3, q4, speed)
            WaitUntilDone(ser)
        else:
            userInput = user.split(",")
            x = float(userInput[0])
            y = float(userInput[1])
            [q1, q2] = InverseKinematicsGeometric(q1, q2, x, y)
            SetJointAngles(ser, q1, q2, q3, q4, speed)
            WaitUntilDone(ser)
'''