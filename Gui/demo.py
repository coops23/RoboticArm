import serial, time, math
import matplotlib.pylab as plt
import mpl_toolkits.mplot3d as Axes3D
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

def InverseKinematicsGeometric(x, y, z):
    try:
        q0 = None
        q1 = None
        q2 = None
        Q0_MIN = -180
        Q0_MAX = 180
        Q1_MIN = 0
        Q1_MAX = 90
        Q2_MIN = -135
        Q2_MAX = 135
        l1 = 120
        l2 = 240

        q0 = int(math.degrees(math.atan2(y, x)))
        r = math.sqrt(math.pow(x,2) + math.pow(y,2))
        z = z

        tau = math.atan2(z, r)
        beta = math.acos((math.pow(l1,2) + math.pow(l2,2) - math.pow(r,2) - math.pow(z,2)) / (2 * l1 * l2))
        alpha = math.acos((math.pow(l1,2) - math.pow(l2,2) + math.pow(r,2) + math.pow(z,2)) / (2 * l1 * math.sqrt(math.pow(r,2) + math.pow(z,2))))

        righty_q1 = int(math.degrees(tau - alpha))
        righty_q2 = int(math.degrees(math.pi - beta))
        lefty_q1 = int(math.degrees(tau + alpha))
        lefty_q2 = int(math.degrees(beta - math.pi))

        if (righty_q1 > Q1_MIN and righty_q1 < Q1_MAX and righty_q2 > Q2_MIN and righty_q2 < Q2_MAX):
            q1 = righty_q1
            q2 = -1*righty_q2
        elif (lefty_q1 > Q1_MIN and lefty_q1 < Q1_MAX and lefty_q2 > Q2_MIN and lefty_q2 < Q2_MAX):
            q1 = lefty_q1
            q2 = -1*lefty_q2
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
        return [q0, q1, q2]

def createWorkspaceGraph2D():
    x_range = [0, 360]
    y_range = [0, 360]
    fig, ax = plt.subplots()
    x = []
    y = []
    for i in range(x_range[0], x_range[1]):
        for j in range(y_range[0], y_range[1]):
            [q0, q1, q2] = InverseKinematicsGeometric(i, 0, j)
            if (q1 is not None and q2 is not None):
                x.append(i)
                y.append(j)

    xvalues = np.array(x)
    yvalues = np.array(y)
    # xx,yy = np.meshgrid(xvalues,yvalues)
    ax.plot(xvalues, yvalues, marker='.', color='k', linestyle='none')
    fig.canvas.mpl_connect('button_press_event', onclick)
    x1, x2, y1, y2 = plt.axis()
    plt.axis((x_range[0], x_range[1], y_range[0], y_range[1]))
    plt.show()

def createWorkspaceGraph3D():
    x_range = [150, 200]
    y_range = [150, 200]
    z_range = [150, 200]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    x = []
    y = []
    z = []
    for i in range(x_range[0], x_range[1]):
        for j in range(y_range[0], y_range[1]):
            for k in range(z_range[0], z_range[1]):
                [q0, q1, q2] = InverseKinematicsGeometric(i, j, k)
                if (q0 is not None and q1 is not None and q2 is not None):
                    x.append(i)
                    y.append(j)
                    z.append(k)

    xvalues = np.array(x)
    yvalues = np.array(y)
    zvalues = np.array(y)
    ax.scatter(xvalues,yvalues,zvalues, c=None)
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')

    plt.show()

def onclick(event):
    q3 = 70
    q4 = 0
    speed = 20

    if (ser.isOpen()):
        x = int(event.xdata)
        z = int(event.ydata)
        debugMsg = str(x) + "," + str(z)
        print(debugMsg)
        [q0, q1, q2] = InverseKinematicsGeometric(x, 0, z)
        if (q1 is not None and q2 is not None):
            SetJointAngles(ser, q1, q2, q3, q4, speed)
            WaitUntilDone(ser)

ser = serial.Serial(port='COM7', baudrate=9600, timeout=1)
createWorkspaceGraph2D()
ser.close()