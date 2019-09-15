import serial, time, math, sys
import numpy as np
import Kinematics
from PyQt5 import QtWidgets, uic
from PyQt5 import QtGui

R_MIN = -360
R_MAX = 360
Z_MIN = -360
Z_MAX = 360
Q0_MIN = -45
Q0_MAX = 45
Q1_MIN = 10
Q1_MAX = 90
Q2_MIN = -135
Q2_MAX = 80
controller = None
SERIAL_PORT = 'COM7'
BAUD_RATE = 9600
TIMEOUT = 1

import matplotlib
matplotlib.use('Qt5Agg')

import sys
from PyQt5 import QtCore, QtGui, uic
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


qtCreatorFile = "main.ui" # my Qt Designer file

Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)

class MyApp(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, controller):
        self.q0 = 0
        self.q1 = 90
        self.q2 = 0
        self.q3 = 70
        self.q4 = 0
        self.speed = 20
        self.controller = controller

        QtWidgets.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.baseAngleSlider.valueChanged[int].connect(self.setQ0)
        self.manipulatorRotationSlider.valueChanged[int].connect(self.setQ3)
        self.manipulatorSlider.valueChanged[int].connect(self.setQ4)
        self.speedSlider.valueChanged[int].connect(self.setSpeed)

        self.createPlot()

    def _J0_Correction(self):
        q0 = self.q0 + 140
        return int(q0)

    def _J1_Correction(self):
        q1 = 180-self.q1
        q1 = q1 - 80
        q1 = q1 * 2
        return int(q1)

    def _J2_Correction(self):
        q2 = self.q2 * 0.888889
        q2 = q2 + 100
        return int(q2)

    def SetJointAngles(self):
        message = str(self._J0_Correction()) + ',' + str(self._J1_Correction()) + ',' + str(self._J2_Correction()) + ',' + str(self.q3) + ',' + str(self.q4) + ',' + str(self.speed) + '\n'
        print(message)
        self.controller.write(message.encode())

    def createPlot(self):
        r_range = [R_MIN, R_MAX]
        z_range = [Z_MIN, Z_MAX]
        ax = self.canvas.figure.add_subplot(111)
        x = []
        y = []
        for i in range(r_range[0], r_range[1]):
            for j in range(z_range[0], z_range[1]):
                [q1, q2] = Kinematics.InverseKinPolar(i, j, 0, R_MIN, R_MAX, Z_MIN, Z_MAX, Q0_MIN, Q0_MAX, Q1_MIN, Q1_MAX, Q2_MIN, Q2_MAX)
                if (q1 is not None and q2 is not None):
                    x.append(i)
                    y.append(j)

        xvalues = np.array(x)
        yvalues = np.array(y)
        ax.plot(xvalues, yvalues, marker='.', color='k', linestyle='none')
        self.canvas.mpl_connect('button_press_event', lambda event: self.positionClick(event))
        self.canvas.draw()

    def setQ0(self, value):
        self.q0 = value

    def setQ3(self, value):
        self.q3 = value

    def setQ4(self, value):
        self.q4 = value

    def setSpeed(self, value):
        deg_ms = (value / 1000) # degrees per ms
        ms_degree = (1 / deg_ms)
        self.speed = int(ms_degree)

    def positionClick(self, event):
        if (self.controller.isOpen()):
            r = int(event.xdata)
            z = int(event.ydata)
            [self.q1, self.q2] = Kinematics.InverseKinPolar(r, z, self.q0, R_MIN, R_MAX, Z_MIN, Z_MAX, Q0_MIN, Q0_MAX, Q1_MIN, Q1_MAX, Q2_MIN, Q2_MAX)
            if (self.q1 is not None and self.q2 is not None):
                self.SetJointAngles()

if __name__ == "__main__":
    try:
        app = QtWidgets.QApplication(sys.argv)
        controller = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)
        window = MyApp(controller)
        window.show()
        sys.exit(app.exec_())
    finally:
        controller.close()
