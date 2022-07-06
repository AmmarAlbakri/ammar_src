#!/usr/bin/env python3
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
import sys
import rospy
#from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class Joystick(QWidget):
    joystic_x = 0.0
    joystic_y = 0.0

    def __init__(self, parent=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(300, 300)
        self.movingOffset = QPointF(0, 0)
        self.grabCenter = False
        self.__maxDistance = 100
        self.UiComponents()
        #self.setWindowIcon(QIcon('/home/mete/catkin_ws/src/prototype_vehicle_mini/joystick.png'))

    def UiComponents(self):
        button = QPushButton("Durdur", self)
        button.setGeometry(200, 10, 100, 30)
        button.clicked.connect(self.clickme)
  
    def clickme(self):
        velocity_publisher_sol = rospy.Publisher('/sol_teker_joint_velocity_controller/command', Float64, queue_size=10)
        velocity_publisher_sag = rospy.Publisher('/sag_teker_joint_velocity_controller/command', Float64, queue_size=10)
        self.vel_msg_sol = Float64()
        self.vel_msg_sag = Float64()
        self.vel_msg_sol.data = 0
        self.vel_msg_sag.data = 0
        velocity_publisher_sol.publish(self.vel_msg_sol)
        velocity_publisher_sag.publish(self.vel_msg_sag)

    def paintEvent(self, event):
        painter = QPainter(self)
        bounds = QRectF(-self.__maxDistance, -self.__maxDistance, self.__maxDistance * 2, self.__maxDistance * 2).translated(self._center())
        painter.drawEllipse(bounds)
        painter.setBrush(Qt.black)
        painter.drawEllipse(self._centerEllipse())

    def _centerEllipse(self):
        if self.grabCenter:
            return QRectF(-20, -20, 40, 40).translated(self.movingOffset)
        return QRectF(-20, -20, 40, 40).translated(self._center())

    def _center(self):
        return QPointF(self.width()/2, self.height()/2)

    def _boundJoystick(self, point):
        limitLine = QLineF(self._center(), point)
        if (limitLine.length() > self.__maxDistance):
            limitLine.setLength(self.__maxDistance)
        return limitLine.p2()

    def mousePressEvent(self, ev):
        self.grabCenter = self._centerEllipse().contains(ev.pos())
        return super().mousePressEvent(ev)

    def mouseReleaseEvent(self, event):
        self.grabCenter = False
        self.movingOffset = QPointF(0, 0)
        self.update()

    def mouseMoveEvent(self, event):
        if self.grabCenter:
            self.movingOffset = self._boundJoystick(event.pos())
            self.update()
        self.joystickDirection()

    def joystickDirection(self):
        if not self.grabCenter: return 0
        normVector = QLineF(self._center(), self.movingOffset)
        currentDistance = normVector.length()
        self.joystic_x = normVector.dx()
        self.joystic_y = normVector.dy()
        angle = normVector.angle()

        distance = min(currentDistance / self.__maxDistance, 1.0)

        

        velocity_publisher_sol = rospy.Publisher('/sol_teker_joint_velocity_controller/command', Float64, queue_size=10)
        velocity_publisher_sag = rospy.Publisher('/sag_teker_joint_velocity_controller/command', Float64, queue_size=10)
        self.vel_msg_sol = Float64()
        self.vel_msg_sag = Float64()
        self.vel_msg_sol.data = - self.joystic_y/10 - self.joystic_x/20
        self.vel_msg_sag.data = self.joystic_y/10 - self.joystic_x/20
        velocity_publisher_sol.publish(self.vel_msg_sol)
        velocity_publisher_sag.publish(self.vel_msg_sag)
        #print ("self.vel_msg_sol.data: " + str(self.vel_msg_sol.data))
        #print ("self.vel_msg_sag.data: " + str(self.vel_msg_sag.data))
        
if __name__ == '__main__':
    app = QApplication([])
    app.setStyle(QStyleFactory.create("Cleanlooks"))
    mw = QMainWindow()
    mw.setWindowTitle('Joystick')
    cw = QWidget()
    ml = QGridLayout()
    cw.setLayout(ml)
    mw.setCentralWidget(cw)
    joystick = Joystick()
    ml.addWidget(joystick,0,0)
    mw.show()

    try: 
        rospy.init_node('joystick_controller', anonymous=True)
    except rospy.ROSInterruptException: pass

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QApplication.instance().exec_()
