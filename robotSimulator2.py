from math import *
import time

from tkinter import *
from eventBasedAnimationClass import EventBasedAnimationClass

class EventBasedAnimationDemo(EventBasedAnimationClass):

    def __init__(self):

        self.width = 800
        self.height = 650

        super(EventBasedAnimationDemo, self).__init__(self.width, self.height)

        self.timerDelay = 1 # Tweaking this value creates inaccuracies
        self.time = 0


        self.cx, self.cy = self.width/2, self.height/2
        theta = 0
        L = 266.5 # mm
        r = 15.112 # mm
        markerDistance = L/3

        self.graphOption = 9
        x,y = self.getIdealPosition(0)
        self.graphPoints = [(x,y)]

        startX = x-markerDistance
        startY = y
        self.robot = Robot(startX, startY, theta, L, r, 0,0, markerDistance)
        self.markerDistance = markerDistance


    
    def onMousePressed(self, event):
        pass
    
    def onKeyPressed(self, event):
        pass
    
    def onTimerFired(self):
        currTime = time.time() * 1000
        self.time = currTime - self.startTime

        '''
        if self.time >= 9000:
            self.robot.powerLeft = 0
            self.robot.powerRight = 0
        elif self.time >= 6000:
            self.robot.powerLeft = -200
            self.robot.powerRight = -175
        elif self.time >= 3000:
            self.robot.powerLeft = 100
            self.robot.powerRight = 100
        else:
            self.robot.powerLeft = 200
            self.robot.powerRight = -200
            '''

        self.setRobotPower()

        self.robot.move((currTime - self.prevTime))

        self.graphPoints.append(self.getIdealPosition(self.time))

        self.prevTime = currTime

    def setRobotPower(self):
        # Calculate where we need to be
        # Calculate power needed for powerLeft and powerRight of robot
        robot = self.robot
        L,r,theta = robot.L, robot.r, robot.theta
        xGoal, yGoal = self.getIdealPosition(self.time+17) # Look ahead in trajectory by 17ms
        xCurr, yCurr = robot.getMarkerPosition()

        xDiff = (xGoal - xCurr)
        yDiff = (- yGoal + yCurr) # Because of negative y in graphics

        kp = 20
        kt = 30
        vFinal = kp*(cos(theta) * xDiff + sin(theta) * yDiff)
        omega  = kt*((cos(theta) * yDiff - sin(theta) * xDiff)/L)

        robot.powerLeft = 9/pi * (2*vFinal - L*omega) / r
        robot.powerRight = 9/pi * (2*vFinal + L*omega) / r




    def getIdealPosition(self, t):
        t = t/1000.0
        if self.graphOption == 1:
            x = 500 * cos(t/10) * sin(t/10)
            y = 200 * sin(t/10) * sin(t/5)
        elif self.graphOption == 2:
        	x = 200 * sin(3*t/5)
        	y = 200 * cos(2*(t/5 + pi/4))
        elif self.graphOption == 3:
        	x = 200 * cos(t/10) * cos(t/5)
        	y = 200 * cos(3*t/10) * sin(t/10)
        elif self.graphOption == 4:
        	x = 200 * (1.0/2*cos(3*t/10)-3.0/4*cos(t/5))
        	y = 200 * (-3.0/4*sin(t/5)-1.0/2*sin(3*t/10))
        elif self.graphOption == 5:
            x = 100 * (-2 * cos(t/5)**2 - sin(t/10) + 1) * sin(t/5)
            y = 100 * cos(t/5) * (-2 * cos(t/5)**3 - sin(t/10) + 1)
        elif self.graphOption == 6:
            x = 100 * (2 * cos(t/12)**3 + 1)*sin(t/4)
            y = 100 * cos(t/4) * (1-2*sin(t/4)**4)
        elif self.graphOption == 7:
            x = 40*(5*cos(9*t/20)-4*cos(t/4))
            y = 40*(-4*sin(t/4)-5*sin(9*t/20))
        elif self.graphOption == 8: # Heart
        	x = 160 * sin(t/5)**3
        	y = 10*(13*cos(t/5) - 5 * cos(2*t/5) - 2*cos(3*t/5) - cos(4*t/5))
        elif self.graphOption == 9: # Batman curve
        	if t < 2:
        		# At t = 0, x = -1, y = 1
        		x = t/8-1
        		y = t + 1
        	elif 2 <= t < 3.5:
        		# At t = 2, x = -0.75, y = 3
        		x = t/6 - 13/12
        		y = -t/2 + 4
        	elif 3.5 <= t < 4.5:
        		# At t = 3.5, x = -1/2, y = 2.25
        		x = t - 4
        		y = 2.25
        	elif 4.5 <= t < 6:
        		# At t = 4.5, x = 0.5, y = 2.25
        		x = t/6 - 0.25
        		y = t/2
        	elif 6 <= t < 8:
        		# At t = 6, 0.75, y = 3
        		x = t/8
        		y = -t + 9
        	elif 8 <= t < 16:
        		# At t = 8, x = 1, y = 1
        		x = t/4 - 1
        		y = (6*sqrt(10)/7 + 1.5 - 0.5*x) - (6*sqrt(10)/14)*sqrt(4-(x-1)**2)
        	elif 16 <= t < 2.0/7*(56 + 6*sqrt(10) + 3*sqrt(33)):
        		# At t=16, x=3, y=6*sqrt(10)/7
        		y = -t/2 + 8 + 6*sqrt(10)/7
        		x = 7 * sqrt(1-(y/3)**2)
        	elif 2.0/7*(56 + 6*sqrt(10) + 3*sqrt(33)) <= t < 38.345:
        		# At t = 2.0/7*(56 + 6*sqrt(10) + 3*sqrt(33)), x = 4, y = -3*sqrt(33)/7
        		x = -2.0/3*t + 4.0/21*(77 + 6*sqrt(10) + 3*sqrt(33))
        		#y = abs(x/2)-(3*sqrt(33)-7)/112*x**2 - 3 + sqrt((1-abs(abs(x)-2)-1)**2)
        		y = abs(x/2) - (3*sqrt(33)-7)/112*x**2 - 3 + sqrt(1-(abs(abs(x)-2)-1)**2)
        	elif 38.345 <= t < 48.69:
        		# At t = 38.345, x = -4, y = -3*sqrt(33)/7
        		y = t/2 - 38.345/2 - 3*sqrt(33)/7
        		x = -7 * sqrt(1-(y/3)**2)
        	elif 48.69 <= t < 56.69:
        		# At t = 48.69, x = -3, y = 6*sqrt(10)/7
        		x = t/4 - 48.69/4 - 3
        		y = (6*sqrt(10)/7 + 1.5 - 0.5*abs(x) - (6*sqrt(10)/14)*sqrt(4-(abs(x)-1)**2))
        	elif 56.69 <= t < 58.69:
        		# x = -1, y = 1
        		x = -1
        		y = -t + 57.69
        	elif 58.69 <= t < 60.19:
        		# x = -1, y = -1
        		x = t/2 -29.345 - 1
        		y = t/2 -29.345 - 1
        	elif 60.19 <= t < 61.69:
        		# x = -0.75, y = -0.75
        		x = -t/2 + 30.095 - 0.25
        		y = t/2 -29.345 - 1
        	else:
        		x = 0
        		y = 0

        	x *= 30
        	y *= 30
        	'''
        	x = 25 * sin(t/2) + 4
        	y = 7 * t + 2
        	'''


        return (self.cx + x, self.cy - y)



    def drawRobot(self):
        robot = self.robot
        robotX, robotY = robot.x, robot.y
        theta = robot.theta
        L = robot.L
        r = robot.r

        # Draw robot body
        x0 = robotX + L/2 * cos(theta + pi/2)
        y0 = robotY - L/2 * sin(theta + pi/2)
        x1 = robotX + L/2 * cos(theta - pi/2)
        y1 = robotY - L/2 * sin(theta - pi/2)
        self.canvas.create_line(x0,y0,x1,y1,tags="redrawables")

        # Draw robot arrow
        x2 = robotX + self.markerDistance * cos(theta)
        y2 = robotY - self.markerDistance * sin(theta)
        self.canvas.create_line(robotX,robotY,x2,y2,tags="redrawables")

        # Draw robot wheels

    def drawRobotTrajectory(self):
        if len(self.robot.trajectoryPoints) > 1:
            self.canvas.create_line(self.robot.trajectoryPoints[-2],self.robot.trajectoryPoints[-1],fill="red",width=1)

    def drawGraph(self):
        if len(self.graphPoints) > 1:
            self.canvas.create_line(self.graphPoints[-2],self.graphPoints[-1],fill="blue",width=2)
    
    def drawTimer(self):
        self.canvas.create_text(10,10,text="%d" % self.time, anchor=NW,tags="redrawables")

    def drawActualTimer(self):
        self.canvas.create_text(self.width-10,10,text="%d" % (time.time()*1000-self.startTime), anchor=NE,tags="redrawables")

    def drawRobotPower(self):
        self.canvas.create_text(self.width-10,self.height-10,text="%d, %d" % (self.robot.powerLeft, self.robot.powerRight), anchor=SE,tags="redrawables")

    def drawRobotStats(self):
        self.canvas.create_text(10,self.height-10,text=self.robot.getStats(), anchor=SW, tags="redrawables")

    def redrawAll(self):
        self.canvas.delete("redrawables")
        
        self.drawRobot()

        self.drawGraph()
        self.drawRobotTrajectory()

        self.drawRobotPower()
        self.drawRobotStats()

        #self.drawTimer()
        self.drawActualTimer()



    def initAnimation(self):
        self.startTime = self.prevTime = time.time() * 1000
        self.canvas.create_rectangle(0,0,self.width+20,self.height+20,fill="grey")
    




class Robot(object):
    def __init__(self, x, y, theta, L, r, powerLeft, powerRight, markerDistance):
        self.x = x
        self.y = y
        self.theta = theta

        self.L = L
        self.r = r

        self.powerLeft = powerLeft
        self.powerRight = powerRight

        markerX = self.x + self.L/3 * cos(self.theta)
        markerY = self.y - self.L/3 * sin(self.theta)
        self.trajectoryPoints = [(markerX, markerY)]

        self.markerDistance = markerDistance

    def move(self, timerDelay):
        vLeft = self.powerLeft/180.0 * pi * self.r * 10
        vRight = self.powerRight/180.0 * pi * self.r * 10
        self.vFinal = (vLeft + vRight)/2
        self.omega = (vRight - vLeft)/self.L

        timerDelayInSeconds = timerDelay/1000.0;
        self.x += (self.vFinal * cos(self.theta)) * timerDelayInSeconds
        self.y -= (self.vFinal * sin(self.theta)) * timerDelayInSeconds
        self.theta += self.omega * timerDelayInSeconds

        self.trajectoryPoints.append(self.getMarkerPosition())

    def getMarkerPosition(self):
        x = self.x + self.markerDistance * cos(self.theta)
        y = self.y - self.markerDistance * sin(self.theta)
        return (x,y)

    def getStats(self):
        return "%.1f, %.1f, %.1f" % (self.x, self.y, self.theta/pi*180)

EventBasedAnimationDemo().run()

