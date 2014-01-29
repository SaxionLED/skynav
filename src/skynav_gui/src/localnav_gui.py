#!/usr/bin/env python

#import roslib
import os
import rospy
import math
import random
import threading
import time

from PyQt4 import QtCore, QtGui

from python_qt_binding.QtGui import QWidget,QVBoxLayout
from python_qt_binding import loadUi

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import PointCloud
from skynav_gui.msg import Object

class Obstacle:
	
	uniqueID = 0
	points = []
	robotStep = 0
	color = None
	

class LocalNavGUI(QWidget):
	
	mShowGrid = True
	mShowAxis = True
	mShowRobot = True
	mShowOrientation = True
	mShowPath = True
	mShowSensorData = False
	mShowObjects = True
	mShowObstacles = False
	
	
	sigRedraw = QtCore.pyqtSignal(bool)
	
	
	pixelsPerMeter = 200
	mapHeight = 0
	mapWidth = 0
	offsetFromOrigin = (25, 25)
	axisLabelLength = 10
	
	robotVectors = []	# for the traversed path
	mSensorData = [] # for the known sensor data
	mObjects = []	# for the seen objects
	mObstacles = []	# for the known obstacles
	
	# add origin vector
	robotVectors.append((0, 0))
	currentOrientaion = 0
	
	# for object opacity
	robotStepsTaken = 0

	def currentPoseCallback(self, data):
		
		self.currentOrientation = data.theta
		self.robotVectors.append((data.x, data.y))
		
		self.robotStepsTaken += 1
		
		
	def sensorDataCallback(self, data):
		
		for p in data.points:
			
			if(self.mSensorData.count((p.x, p.y)) == 0):	# if not already in list
		
				self.mSensorData.append((p.x, p.y))
			
			
	def objectCallback(self, data):
		
		# check if the unique object ID already exists in the list, if so, remove it and add this updated object
		
		oldColor = None
		
		for o in self.mObjects:
			if(o.uniqueID == data.uniqueID):
				oldColor = o.color	# save old color
				self.mObjects.remove(o)
				break # no need to continue
		
		if(len(data.points) > 0):	# dont add objects with 0 points, effectively removing them (see above)
			#self.mObjects.append(data)
			ob = Obstacle()
			ob.points = data.points
			ob.uniqueID = data.uniqueID
			ob.robotStep = self.robotStepsTaken
			ob.color = oldColor
			
			self.mObjects.append(ob)
		
	
	def sigRedrawHandler(self, data):
		
		self.redrawMap()
	
		
	def threadRedraw(self):
		
		while not rospy.is_shutdown():
						
			rospy.sleep(0.1)
			
			self.sigRedraw.emit(True)
		
	
	def __init__(self,):
		super(LocalNavGUI, self).__init__()
		self.setObjectName("LocalNavGUI")
		self._widget = QWidget()

		ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'localnav_gui.ui')
		loadUi(ui_file, self._widget)
		l = QVBoxLayout(self)
		l.addWidget(self._widget)
		
		# set ROS subscriber
		#rospy.Subscriber("/SLAM/current_pose", Pose2D, self.currentPoseCallback)
		rospy.Subscriber("/localnav/sensor_data", PointCloud, self.sensorDataCallback)
		rospy.Subscriber("/localnav/objects", Object, self.objectCallback)
		
		# set UI stuff
		self._widget.map.setScene(QtGui.QGraphicsScene(self))
		
		self.viewRect = self._widget.map.rect()
		self.mapHeight = self.viewRect.height()
		self.mapWidth = self.viewRect.width()
		
		self._widget.map.setSceneRect(QtCore.QRectF(self.viewRect))
		
		self.scene = self._widget.map.scene()
		
		# link buttons to functions
		self._widget.buttonClear.clicked.connect(self.buttonClear)
		
		self._widget.checkGrid.stateChanged.connect(self.filterGrid)
		self._widget.checkAxis.stateChanged.connect(self.filterAxis)
		self._widget.checkRobot.stateChanged.connect(self.filterRobot)
		self._widget.checkOrientation.stateChanged.connect(self.filterOrientation)
		self._widget.checkPath.stateChanged.connect(self.filterPath)
		self._widget.checkSensorData.stateChanged.connect(self.filterSensorData)
		self._widget.checkObjects.stateChanged.connect(self.filterObjects)
		self._widget.checkObstacles.stateChanged.connect(self.filterObstacles)
		
		
		# subscribe to signals
		self.sigRedraw.connect(self.sigRedrawHandler)
		
		
		# launch thread that redraws the map at a set rate
		t_redraw = threading.Thread(target=self.threadRedraw)
		t_redraw.daemon = True
		t_redraw.start()
		
		
	
	def redrawMap(self):
		
		self.scene.clear() # clear map
		
		# draw axis
		pen = QtGui.QPen(QtCore.Qt.black)
		pen.setWidth(1)
		
		if(self.mShowAxis):
			self.scene.addLine(self.offsetFromOrigin[0], self.mapHeight - self.offsetFromOrigin[1], self.mapWidth - self.offsetFromOrigin[0], self.mapHeight - self.offsetFromOrigin[1], pen) # x axis
			self.scene.addLine(self.offsetFromOrigin[0], self.mapHeight - self.offsetFromOrigin[1], self.offsetFromOrigin[0], self.offsetFromOrigin[1], pen) # y axis
		
		gridpen = QtGui.QPen(QtCore.Qt.gray)
		gridpen.setWidth(1)
		gridpen.setStyle(QtCore.Qt.DashLine)

		# draw x axis values and lines
		for i in range(0, (self.mapWidth - self.offsetFromOrigin[0] - self.offsetFromOrigin[0]) / self.pixelsPerMeter):	# width minus both offset sides
			x = self.offsetFromOrigin[0] + self.pixelsPerMeter + self.pixelsPerMeter * i
			y = self.mapHeight - self.offsetFromOrigin[1]
			
			if(self.mShowAxis):
				self.scene.addLine(x, y, x, y + self.axisLabelLength, pen)
			
				# draw labels
				meterLabel = QtGui.QGraphicsSimpleTextItem(repr(i+1) + "m")
				meterLabel.setPos(x - 7, y + 8)
				self.scene.addItem(meterLabel)
			
			# draw gridlines
			if(self.mShowGrid):
				self.scene.addLine(x, y, x, self.offsetFromOrigin[1], gridpen)
			
		
		# draw y axis values and lines
		for i in range(0, (self.mapHeight - self.offsetFromOrigin[1] - self.offsetFromOrigin[1]) / self.pixelsPerMeter):	# height minus both offset sides
			y = self.mapHeight - self.offsetFromOrigin[1] - self.pixelsPerMeter - self.pixelsPerMeter * i
			x = self.offsetFromOrigin[0]
			
			if(self.mShowAxis):
				self.scene.addLine(x, y, x - self.axisLabelLength, y, pen)
				
				meterLabel = QtGui.QGraphicsSimpleTextItem(repr(i+1) + "m")
				meterLabel.setPos(x - 23, y + 4)
				self.scene.addItem(meterLabel)
				
			# draw gridlines
			if(self.mShowGrid):
				self.scene.addLine(x, y, self.mapWidth - self.offsetFromOrigin[0], y, gridpen)
		
		
		
		pen = QtGui.QPen(QtCore.Qt.blue)
		pen.setWidth(3)
		
		for index, point in enumerate(self.robotVectors):
		
			if index > 0: # dont try to connect 0 to -1
		
				pixelX = point[0] * self.pixelsPerMeter + self.offsetFromOrigin[0]
				pixelY = self.mapHeight - (point[1] * self.pixelsPerMeter + self.offsetFromOrigin[1])
				prevPixelX = self.robotVectors[index - 1][0] * self.pixelsPerMeter + self.offsetFromOrigin[0]
				prevPixelY = self.mapHeight - (self.robotVectors[index - 1][1] * self.pixelsPerMeter + self.offsetFromOrigin[1])
				
				# draw line from prev vector to current
				if(self.mShowPath):
					self.scene.addLine(pixelX, pixelY, prevPixelX, prevPixelY, pen)
				
				if index == (len(self.robotVectors) - 1):	# last index aka current location
					
					# draw circle at current location
					if(self.mShowRobot):
						pen = QtGui.QPen(QtCore.Qt.red)
						radius = 10
						self.scene.addEllipse(pixelX - radius / 2, pixelY - radius / 2, radius, radius, pen, QtGui.QBrush(QtGui.QColor("red")))
					
					# draw cone to indicate viewing angle TODO turn line into cone
					if(self.mShowOrientation):
						coneLength = self.pixelsPerMeter / 10
				
						pen = QtGui.QPen(QtCore.Qt.red)
						pen.setWidth(2)
						self.scene.addLine(pixelX, pixelY, pixelX + math.cos(self.currentOrientation) * coneLength, pixelY - math.sin(self.currentOrientation) * coneLength, pen)
			

		pen = QtGui.QPen(QtCore.Qt.black)
		pen.setWidth(3)
		
		# draw sensor data points
		if(self.mShowSensorData):
			
			radius = 1
			
			for point in self.mSensorData:
				
				x = point[0] * self.pixelsPerMeter + self.offsetFromOrigin[0]
				y = self.mapHeight - point[1] * self.pixelsPerMeter - self.offsetFromOrigin[1]
				
				self.scene.addEllipse(x, y, radius, radius, pen)
		
	
		
		# draw objects
		if(self.mShowObjects):
			
			for o in self.mObjects:
				
				if(o.color is None):
					o.color = QtGui.QColor(random.randrange(50, 200), random.randrange(50, 200), random.randrange(50, 200))
				else:
					o.color.setAlpha(max((255 - ((self.robotStepsTaken - o.robotStep) * 10)), 25))
				
				pen = QtGui.QPen(o.color)
				radius = 10
				
				for p in o.points: # draw a big dot on each point in an object, creating a blob like thing
				
					x = p.x * self.pixelsPerMeter - radius / 2 + self.offsetFromOrigin[0]
					y = self.mapHeight - p.y * self.pixelsPerMeter - radius / 2 - self.offsetFromOrigin[1]
					
					self.scene.addEllipse(x,  y, radius, radius, pen, QtGui.QBrush(o.color))

				# or alternatively, draw a filled polygon that connects all points with all points
				# self.scene.addPolygon(o.points, pen, QtGui.QBrush(QtGui.QColor("green"))
					
					
			
			
		
 		self.scene.update()
 		
	
	def buttonClear(self):
		
		del self.mObjects[:]
		del self.mSensorData[:]
		del self.mObstacles[:]


	def filterAxis(self):
		self.mShowAxis = not self.mShowAxis
		
	def filterGrid(self):
		self.mShowGrid = not self.mShowGrid
		
	def filterRobot(self):
		self.mShowRobot = not self.mShowRobot
	
	def filterOrientation(self):
		self.mShowOrientation = not self.mShowOrientation
	
	def filterPath(self):
		self.mShowPath = not self.mShowPath
		
	def filterSensorData(self):
		self.mShowSensorData = not self.mShowSensorData
		
	def filterObjects(self):
		self.mShowObjects = not self.mShowObjects
		
	def filterObstacles(self):
		self.mShowObstacles = not self.mShowObstacles