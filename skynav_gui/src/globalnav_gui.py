## when using rosbuild these lines are required to make sure that all dependent Python packages are on the PYTHONPATH:
import os
import rospy
import roslib
import sys
import threading
roslib.load_manifest('skynav_gui')

import Image
import ImageQt
import QtCore
from python_qt_binding.QtGui import QWidget,QVBoxLayout,QFileDialog, \
QMessageBox,QGraphicsScene, QPen, QBrush,QColor,QInputDialog, QDialog,\
QPixmap, QTableWidget, QTableWidgetItem
from python_qt_binding import loadUi

import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Log
from nav_msgs.msg import Path
from skynav_msgs.msg import *
from skynav_msgs.srv import *


class EnvironmentMapScene(QGraphicsScene):
	def __init__(self, x, y):	
		super(EnvironmentMapScene,self).__init__()
		self.x = x;
		self.y = y
		self.resetMap()
		
	def resetMap(self):
		self.clear()
		self.borderItem = self.addRect(-1,-1,self.x+1,self.y+1)

class MapData():
	def __init__(self, x, y, res, data):
		self.xdim = x;
		self.ydim = y;
		self.res = res
		self.data = data	
		
	def getOccupancyAt(self,x,y):
		return self.data[y*self.xdim+x]
		
class Waypoint():
	def __init__(self,x,y,theta,name,type):
		self.x = x
		self.y = y
		self.theta = theta
		self.name = name
		self.type = type
		
	def information(self):
		print self.name
		print self.x
		print self.y
		print self.theta
		print self.type
		print '--'


class GlobalNavGUI(QWidget):
	#variables
	scale = 100	# temporary fixed scaling variable, 
	wpointsize = 2	# pointsize on drawn map
	
	waypoints = []	#the list with user added waypoints
	
	#signals
	sigPathUpdate = QtCore.pyqtSignal(object)
	sigDebugOut = QtCore.pyqtSignal(object)
	
	def __init__(self):
		super(GlobalNavGUI, self).__init__()
		
		self.setObjectName("globalNavGUI")
		self._widget = QWidget()
		ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'globalnav_gui.ui')
		loadUi(ui_file, self._widget)
		l = QVBoxLayout(self)
		l.addWidget(self._widget)
		
		self.gScene = EnvironmentMapScene(1,1)
		self._widget.graphicsView.setScene(self.gScene)
		self._widget.graphicsView.fitInView(self.gScene.borderItem)
		
		self._widget.tableWidget.setRowCount(0)
		self._widget.tableWidget.setColumnCount(6)
		self._widget.tableWidget.setHorizontalHeaderLabels(["nodename", "x", "y", "theta", "node type", "comment"]) 
		self._widget.tableWidget.showGrid()
		
		#signal subscribers
		self.sigPathUpdate.connect(self.updatePath)
		self.sigDebugOut.connect(self.debugOutput)		
		self._widget.start_searcher.currentIndexChanged.connect(self.setCoordsStart)
		self._widget.target_searcher.currentIndexChanged.connect(self.setCoordsTarget)

		#ROS subscribers
		rospy.Subscriber("/globalnav/waypoints", Path, self.updatePathCallback)
		rospy.Subscriber("rosout_agg",Log, self.debugOutCallback)
		
		#ROS publishers		
		self.query_pub = rospy.Publisher('/globalnav/user_input_query',user_input_query)	
		self.re_init_pub = rospy.Publisher('/globalnav/user_init',user_init)	
		self.target_pub = rospy.Publisher('/globalnav/waypoints',Path)
		self.embreak_pub = rospy.Publisher('/control/emergencystop_interrupt',UInt8)
		
		#ROS services
		self.loadNewMap_srv = rospy.ServiceProxy('/globalnav/update_map_req',mapreader_srv)
		self.updateFixedWaypoints_srv = rospy.ServiceProxy('/globalnav/edit_fixedWPs',edit_fixedWPs_srv)
		self.getPath_query_srv = rospy.ServiceProxy('/globalnav/path_query',path_query_srv)

		#button handlers
		self._widget.GO_button.clicked.connect(self.sendQuery)
		self._widget.loadmap_button.clicked.connect(self.loadMap)
		self._widget.reinitialize_button.clicked.connect(self.reInit)		
		self._widget.fixedwps_update.clicked.connect(self.updateFixedWaypoints)
		self._widget.fixedwps_add.clicked.connect(self.addRow)
		self._widget.fixedwps_delete.clicked.connect(self.delRow)
		self._widget.manualTargetButton.clicked.connect(self.sendManualTarget)
		self._widget.meters_toggle.clicked.connect(self.toggleMeters)
		self._widget.emergencyStopButton.clicked.connect(self.emergencyBreakSend)

		self._widget.save_button.clicked.connect(self.demoFunction)
		

	def resizeEvent(self,ev):
		self._widget.graphicsView.fitInView(self.gScene.borderItem)
	
	
	#output a manual target pose
	def sendManualTarget(self):
		xM = self._widget.manualTarget_x_input.text()
		yM = self._widget.manualTarget_y_input.text()
		thM = self._widget.manualTarget_th_input.text()
		pos = Pose()
		if xM and yM:
			pos.position.x = float(xM)
			pos.position.y = float(yM)
		else:
			pos.position.x = float(0)
			pos.position.y = float(0)
		if thM:
			pos.orientation.z = float(thM)
		else:
			pos.orientation.z = float(0)
		
		msg = Path()
		msg.header.frame_id = "/map"
		msg.header.stamp = rospy.Time.now()
		target = PoseStamped()
		target.header.frame_id = "/map"
		target.header.stamp = rospy.Time.now()
		target.pose = pos
	
		msg.poses.append(target)
		
		self.target_pub.publish(msg)
		
	#output a query request	
	def sendQuery(self):
		xS = self._widget.start_x_input.text()
		yS = self._widget.start_y_input.text()
		thS = self._widget.start_th_input.text()
		xT = self._widget.target_x_input.text()
		yT = self._widget.target_y_input.text()
		thT = self._widget.target_th_input.text()
		
		if self._widget.meters_toggle.isChecked():
			xS = float(xS)*100
			yS = float(yS)*100
			xT = float(xT)*100
			yT = float(yT)*100
			
		if xS and yS and xT and yT:
			start= Pose2D()
			target=Pose2D()
			
			start.x = float(xS)
			start.y = float(yS)
			
			target.x = float(xT)
			target.y = float(yT)
			
			if thS:
				start.theta = float(thS)
			else:	 
				#print "no viable Start theta set, default"
				self._widget.start_th_input.setText("0")
				target.theta = float(0)
				
			if thT:
				target.theta = float(thT)
			else:
				#print "no viable Target theta set, default"
				self._widget.target_th_input.setText("0")
				target.theta = float(0)
					
			try:
				ret = self.getPath_query_srv(1,start,target)
				self._widget.info_textbox.append("Path from ("+ str(xS)+","+str(yS)+") to ("+str(xT)+","+str(yT)+")")			
			except rospy.ServiceException, e:
				self._widget.info_textbox.append("Error with query")
				self._widget.debug_textbrowser.append("Service call failed: %s"%e)
		else:
			self._widget.info_textbox.append( "ERROR, specify start or target")	
		
		
	#output a new map msg and get data[] and dimensions back
	def loadMap(self):
		fileName, selectedfilter = QFileDialog.getOpenFileName(self, "Open map","/home", "Text files (*.txt)")
		if fileName:
			try:
				ret = self.loadNewMap_srv(1,fileName)

				xdim = ret.map.info.width
				ydim = ret.map.info.height
				res = ret.map.info.resolution
				
				self._widget.info_textbox.append("Map dimensions x%d * y%d, measured in absolute gridcells" %(xdim,ydim))
				self.mapdata = MapData(xdim,ydim,res,ret.map.data)
				
				self.updateView(xdim,ydim,fileName)

			except rospy.ServiceException, e:
				self._widget.info_textbox.append("No map could be read")
				self._widget.debug_textbrowser.append("Service call failed: %s"%e)
		
	#send a re-init msg to global planner		
	def reInit(self):
		msgBox = QMessageBox()
		msgBox.setText("Re-initialize the map and roadmap?")
		msgBox.setInformativeText("Do you want to proceed?")
		msgBox.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
		msgBox.setDefaultButton(QMessageBox.Cancel)
		ret = msgBox.exec_()
		if ret == QMessageBox.Ok:
			#do (re)init
			self._widget.info_textbox.append( "re-init")
			msg = user_init()
			msg.state = 1;
			self.re_init_pub.publish(msg)
		elif ret ==QMessageBox.Cancel:
			pass
			#do nothing
		else:
			pass
			#do nothing
	
	
	def updatePathCallback(self,path):
		if hasattr(self, "mapdata"):
			self.sigPathUpdate.emit(path)
		
	#on receiving new path data, redraw the path
	def updatePath(self,path):
		self.gScene.resetMap()
		self.drawEnvironmentMap()

		height = self.gScene.y
		wpradius = self.wpointsize/2
		
		#draw lines
		for p1,p2 in zip(path.poses,path.poses[1:]):
			pen = QPen(QtCore.Qt.blue)
			#pen.setWidth(1)
			
			self.gScene.addLine((p1.pose.position.x*self.scale),\
			height-((p1.pose.position.y*self.scale)),\
			(p2.pose.position.x*self.scale),\
			height-((p2.pose.position.y*self.scale)),pen)
		#draw dots		
		for i in path.poses:
			pen = QPen(QtCore.Qt.red)
			pen.setWidth(1)
			brush = QBrush(QtCore.Qt.red)
						
			self.gScene.addEllipse(((i.pose.position.x*self.scale)-wpradius),\
			height-((i.pose.position.y*self.scale)+wpradius),\
			self.wpointsize,self.wpointsize,pen, brush)
					
		self.gScene.update()	
		self._widget.graphicsView.fitInView(self.gScene.borderItem)
			
	#redraw the graphicsview 
	def updateView(self, x, y,fileName):
		try:
			print fileName

			self.gScene = EnvironmentMapScene(x,y)
			self.gScene.setBackgroundBrush(QColor('white'))
			self.drawEnvironmentMap()
			self._widget.graphicsView.setScene(self.gScene)
			self._widget.graphicsView.fitInView(self.gScene.borderItem)
			
		except rospy.ServiceException, e:
			print e
			
	#draw the map on the scene	
	def drawEnvironmentMap(self):
		height = self.gScene.y
		cellradius = 1
		
		for y in range(0, self.mapdata.ydim):
			for x in range(0, self.mapdata.xdim): 
				if self.mapdata.getOccupancyAt(x,y) == 100:
					self.gScene.addRect(x-cellradius,height-(y+cellradius),1,1,pen=QPen(QColor('black')),brush = QBrush(QColor('gray')))
	
	
	#method for udpate fixed waypoints and send update to environment 
	#todo. needs input control and error checking
	def updateFixedWaypoints(self):
		
		self.waypoints[:] = []
		self._widget.start_searcher.clear()
		self._widget.target_searcher.clear()
		
		for row in range(self._widget.tableWidget.rowCount()):
			data = []
			for col in range(6):
				item = self._widget.tableWidget.item(row,col)
				if item:
					data.extend([item.text()])
				else:
					data.extend([0])
			if data[1] and data[2]:	
				if not data[3]:
					data[3] = "0";
				self.waypoints.extend([Waypoint(data[1],data[2],data[3],data[0],data[4])])
			else:
				ind = row+1
				self._widget.info_textbox.append("-ERROR with fixed waypoint %d, check coordinates" %ind)				
		#add first of the list to drop down menu	
		for x in range(len(self.waypoints)):				
			self._widget.start_searcher.addItem(self.waypoints[x].name)
			self._widget.target_searcher.addItem(self.waypoints[x].name)
			
		# send waypoints update to global planner
		wps = []
		for	x in range(len(self.waypoints)):
			node = self.waypoints[x]		
			pos = Pose2D()
			pos.x = float(node.x)
			pos.y = float(node.y)
			pos.theta = float(node.theta)
			wps.extend([pos])
		try:
			ret = self.updateFixedWaypoints_srv(1,wps)
			self._widget.info_textbox.append("-updated fixed waypoints to current situation")				
		except rospy.ServiceException, e:
			self._widget.info_textbox.append("-failed update fixed waypoints ")
			self._widget.debug_textbrowser.append("Service call failed: %s"%e)
				
	#update start to selected in drop down menu
	def setCoordsStart(self, index):
		if index > -1:
			self._widget.start_x_input.setText(self.waypoints[index].x)
			self._widget.start_y_input.setText(self.waypoints[index].y)
			self._widget.start_th_input.setText(self.waypoints[index].theta)
		else:
			self._widget.start_x_input.setText('0')
			self._widget.start_y_input.setText('0')
			self._widget.start_th_input.setText('0')
	
	#update start to selected in drop down menu		
	def setCoordsTarget(self, index):
		if index > -1:
			self._widget.target_x_input.setText(self.waypoints[index].x)
			self._widget.target_y_input.setText(self.waypoints[index].y)
			self._widget.target_th_input.setText(self.waypoints[index].theta)
		else:
			self._widget.target_x_input.setText('0')
			self._widget.target_y_input.setText('0')
			self._widget.target_th_input.setText('0')
	
	#add a row to the table		
	def addRow(self):
		self._widget.tableWidget.insertRow(self._widget.tableWidget.rowCount())
	
	#remove row from the table
	def delRow(self):
		selected = self._widget.tableWidget.currentRow()
		self._widget.tableWidget.removeRow(selected)		

	def debugOutCallback(self, log):
		self.sigDebugOut.emit(log)
		
	#write debug information into the debug screen
	def debugOutput(self, log):
		self._widget.debug_textbrowser.append(str(log.header.stamp.secs) + " : "+ log.name + " --  " + log.msg)
	
	def toggleMeters(self):
		if self._widget.meters_toggle.isChecked():
			self._widget.meters_label.setText("Meters")
		else:
			self._widget.meters_label.setText("Gridcells")
			
	def emergencyBreakSend(self):
		msg = UInt8(1)
		self.embreak_pub.publish(msg)


#-------------------------------------------------------------------------------------------------------------------------------------------------------
	#load a demo map and set the start and end coordinates automatically
	def demoFunction(self):
		fileName = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../maps/testmap7.txt')
		try:
				ret = self.loadNewMap_srv(1,fileName)

				xdim = ret.map.info.width
				ydim = ret.map.info.height
				res = ret.map.info.resolution
				
				self._widget.info_textbox.append("Map dimensions x%d * y%d, measured in absolute gridcells" %(xdim,ydim))
				self.mapdata = MapData(xdim,ydim,res,ret.map.data)
				
				self.updateView(xdim,ydim,fileName)
				
								#demo coordinates
				self._widget.start_x_input.setText("0.01")
				self._widget.start_y_input.setText("0.01")
				self._widget.target_x_input.setText("4")
				self._widget.target_y_input.setText("4")
				self._widget.start_th_input.setText("0")
				self._widget.target_th_input.setText("180")
				
		except rospy.ServiceException, e:
				self._widget.info_textbox.append("no map could be read")
				self._widget.debug_textbrowser.append("Service call failed: %s"%e)
		

	
