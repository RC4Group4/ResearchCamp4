#!/usr/bin/python
import time
import os
import sys
import types
import thread
import commands

# ROS imports
import roslib
roslib.load_manifest('raw_script_server')
import rospy
import actionlib

from raw_script_server.msg import *


# graph includes
import pygraphviz as pgv

graph=""
graph_wait_list=[]
function_counter = 0
ah_counter = 0
graph = pgv.AGraph()
graph.node_attr['shape']='box'
last_node = "Start"

## Script class from which all script inherit.
#
# Implements basic functionalities for all scripts.
class script():
	def __init__(self):
		# use filename as nodename
		filename = os.path.basename(sys.argv[0])
		self.basename, extension = os.path.splitext(filename)
		rospy.init_node(self.basename)
		self.graph_pub = rospy.Publisher("/script_server/graph", String)
		print "heyyyy"

	## Dummy function for initialization
	def Initialize(self):
		pass

	## Dummy function for main run function
	def Run(self):
		pass

	## Function to start the script
	#
	# First does a simulated turn and then calls Initialize() and Run().
	def Start(self):
		self.Parse()
		global ah_counter
		ah_counter = 0
		self.sss = simple_script_server()
		rospy.loginfo("Starting <<%s>> script...",self.basename)
		self.Initialize()
		self.Run()
		# wait until last threaded action finishes
		rospy.loginfo("Wait for script to finish...")
		while ah_counter != 0:
			rospy.sleep(1)
		rospy.loginfo("...script finished.")
	
	## Function to generate graph view of script.
	#
	# Starts the script in simulation mode and calls Initialize() and Run().
	def Parse(self):
		rospy.loginfo("Start parsing...")
		global graph
		global function_counter
		function_counter = 0
		# run script in simulation mode
		self.sss = simple_script_server(parse=True)
		self.Initialize()
		self.Run()
		
		# save graph on parameter server for further processing
#		self.graph = graph
		rospy.set_param("/script_server/graph", graph.string())
		self.graph_pub.publish(graph.string())
		rospy.loginfo("...parsing finished")
		function_counter = 0
		return graph.string()

## Simple script server class.
#
# Implements the python interface for the script server.
class simple_script_server:
	## Initializes simple_script_server class.
	#
	# \param parse Defines wether to run script in simulation for graph generation or not
	def __init__(self, parse=False):
		global graph
		self.ns_global_prefix = "/script_server"
		self.wav_path = ""
		self.parse = parse
		
		# init light publisher
		self.arm_client = actionlib.SimpleActionClient('/katana_arm_controller/joint_movement_action', katana_msgs.msg.JointMovementAction)

		rospy.sleep(1) # we have to wait here until publishers are ready, don't ask why


	def stop(self,component_name):
		return self.trigger(component_name,"stop")

	
	def trigger(self,component_name,service_name,blocking=True, planning=False):
		ah = action_handle(service_name, component_name, "", blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()

		rospy.loginfo("<<%s>> <<%s>>", service_name, component_name)
		rospy.loginfo("Wait for <<%s>> to <<%s>>...", component_name, service_name)
		service_full_name = "/" + component_name + "_controller/" + service_name
		
		# check if service is available
		try:
			rospy.wait_for_service(service_full_name,rospy.get_param('server_timeout',3))
		except rospy.ROSException, e:
			error_message = "%s"%e
			rospy.logerr("...<<%s>> service of <<%s>> not available, error: %s",service_name, component_name, error_message)
			ah.set_failed(4)
			return ah
		
		# check if service is callable
		try:
			init = rospy.ServiceProxy(service_full_name,Trigger)
			#print init()
			resp = init()
		except rospy.ServiceException, e:
			error_message = "%s"%e
			rospy.logerr("...calling <<%s>> service of <<%s>> not successfull, error: %s",service_name, component_name, error_message)
			ah.set_failed(10)
			return ah
		
		# evaluate sevice response
		if not resp.success.data:
			rospy.logerr("...<<%s>> <<%s>> not successfull, error: %s",service_name, component_name, resp.error_message.data) 
			ah.set_failed(10)
			return ah
		
		# full success
		rospy.loginfo("...<<%s>> is <<%s>>", component_name, service_name)
		ah.set_succeeded() # full success
		return ah

	def move(self, component_name,parameter_name,blocking=True):
		if component_name == "base":
			return self.move_base(component_name,parameter_name,blocking)
		elif component_name == "arm":
			return self.move_arm(component_name,parameter_name,blocking)
		elif component_name == "arm_traj":
			return self.move_arm_traj(component_name,parameter_name,blocking)
		elif component_name == "gripper":
			return self.move_gripper(component_name,parameter_name,blocking)
		elif component_name == "head":
			return self.move_head(component_name,parameter_name,blocking)

	def move_base(self, component_name, parameter_name, blocking):
		ah = action_handle("move", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		rospy.loginfo("Move <<%s>> to <<%s>>", component_name, parameter_name)
		
		action_server_name = '/vb_2dnav/MoveBaseToLocation'
				
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, vb_2dnav.msg.MoveBaseToLocationAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal		
		client_goal = vb_2dnav.msg.MoveBaseToLocationGoal(location_name=parameter_name)

		#print client_goal
		client.send_goal(client_goal)
		ah.set_client(client)

		ah.wait_inside()

		return ah
	
	def move_arm(self, component_name, parameter_name, blocking):
		ah = action_handle("move", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		rospy.loginfo("Move <<%s>> to <<%s>>", component_name, parameter_name)
		
		action_server_name = '/katana_arm_controller/joint_movement_action'
		
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, katana_msgs.msg.JointMovementAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal		
		joint_names = rospy.get_param('/script_server/' + component_name + '/joint_names')
		joint_velocity = rospy.get_param('/script_server/' + component_name + '/velocity')	
		joint_config = rospy.get_param('/script_server/' + component_name + '/' + parameter_name)
		
		client_goal = katana_msgs.msg.JointMovementGoal()
		client_goal.jointGoal.name = joint_names
		client_goal.jointGoal.velocity = joint_velocity
		client_goal.jointGoal.position = joint_config[0]
			
		#print client_goal
		client.send_goal(client_goal)
	
		ah.set_client(client)
		ah.wait_inside()

		return ah
	
	def move_arm_traj(self, component_name, parameter_name, blocking):
		ah = action_handle("move", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		rospy.loginfo("Move <<%s>> to <<%s>>", component_name, parameter_name)
		
		action_server_name = '/katana_arm_controller/joint_trajectory_action'
		
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, pr2_controllers_msgs.msg.JointTrajectoryAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		joint_names = rospy.get_param('/script_server/' + component_name + '/joint_names')
		joint_velocity = rospy.get_param('/script_server/' + component_name + '/velocity')	
		joint_config = rospy.get_param('/script_server/' + component_name + '/' + parameter_name)

		goal_config = pr2_controllers_msgs.msg.JointTrajectoryGoal()
		goal_config.trajectory.joint_names = joint_names
		
		for item in joint_config:
			point_in_traj = rospy.get_param('/script_server/arm/' + item)
							
			###### WORKAROUND SINCE TRAJECTOTY STUFF IS NOT WORKING YET IN THE KATANA LIB
			goal_config = katana_msgs.msg.JointMovementGoal()
			#goal_config.jointGoal.header = rospy.Time.now()
			goal_config.jointGoal.name = joint_names
			goal_config.jointGoal.velocity = joint_velocity
			goal_config.jointGoal.position = point_in_traj[0]
			
			self.arm_client.send_goal(goal_config)
			self.arm_client.wait_for_result()		

		ah.set_client(client)
		ah.wait_inside()

		return ah

	def move_gripper(self, component_name, parameter_name, blocking):
		ah = action_handle("move", component_name, parameter_name, blocking, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		rospy.loginfo("Move <<%s>> to <<%s>>", component_name, parameter_name)
		
		action_server_name = '/gripper_grasp_posture_controller'
				
		rospy.logdebug("calling %s action server",action_server_name)
		client = actionlib.SimpleActionClient(action_server_name, object_manipulation_msgs.msg.GraspHandPostureExecutionAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.set_failed(4)
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal		
		gripper_state = rospy.get_param('/script_server/' + component_name + '/' + parameter_name)
					
		goal_config = object_manipulation_msgs.msg.GraspHandPostureExecutionGoal()
		goal_config.grasp.grasp_posture.position = [0.0] ### dummy value
		goal_config.goal = gripper_state
		
		#print client_goal
		client.send_goal(goal_config)
		ah.set_client(client)

		ah.wait_inside()

		return ah

	

#------------------- General section -------------------#
	## Sleep for a certain time.
	#
	# \param duration Duration in seconds to sleep.
	#
	def sleep(self,duration):
		ah = action_handle("sleep", "", str(duration), True, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		rospy.loginfo("Wait for %f sec",duration)
		rospy.sleep(duration)
		
		ah.set_succeeded()

	## Waits for user input.
	#
	# Waits either for a user input or until timeout is reached.
	#
	# \param duration Duration in seconds for timeout.
	# 
	# \todo TODO: implement waiting for timeout
	def wait_for_input(self,duration=0):
		ah = action_handle("wait", "input", str(duration), True, self.parse)
		if(self.parse):
			return ah
		else:
			ah.set_active()
		
		if (duration != 0):
			rospy.logerr("Wait with duration not implemented yet") # \todo TODO: implement waiting with duration
		
		rospy.loginfo("Wait for user input...")
		retVal = raw_input()
		rospy.loginfo("...got string <<%s>>",retVal)
		ah.set_succeeded()
		return retVal

#------------------- action_handle section -------------------#	
## Action handle class.
#
# The action handle is used to implement asynchronous behaviour within the script.
class action_handle:
	## Initializes the action handle.
	def __init__(self, function_name, component_name, parameter_name, blocking, parse):
		global graph
		global function_counter
		self.parent_node = ""
		self.error_code = -1
		self.wait_log = False
		self.function_counter = function_counter
		self.function_name = function_name
		self.component_name = component_name
		self.parameter_name = parameter_name
		self.state = ScriptState.UNKNOWN
		self.blocking = blocking
		self.parse = parse
		self.level = int(rospy.get_param("/script_server/level",100))
		self.state_pub = rospy.Publisher("/script_server/state", ScriptState)
		self.AppendNode(blocking)
		self.client = actionlib.SimpleActionClient("dummy",ScriptAction)

	## Sets the actionlib client.
	def set_client(self,client):
		self.client = client

	## Sets the execution state to active, if not paused
	def set_active(self):
		self.check_pause()
		self.state = ScriptState.ACTIVE
		self.error_code = -1
		self.PublishState()
		
		global ah_counter
		ah_counter += 1
		
	## Checks for pause
	def check_pause(self):
		param_string = "/script_server/pause"
		while bool(rospy.get_param(param_string,False)):
			rospy.logwarn("Script is paused...")
			self.state = ScriptState.PAUSED
			self.PublishState()
			rospy.sleep(1)
		if self.state == ScriptState.PAUSED:
			rospy.loginfo("...continuing script")
		
	## Sets the execution state to succeeded.
	def set_succeeded(self):
		self.state = ScriptState.SUCCEEDED
		self.error_code = 0
		self.PublishState()
		
		global ah_counter
		ah_counter -= 1
		
	## Sets the execution state to failed.
	def set_failed(self,error_code):
		self.state = ScriptState.FAILED
		self.error_code = error_code
		self.PublishState()

		global ah_counter
		ah_counter -= 1
		
	## Gets the state of an action handle.
	def get_state(self):
		return self.client.get_state()

	## Gets the error code of an action handle.
	def get_error_code(self):
		return self.error_code
	
	## Returns the graphstring.
	def GetGraphstring(self):
		if type(self.parameter_name) is types.StringType:
			graphstring = str(self.function_counter)+"_"+self.function_name+"_"+self.component_name+"_"+self.parameter_name
		else:
			graphstring = str(self.function_counter)+"_"+self.function_name+"_"+self.component_name
		return graphstring

	## Gets level of function name.
	def GetLevel(self,function_name):
		if (function_name == "move"):
			level = 0
		elif (function_name == "init"):
			level = 1
		elif (function_name == "stop"):
			level = 1
		elif (function_name == "sleep"):
			level = 2
		else:
			level = 100
		return level
		
	## Appends a registered function to the graph.
	def AppendNode(self, blocking=True):
		global graph
		global graph_wait_list
		global function_counter
		global last_node
		graphstring = self.GetGraphstring()
		if self.parse:
			if ( self.level >= self.GetLevel(self.function_name)):
				#print "adding " + graphstring + " to graph"
				graph.add_edge(last_node, graphstring)
				for waiter in graph_wait_list:
					graph.add_edge(waiter, graphstring)
				graph_wait_list=[]
				if blocking:
					last_node = graphstring
				else:
					self.parent_node = graphstring
			#else:
				#print "not adding " + graphstring + " to graph"
		#else:
			#self.PublishState()
		function_counter += 1
		
	## Publishs the state of the action handle
	def PublishState(self):
		script_state = ScriptState()
		script_state.header.stamp = rospy.Time.now()
		script_state.number = self.function_counter
		script_state.function_name = self.function_name
		script_state.component_name = self.component_name
		script_state.full_graph_name = self.GetGraphstring()
		if ( type(self.parameter_name) is str ):
			script_state.parameter_name = self.parameter_name
		else:
			script_state.parameter_name = ""
		script_state.state = self.state
		script_state.error_code = self.error_code
		self.state_pub.publish(script_state)
		
	## Handles wait.
	#
	# This function is meant to be uses directly in the script.
	#
	# \param duration Duration for timeout.
	def wait(self, duration=None):
		global ah_counter
		ah_counter += 1
		self.blocking = True
		self.wait_for_finished(duration,True)

	## Handles inside wait.
	#
	# This function is meant to be uses inside the simple_script_server.
	#
	# \param duration Duration for timeout.
	def wait_inside(self, duration=None):
		if self.blocking:
			self.wait_for_finished(duration,True)
		else:
			thread.start_new_thread(self.wait_for_finished,(duration,False,))
		return self.error_code
	
	## Waits for the action to be finished.
	#
	# If duration is specified, waits until action is finished or timeout is reached.
	#
	# \param duration Duration for timeout.
	# \param logging Enables or disables logging for this wait.
	def wait_for_finished(self, duration, logging):
		global graph_wait_list
		if(self.parse):
			if(self.parent_node != ""):
				graph_wait_list.append(self.parent_node)
			return

		if self.error_code <= 0:			
			if duration is None:
				if logging:
					rospy.loginfo("Wait for <<%s>> reaching <<%s>>...",self.component_name, self.parameter_name)
				self.client.wait_for_result()
			else:
				if logging:
					rospy.loginfo("Wait for <<%s>> reached <<%s>> (max %f secs)...",self.component_name, self.parameter_name,duration)
				if not self.client.wait_for_result(rospy.Duration(duration)):
					if logging:
						rospy.logerr("Timeout while waiting for <<%s>> to reach <<%s>>. Continuing...",self.component_name, self.parameter_name)
					self.set_failed(10)
					return
			# check state of action server
			#print self.client.get_state()
			if self.client.get_state() != 3:
				if logging:
					rospy.logerr("...<<%s>> could not reach <<%s>>, aborting...",self.component_name, self.parameter_name)
				self.set_failed(11)
				return

			if logging:
				rospy.loginfo("...<<%s>> reached <<%s>>",self.component_name, self.parameter_name)
		else:
			rospy.logwarn("Execution of <<%s>> to <<%s>> was aborted, wait not possible. Continuing...",self.component_name, self.parameter_name)
			self.set_failed(self.error_code)
			return
			
		self.set_succeeded() # full success
