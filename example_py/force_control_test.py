import sys
sys.path.insert(0, "../lib_py")
import flexivrdk
import time
import numpy as np
import serial




robot_states = flexivrdk.RobotStates()
log = flexivrdk.Log()
mode = flexivrdk.Mode
log.info("Tutorial description:")
robot = flexivrdk.Robot("192.168.2.100", "192.168.2.105")

ser = serial.Serial('/dev/ttyACM0',115200)
cmd_close = [0x7b,0x01,0x02,0x01,0x20,0x46,0x50,0x01,0x2c,0x62,0x7d]
cmd_open = [0x7b,0x01,0x02,0x00,0x20,0x46,0x50,0x01,0x2c,0x63,0x7d]

ser.write(serial.to_bytes(cmd_close))
time.sleep(1.0)
ser.write(serial.to_bytes(cmd_open))
time.sleep(1.0)


robot.enable()
while not robot.isOperational():
            time.sleep(1)
robot.setMode(mode.NRT_PRIMITIVE_EXECUTION)
robot.executePrimitive("Home()")

while robot.isBusy():
	time.sleep(0.1)
	print("waiting for robot go Home")


robot.executePrimitive("ZeroFTSensor()")
while robot.isBusy():
	time.sleep(0.1)
	print("waiting for zero force sensor.")

robot.getRobotStates(robot_states)
init_pose = robot_states.tcpPose.copy()
SEARCH_VELOCITY = 0.3
PRESSING_FORCE = 1.0
SWING_FREQ = 0.3
SWING_AMP = 0.1
SEARCH_DISTANCE = 0.2

MAX_WRENCH_FOR_CONTACT_SEARCH = [10.0, 10.0, 10.0, 5.0, 5.0, 5.0]
robot.setMode(mode.NRT_CARTESIAN_MOTION_FORCE)
time.sleep(0.5)

robot.setMaxContactWrench(MAX_WRENCH_FOR_CONTACT_SEARCH)
time.sleep(0.1)
robot.writeDigitalOutput([0], [True])

######################################
# MAX_WRENCH_FOR_CONTACT_SEARCH = [1, 1, 1, 1, 1, 1]
# robot.setMode(mode.NRT_CARTESIAN_MOTION_FORCE)
# time.sleep(0.5)

# robot.setMaxContactWrench(MAX_WRENCH_FOR_CONTACT_SEARCH)
# time.sleep(0.1)
# robot.writeDigitalOutput([0], [True])
# target_pose = [0.6715241074562073, -0.11414425820112228, 0.4243335723876953, 0.0015436437679454684, -0.35593071579933167, 0.9343850016593933, -0.015347708947956562]
# robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
# import rospy
# while True:
# 	robot.getRobotStates(robot_states)
# 	current_pose = robot_states.tcpPose.copy()
# 	print("current pose: ", current_pose)
# 	now = rospy.get_rostime()
# 	print("Current time: ", now.secs, now.nsecs)
######################################


while True:
	target_pose = [0.6715241074562073, -0.11414425820112228, 0.4243335723876953, 0.0015436437679454684, -0.35593071579933167, 0.9343850016593933, -0.015347708947956562]
	robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
	while True:
		robot.getRobotStates(robot_states)
		current_pose = robot_states.tcpPose.copy()
		if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
			print(f"Reached target pose: {target_pose}")
			break
		time.sleep(0.25)
	target_pose = [0.6301706433296204, -0.14819204807281494, 0.6074711680412292, 0.12268102914094925, -0.34803590178489685, 0.9214996099472046, -0.12107354402542114]
	robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
	while True:
		robot.getRobotStates(robot_states)
		current_pose = robot_states.tcpPose.copy()
		if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
			print(f"Reached target pose: {target_pose}")
			break
		time.sleep(0.25)

	ser.write(serial.to_bytes(cmd_open))
	time.sleep(1.5)
	target_pose = [0.9057360887527466, -0.18640251457691193, 0.45811301469802856, 0.36958619952201843, -0.31538552045822144, 0.8557116985321045, -0.17803239822387695]
	robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
	while True:
		robot.getRobotStates(robot_states)
		current_pose = robot_states.tcpPose.copy()
		if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
			print(f"Reached target pose: {target_pose}")
			break
		time.sleep(0.25)
	ser.write(serial.to_bytes(cmd_close))
	time.sleep(1.5)
	target_pose = [0.8503658175468445, -0.18545803427696228, 0.5599230527877808, 0.39792510867118835, -0.3125131130218506, 0.8392051458358765, -0.19931353628635406]
	robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
	while True:
		robot.getRobotStates(robot_states)
		current_pose = robot_states.tcpPose.copy()
		if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
			print(f"Reached target pose: {target_pose}")
			break
		time.sleep(0.25)

	target_pose = [0.4272298812866211, -0.1001782938838005, 0.6095005869865417, -0.03239191696047783, -0.36401522159576416, 0.9302775859832764, -0.0320514515042305]
	robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
	while True:
		robot.getRobotStates(robot_states)
		current_pose = robot_states.tcpPose.copy()
		if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
			print(f"Reached target pose: {target_pose}")
			break
		time.sleep(0.25)

	target_pose = [0.4194954037666321, -0.03879353404045105, 0.293435275554657, -0.0034433763939887285, -0.3782423138618469, 0.9255666732788086, 0.015724660828709602]
	robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
	while True:
		robot.getRobotStates(robot_states)
		current_pose = robot_states.tcpPose.copy()
		if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
			print(f"Reached target pose: {target_pose}")
			break
		time.sleep(0.25)
	ser.write(serial.to_bytes(cmd_open))
	time.sleep(1.0)

	target_pose = [0.45398491621017456, -0.0588693730533123, 0.5433620810508728, -0.035461168736219406, -0.35336393117904663, 0.9341520667076111, -0.03516160324215889]
	robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
	while True:
		robot.getRobotStates(robot_states)
		current_pose = robot_states.tcpPose.copy()
		if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
			print(f"Reached target pose: {target_pose}")
			break
		time.sleep(0.25)

	target_pose = [0.6326221823692322, -0.10481802374124527, 0.46692779660224915, -0.011052127927541733, -0.38576918840408325, 0.9217559099197388, -0.0377623587846756]
	robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
	while True:
		robot.getRobotStates(robot_states)
		current_pose = robot_states.tcpPose.copy()
		if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
			print(f"Reached target pose: {target_pose}")
			break
		time.sleep(0.25)

# target_pose = [0.6740550398826599, -0.057421136647462845, 0.41261452436447144, -0.01638549007475376, -0.3996100425720215, 0.9155311584472656, -0.04296479746699333]
# robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
# while True:
# 	robot.getRobotStates(robot_states)
# 	current_pose = robot_states.tcpPose.copy()
# 	if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
# 		print(f"Reached target pose: {target_pose}")
# 		break
# 	time.sleep(0.25)
# target_pose = [0.7827813625335693, -0.023120228201150894, 0.06880372017621994, 0.045938849449157715, -0.4157485067844391, 0.908318281173706, 0.0008334366139024496]
# robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
# while True:
# 	robot.getRobotStates(robot_states)
# 	current_pose = robot_states.tcpPose.copy()
# 	if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
# 		print(f"Reached target pose: {target_pose}")
# 		break
# 	time.sleep(0.25)
# target_pose = [0.8741381168365479, -0.017693594098091125, 0.036857713013887405, 0.5321939587593079, -0.3394416868686676, 0.757969081401825, -0.16441357135772705]
# robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
# while True:
# 	robot.getRobotStates(robot_states)
# 	current_pose = robot_states.tcpPose.copy()
# 	if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
# 		print(f"Reached target pose: {target_pose}")
# 		break
# 	time.sleep(0.25)
# target_pose = [0.7284878492355347, -0.07416847348213196, 0.2608020305633545, 0.20836521685123444, -0.4204094111919403, 0.8806778192520142, -0.065164715051651]
# robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
# while True:
# 	robot.getRobotStates(robot_states)
# 	current_pose = robot_states.tcpPose.copy()
# 	if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
# 		print(f"Reached target pose: {target_pose}")
# 		break
# 	time.sleep(0.25)
# target_pose = [0.505584716796875, -0.07161444425582886, 0.5622490048408508, 0.1765972226858139, -0.4499531686306, 0.8729965686798096, -0.065058134496212]
# robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
# while True:
# 	robot.getRobotStates(robot_states)
# 	current_pose = robot_states.tcpPose.copy()
# 	if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
# 		print(f"Reached target pose: {target_pose}")
# 		break
# 	time.sleep(0.25)
# target_pose = [0.3890444338321686, -0.025291375815868378, 0.36880943179130554, 0.010727300308644772, -0.37737345695495605, 0.9258661270141602, 0.015686659142374992]
# robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
# while True:
# 	robot.getRobotStates(robot_states)
# 	current_pose = robot_states.tcpPose.copy()
# 	if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
# 		print(f"Reached target pose: {target_pose}")
# 		break
# 	time.sleep(0.25)
# target_pose = [0.6638741493225098, -0.059881359338760376, 0.36046889424324036, -0.0002676227886695415, -0.38505733013153076, 0.9227932691574097, -0.013541796244680882]
# robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)
# while True:
# 	robot.getRobotStates(robot_states)
# 	current_pose = robot_states.tcpPose.copy()
# 	if sum(abs(np.array(current_pose) - np.array(target_pose))) < 0.05:
# 		print(f"Reached target pose: {target_pose}")
# 		break
# 	time.sleep(0.25)

print("finished")
period = 0.1
while True:
	# Use sleep to control loop period
	time.sleep(period)

	# Monitor fault on robot server
	if robot.isFault():
	    raise Exception("Fault occurred on robot server, exiting ...")