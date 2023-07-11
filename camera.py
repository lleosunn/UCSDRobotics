import os, inspect
import pybullet as p
import pybullet_data
import math
import time

# Simulation
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
parentdir = os.path.join(currentdir, "../gym")
os.sys.path.insert(0, parentdir)
cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -10)
useRealTimeSim = 1

#for video recording (works best on Mac and Linux, not well on Windows)
#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "racecar.mp4")
p.setRealTimeSimulation(useRealTimeSim)  # either this
#p.loadURDF("plane.urdf")
p.loadSDF(os.path.join(pybullet_data.getDataPath(), "stadium.sdf"))

# Car
car = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "racecar/racecar.urdf"))
for i in range(p.getNumJoints(car)):
  print(p.getJointInfo(car, i))
inactive_wheels = [5, 7]
wheels = [2, 3]
for wheel in inactive_wheels:
  p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
steering = [4, 6]


BARRIER = [[5, 5], [5, -1], [-1, -1], [-1, 5]]
#BOX IMPLEMENTATION 
box_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.4, 0.4, 0.4])

# Create the box body
for coordinate in BARRIER:
	box_body = p.createMultiBody(
        baseMass=1.0,  # Mass of the box
        baseCollisionShapeIndex=box_shape,  # Corrected argument name
        basePosition=[coordinate[0], coordinate[1], 1],  # Initial position of the box
    )

# Move to Point function
def moveTo(targetX, targetY):
  pos, hquat = p.getBasePositionAndOrientation(car)
  h = p.getEulerFromQuaternion(hquat)
  x = pos[0]
  y = pos[1]
  distance = math.sqrt((targetX - x)**2 + (targetY - y)**2)
  theta = math.atan2((targetY - y), (targetX - x))
  while distance > 1:
    pos, hquat = p.getBasePositionAndOrientation(car)
    h = p.getEulerFromQuaternion(hquat)
    x = pos[0]
    y = pos[1]
    distance = math.sqrt((targetX - x)**2 + (targetY - y)**2)
    theta = math.atan2((targetY - y), (targetX - x))
    maxForce = 20
    targetVelocity = 5*distance
    steeringAngle = theta - h[2]
    if steeringAngle > (math.pi / 2) or steeringAngle < -(math.pi / 2):
       steeringAngle = h[2] - theta
    else:
       steeringAngle = theta - h[2]


    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    camData = p.getDebugVisualizerCamera()
    viewMat = camData[2]
    projMat = camData[3]
    p.getCameraImage(256, 256, viewMatrix=viewMat, projectionMatrix=projMat, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    p.resetDebugVisualizerCamera(
      cameraDistance = 0.1,
      cameraYaw = math.degrees(h[2]) - 90, 
      cameraPitch = -20,
      cameraTargetPosition = [pos[0] + math.cos(h[2]), pos[1] + math.sin(h[2]), pos[2] + 0.1],
      physicsClientId=0
    )



    for wheel in wheels:
        p.setJointMotorControl2(car,
                            wheel,
                            p.VELOCITY_CONTROL,
                            targetVelocity=targetVelocity,
                            force=maxForce)
    for steer in steering:
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)
    steering
    if (useRealTimeSim == 0):
        p.stepSimulation()
    time.sleep(0.01)

# Waypoints
points = [(2, 0), (2, 2), (0, 2), (0, 0)]

for i in points:
   moveTo(i[0], i[1])
   

