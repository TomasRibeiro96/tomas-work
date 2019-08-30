import pybullet as p
import time
import pybullet_data
import inspect

usePhysX = True

print(inspect.getfile(p))

if usePhysX:
    p.connect(p.PhysX)
    p.loadPlugin("eglRendererPlugin")
else:
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf", useFixedBase=True)

'''
------ URDF FILES --------
Can't load just one file with all the bodies
because the simulation will crash.
Need to create different files for each
object in the scene
'''
targetID = p.loadURDF("URDF-target.xml", flags=p.URDF_USE_SELF_COLLISION or p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT)
boxID = p.loadURDF("URDF-box.xml")
robotID = p.loadURDF("URDF-robot_arm.xml", useFixedBase=True)

p.changeDynamics(boxID, -1, lateralFriction=.3, spinningFriction=.3, rollingFriction=.3)
p.setCollisionFilterPair(boxID, targetID, -1, -1, 0)

num_joints = p.getNumJoints(robotID)

for i in range(num_joints):
    x = p.getJointInfo(robotID, i)
    print("-------------------------------------------------------------------------------------------------")
    print("ID: ", x[0], " || Name: ", x[1], " || Type: ", x[2], " || qIndex: ", x[3], " || uIndex ", x[4])
    print("Flags: ", x[5], " || jointDamping: ", x[6], " || jointFriction: ", x[7], " || jointLowerLimit: ", x[8])
    print("JointUpperLimit: ", x[9], " || JointMaxForce: ", x[10], " || jointMaxVelocity: ", x[11])
    print("linkName: ", x[12], " || jointAxis: ", x[13])
    print("parentFramePos: ", x[14], " || ParentFrameOrn: ", x[15], " || parentIndex: ", x[16])
    print("-------------------------------------------------------------------------------------------------")

for i in range(100000):
    p.stepSimulation()
    time.sleep(1./240.)
    #print(p.getBasePositionAndOrientation(boxID))

#print(cubePos,cubeOrn)

p.disconnect()
