import sdurw
import sdurw_kinematics
import sdurw_proximitystrategies
import sdurw_math
import numpy as np
import copy



def getConfigurations(nameGoal, nameTcp, robot, wc, state):
    # Get, make and print name of frames
    robotName = robot.getName()
    nameRobotBase = robotName + "." + "Base"
    nameRobotTcp = robotName + "." + "TCP"

    # Find frames and check for existence
    frameGoal = wc.findFrame(nameGoal)
    frameTcp = wc.findFrame(nameTcp)
    frameRobotBase = wc.findFrame(nameRobotBase)
    frameRobotTcp = wc.findFrame(nameRobotTcp)
    if frameGoal == None or frameTcp == None or frameRobotBase == None or frameRobotTcp == None:
        print(" ALL FRAMES NOT FOUND:")
        print(" Found \"", nameGoal, "\": ", "NO!" if frameGoal == None else "YES!")
        print(" Found \"", nameTcp, "\": ", "NO!" if frameTcp == None else "YES!")
        print(" Found \"", nameRobotBase, "\": ", "NO!" if frameRobotBase == None else "YES!")
        print(" Found \"", nameRobotTcp, "\": ", "NO!" if frameRobotTcp == None else "YES!")

    # Make "helper" transformations
    frameBaseTGoal = sdurw_kinematics.Kinematics_frameTframe(frameRobotBase, frameGoal, state)
    frameTcpTRobotTcp = sdurw_kinematics.Kinematics_frameTframe(frameTcp, frameRobotTcp, state)

    # get grasp frame in robot tool frame
    targetAt = frameBaseTGoal * frameTcpTRobotTcp
    closedFormSovler = sdurw.ClosedFormIKSolverUR(robot.cptr(), state)
    return closedFormSovler.solve(targetAt, state)



if __name__ == "__main__":
    #load workcell
    wc = sdurw.WorkCellLoaderFactory.load("Scene.wc.xml")
    if wc.isNull():
        raise Exception("COULD NOT LOAD scene... check path!")

    # find relevant frames
    
    cylinderFrame = wc.findMovableFrame("Cylinder")
    if cylinderFrame == None:
        raise Exception("COULD not find movable frame Cylinder ... check model")
        
    squareFrame = wc.findMovableFrame("Square")
    if squareFrame == None:
        raise Exception("COULD not find movable frame square ... check model")
    
    bottleFrame = wc.findMovableFrame("Bottle")
    if bottleFrame == None:
        raise Exception("COULD not find movable frame bottle ... check model")
    
    placeFrame = wc.findMovableFrame("PlaceLocation")
    if placeFrame == None:
        raise Exception("COULD not find movable frame PlaceLocation ... check model")
    
    refFrame = wc.findMovableFrame("URReference");

    robotUR5 = wc.findSerialDevice("UR-6-85-5-A")
    if robotUR5.isNull():
        raise Exception("COULD not find device UR5 ... check model")

    detector = sdurw.CollisionDetector(wc, sdurw_proximitystrategies.ProximityStrategyFactory_makeDefaultCollisionStrategy())


    # get the default state
    state = wc.getDefaultState()
    totalMax = 0
    bestX = -10
    bestY = -10
    bestCollisionFreeSolutions = [[],[],[],[]]
    for x in range(7):
        for y in range(9):
    
            collisionFreeSolutions = [[],[],[],[]]
            posX = round(-0.3+0.1*x,1) #round because for some reason it would sometimes give smalls errors
            posY = round(-0.5+0.1*y,1)
            refFrame.moveTo(sdurw.Transform3Dd(sdurw.Vector3Dd(posX,posY,0.0), refFrame.getTransform(state).R()), state)


            for rollAngle in range(360): # for every degree around the roll axis
                bottleFrame.moveTo(sdurw.Transform3Dd(sdurw.Vector3Dd(0, 0.474, 0.11), sdurw.RPYd(np.deg2rad(rollAngle),np.deg2rad(0),np.deg2rad(90))), state)
                squareFrame.moveTo(sdurw.Transform3Dd(sdurw.Vector3Dd(0.25, 0.474, 0.050), sdurw.RPYd(np.deg2rad(rollAngle),np.deg2rad(0),np.deg2rad(90))), state)
                cylinderFrame.moveTo(sdurw.Transform3Dd(sdurw.Vector3Dd(-0.25, 0.474, 0.050), sdurw.RPYd(np.deg2rad(rollAngle),np.deg2rad(0),np.deg2rad(90))), state)
                placeFrame.moveTo(sdurw.Transform3Dd(sdurw.Vector3Dd(0.3, -0.5, 0.050), sdurw.RPYd(np.deg2rad(rollAngle),0,0)), state)

                solutions = []
                solutions.append(getConfigurations("GraspBottle", "GraspTCP", robotUR5, wc, state))
                solutions.append(getConfigurations("GraspSquare", "GraspTCP", robotUR5, wc, state))
                solutions.append(getConfigurations("GraspCylinder", "GraspTCP", robotUR5, wc, state))
                solutions.append(getConfigurations("PlaceTarget", "GraspTCP", robotUR5, wc, state))

                for i in range(len(solutions)):
                    for j in range(len(solutions[i])):
	            	# set the robot in that configuration and check if it is in collision
                        robotUR5.setQ(solutions[i][j], state)
                        res1 = sdurw.ProximityData()
                        if not detector.inCollision(state, res1):
                            collisionFreeSolutions[i].append(solutions[i][j]) # save it
                            break # we only need one
            print("------------------------------------------------------------------------")
            print("collision free inverse kinematics solutions at (",posX,",",posY,") : ")
            print("for the bottle : ",len(collisionFreeSolutions[0]))
            print("for the square : ",len(collisionFreeSolutions[1]))
            print("for the cylinder : ",len(collisionFreeSolutions[2]))
            print("for the place location : ",len(collisionFreeSolutions[3]))
            total = len(collisionFreeSolutions[0]) +len(collisionFreeSolutions[1])+len(collisionFreeSolutions[2])+len(collisionFreeSolutions[3])
            print("TOTAL (",posX,",",posY,") ; ", total)
                #print("Current position of the robot vs object to be grasped has: ", len(collisionFreeSolutions), " collision-free inverse kinematics solutions!")
            if(total > totalMax):
                totalMax = total
                bestX = posX
                bestY = posY
                bestCollisionFreeSolutions =  [[],[],[],[]]
                #copy collusion free solutions into the best solutions
                for i in range(len(collisionFreeSolutions)):
                    for j in range(len(collisionFreeSolutions[i])):
                        bestCollisionFreeSolutions[i].append(collisionFreeSolutions[i][j]) # save it
                        
                
    print("--------------------------------------------------------------")
    print("Best position : (",bestX,",",bestY,"), total : ", totalMax)
    #move back the robot at it's best position for the replay
    refFrame.moveTo(sdurw.Transform3Dd(sdurw.Vector3Dd(bestX,bestY,0.0), refFrame.getTransform(state).R()), state)   
    # visualize them
    tStatePath = sdurw.PathTimedState()
    time=0
    for i in range(len(bestCollisionFreeSolutions)):
        for j in range(len(bestCollisionFreeSolutions[i])):
            robotUR5.setQ(bestCollisionFreeSolutions[i][j], state)
            tStatePath.push_back(sdurw.TimedState(time,state))
            time+=0.01

    tStatePath.save("visu.rwplay", wc)



