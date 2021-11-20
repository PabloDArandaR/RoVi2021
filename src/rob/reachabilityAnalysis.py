import sdurw
import sdurw_kinematics
import sdurw_proximitystrategies
import sdurw_math
import numpy as np
import copy
import matplotlib.pyplot as plt
import seaborn as sns

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
    print(f"Target is: {targetAt}")
    closedFormSovler = sdurw.ClosedFormIKSolverUR(robot.cptr(), state)
    return closedFormSovler.solve(targetAt, state)



if __name__ == "__main__":
    # Basic variables to iterate adequately
    n_x = 10
    n_y = 15
    x_max = 0.325
    y_max = 0.525
    y_dims = np.zeros((n_x, n_y))
    x_dims = np.zeros((n_x, n_y))
    for i in range(0, n_x):
        for j in range(0, n_y):
            y_dims[i,j] = round(-y_max + (2*y_max/n_y)*j,3)
            x_dims[i,j] = round(-x_max + (2*x_max/n_x)*i,3)
    
    bottleLateral = np.zeros((n_x, n_y))
    bottleUp = np.zeros((n_x, n_y))
    objectiveLateral = np.zeros((n_x, n_y))
    objectiveUp = np.zeros((n_x, n_y))

    #load workcell
    wc = sdurw.WorkCellLoaderFactory.load("resources/Project_WorkCell/Scene.wc.xml")
    if wc.isNull():
        raise Exception("COULD NOT LOAD scene... check path!")

    # find relevant frames
    
    bottleFrame = wc.findMovableFrame("Bottle")
    if bottleFrame == None:
        raise Exception("COULD not find movable frame bottle ... check model")
    
    placeFrame = wc.findMovableFrame("Objective")
    if placeFrame == None:
        raise Exception("COULD not find movable frame ObjectiveLateral ... check model")
    
    refFrame = wc.findMovableFrame("URReference");

    robotUR5 = wc.findSerialDevice("UR-6-85-5-A")
    if robotUR5.isNull():
        raise Exception("COULD not find device UR5 ... check model")

    gripper = wc.findDevice("WSG50")
    if gripper.isNull():
        raise Exception("COULD not find device WSG50 ... check model")


    detector = sdurw.CollisionDetector(wc, sdurw_proximitystrategies.ProximityStrategyFactory_makeDefaultCollisionStrategy())

    # get the default state
    state = wc.getDefaultState()
    jointPos = sdurw_math.Q(1)
    jointPos[0] = 0.055
    gripper.setQ(jointPos, state)
    
    for i in range(0, n_x):
        for j in range(0, n_y):
            posX = x_dims[i,j]
            posY = y_dims[i,j]

            refFrame.moveTo(sdurw.Transform3Dd(sdurw.Vector3Dd(posX, posY, 0.0), refFrame.getTransform(state).R()), state)

            ## Check for the Bottle Lateral
            nValidSolutions = 0
            for rollAngle in range(360):
                bottleFrame.moveTo(sdurw.Transform3Dd(bottleFrame.getTransform(state).P(), sdurw.RPYd(np.deg2rad(rollAngle),np.deg2rad(0),np.deg2rad(90))), state)

                # Finding solutions for the given angle
                solutions = []
                solutions.append(getConfigurations("Bottle", "GraspTCP", robotUR5, wc, state))

                # Check if the solution is actually a valid solution
                for k in range(len(solutions)):

                    for l in range(len(solutions[k])):
                        robotUR5.setQ(solutions[k][l], state)
                        res1 = sdurw.ProximityData()
                        if not detector.inCollision(state, res1):
                            nValidSolutions += 1
                            break
        
            print(f" - In bottle lateral, number of solutions: {nValidSolutions}")

            """ ## Check for the Bottle Up
            nValidSolutions = 0
            for rollAngle in range(360):
                bottleFrame.moveTo(sdurw.Transform3Dd(bottleFrame.getTransform(state).P(), sdurw.RPYd(np.deg2rad(rollAngle),np.deg2rad(0),np.deg2rad(180))), state)

                # Finding solutions for the given angle
                solutions = []
                solutions.append(getConfigurations("Bottle", "GraspTCP", robotUR5, wc, state))

                # Check if the solution is actually a valid solution
                for k in range(len(solutions)):

                    for l in range(len(solutions[k])):
                        robotUR5.setQ(solutions[k][l], state)
                        res1 = sdurw.ProximityData()
                        if not detector.inCollision(state, res1):
                            nValidSolutions += 1
                            break
            print(f" - In bottle up, number of solutions: {nValidSolutions}") """
            
                
    print("--------------------------------------------------------------")
    print("Writing solution...")

    plt.imshow(bottleLateral, cmap="hot", interpolation='nearest')
    fig_bottleLateral = sns.heatmap(bottleLateral)
    fig_bottleLateral.set_title("Number of available poses gripping the bottle from the lateral")
    plt.savefig("bottleLateral.png", dpi=300)
    plt.clf()

    fig_objectiveLateral = sns.heatmap(objectiveLateral)
    fig_objectiveLateral.set_title("Number of available poses for the objective from the lateral")
    plt.savefig("objectiveLateral.png", dpi=300)
    plt.clf()





