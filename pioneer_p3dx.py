# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import os
import sys
import time
import numpy as np
import sim


class Pioneer_p3dx:

    def __init__(self, clientID):
        self.clientID = clientID
        self.N_ULT_SENSOR = 16
        self.v0 = 0.0

        self.u_sensors = []
        for i in range(self.N_ULT_SENSOR):
            _, oh = sim.simxGetObjectHandle(
                clientID,
                "Pioneer_p3dx_ultrasonicSensor" + str(i + 1),
                sim.simx_opmode_blocking,
            )
            self.u_sensors.append(oh)

        _, self.motorLeft = sim.simxGetObjectHandle(
            clientID, "Pioneer_p3dx_leftMotor", sim.simx_opmode_blocking
        )
        _, self.motorRight = sim.simxGetObjectHandle(
            clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_blocking
        )

    def actuation_sample(self, duration=60*5):
        """
        This is a very simple EXAMPLE navigation program, which avoids obstacles using the Braitenberg algorithm
        """
        self.v0 = 2.0
        noDetectionDist = 0.5
        maxDetectionDist = 0.2
        detect = np.zeros(self.N_ULT_SENSOR)
        braitenbergL = [-0.2, -0.4, -0.6, -0.8, -1.0, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        braitenbergR = [-1.6, -1.4, -1.2, -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Start actuation loop 
        startTime = time.time()
        while time.time() - startTime < duration:
            for i in range(self.N_ULT_SENSOR):
                # Sensing information from the proximity sensor
                (
                    returnCode,
                    detectionState,
                    detectedPoint,
                    detectedObjectHandle,
                    detectedSurfaceNormalVector,
                ) = sim.simxReadProximitySensor(
                    self.clientID, self.u_sensors[i], sim.simx_opmode_blocking
                    # self.clientID, self.u_sensors[i], sim.simx_opmode_streaming
                )
                dist = np.linalg.norm(detectedPoint)
                if detectionState and dist < noDetectionDist:
                    if dist < maxDetectionDist:
                        dist = maxDetectionDist
                        detect[i] = 1 - (
                            (dist - maxDetectionDist)
                            / (noDetectionDist - maxDetectionDist)
                        )
                else:
                    detect[i] = 0

            vLeft = self.v0
            vRight = self.v0

            for i in range(self.N_ULT_SENSOR):
                vLeft = vLeft + braitenbergL[i] * detect[i]
                vRight = vRight + braitenbergR[i] * detect[i]

            _ = sim.simxSetJointTargetVelocity(
                # self.clientID, self.motorLeft, vLeft, sim.simx_opmode_blocking
                self.clientID, self.motorLeft, vLeft, sim.simx_opmode_oneshot
            )
            _ = sim.simxSetJointTargetVelocity(
                # self.clientID, self.motorRight, vRight, sim.simx_opmode_blocking
                self.clientID, self.motorRight, vRight, sim.simx_opmode_oneshot
            )


def main():
    print("Program started")
    sim.simxFinish(-1)  # just in case, close all opened connections
    clientID = sim.simxStart(
        "127.0.0.1", 19999, True, True, 5000, 5
    )  # Connect to CoppeliaSim

    if clientID != -1:
        print("Connected to remote API server")
        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        res, objs = sim.simxGetObjects(
            clientID, sim.sim_handle_all, sim.simx_opmode_blocking
        )

        if res == sim.simx_return_ok:
            print("Number of objects in the scene: ", len(objs))
            print("Client ID: ", clientID)
        else:
            print("Remote API function call returned with error code: ", res)
            sys.exit("Could not connect")

        pioneerRob = Pioneer_p3dx(clientID)
        pioneerRob.actuation_sample()

        """
        err_code, l_motor_handle = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor",  sim.simx_opmode_blocking)
        err_code, r_motor_handle = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_blocking)

        err_code = sim.simxSetJointTargetVelocity(clientID, l_motor_handle,  1.0, sim.simx_opmode_streaming)
        err_code = sim.simxSetJointTargetVelocity(clientID, r_motor_handle,  1.0, sim.simx_opmode_streaming)
        time.sleep(5)

        err_code = sim.simxSetJointTargetVelocity(clientID, l_motor_handle, -1.0, sim.simx_opmode_streaming)
        err_code = sim.simxSetJointTargetVelocity(clientID, r_motor_handle,  1.0, sim.simx_opmode_streaming)
        time.sleep(2)

        err_code = sim.simxSetJointTargetVelocity(clientID, l_motor_handle,  0.0, sim.simx_opmode_streaming)
        err_code = sim.simxSetJointTargetVelocity(clientID, r_motor_handle,  0.0, sim.simx_opmode_streaming)
        time.sleep(2)

        err_code = sim.simxSetJointTargetVelocity(clientID, l_motor_handle,  1.0, sim.simx_opmode_streaming)
        err_code = sim.simxSetJointTargetVelocity(clientID, r_motor_handle,  1.0, sim.simx_opmode_streaming)
        time.sleep(2)
        """

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    else:
        print("Failed connecting to remote API server")
    print("Program ended")


if __name__ == "__main__":
    main()
