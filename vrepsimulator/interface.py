__author__ = 'will'
import vrep
import numpy as np
import cv2
import time
from math import sin, cos, pi

class RobotInterface():
    """
    Esta classe facilita a interface com o simulador
    """

    def __init__(self):
        vrep.simxFinish(-1)  # just in case, close all opened connections
        time.sleep(0.5)
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5) # tenta conectar no simulador, se salva o clientID

        # paramos a simulacao, se estiver em andamento, e comecamos denovo
       # vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot)
        #time.sleep(0.5)
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot)

        # modo da API, como eh False, esta no modo assincrono(os ticks da simulacao rodam em velocidade independente)
        vrep.simxSynchronous(self.clientID, False)
        print("connected with id ", self.clientID)

        self.oldtarget = None
        self.camera = None

        self.setup()
        self.lastimageAcquisitionTime = 0

    def finish_iteration(self):
        vrep.simxSynchronousTrigger(self.clientID)


    def _read_camera(self):
        data = vrep.simxGetVisionSensorImage(self.clientID,self.camera,1,vrep.simx_opmode_buffer)
        if data[0] == vrep.simx_return_ok :
            return data
        return None

    def get_image_from_camera(self):
        """
        Loads image from camera.
        :return:
        """
        img = None
        while not img:  img = self._read_camera()


        size = img[1][0]
        img = cv2.flip(np.array(img[2], dtype='uint8').reshape((size, size)),0)

        return img


    def get_target(self):
        time.sleep(0.1)
        ret, xyz =vrep.simxGetObjectPosition(self.clientID, self.target, -1,vrep.simx_opmode_oneshot)
        print(ret, xyz)
        x,y,z = xyz
        ret, orientation = vrep.simxGetObjectOrientation(self.clientID, self.target,-1, vrep.simx_opmode_oneshot)
        rot = orientation[2]

        return (x, y, z, rot)

    def set_target(self,x,y,z,rot):
        vrep.simxSetObjectPosition(self.clientID,self.target,-1,(x,y,z), vrep.simx_opmode_oneshot)
        vrep.simxSetObjectOrientation(self.clientID,self.target,-1,(0,0,rot), vrep.simx_opmode_oneshot)
        self.oldtarget = (x,y,z,rot)

    def move(self,dx,dy,dz,drot):
        if not self.oldtarget:
            self.oldtarget = self.get_target()


        oldx, oldy, oldz, rot = self.oldtarget

        newx = oldx + cos(rot) * dx + sin(rot) * dy
        newy = oldy + sin(rot)*dx + cos(rot)*dy
        new_target = [newx,newy,self.oldtarget[2]+dz,self.oldtarget[3]+drot]

        self.oldtarget = new_target
        self.set_target(*new_target)


    def stop(self):
        vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)

    def setup(self):
        if self.clientID != -1:
            errorCode, handles, intData, floatData, array = vrep.simxGetObjectGroupData(self.clientID,
                                                                                        vrep.sim_appobj_object_type,
                                                                                        0,
                                                                                        vrep.simx_opmode_oneshot_wait)
            data = dict(zip(array, handles))
            self.camera = [value for key, value in data.items() if "Vision_sensor" in key][0]
            self.target = [value for key, value in data.items() if "Quadricopter_target" == key][0]
            vrep.simxGetVisionSensorImage(self.clientID, self.camera, 1, vrep.simx_opmode_streaming)
