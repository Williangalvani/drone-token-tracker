from interface import RobotInterface
import time
import cv2
from vision.token_locator import QrFinder


class PidController():
    def __init__(self,P=0.26 ,I=0.001   ,D=0.0):
        self.P = P
        self.I = I
        self.D = D
        self.cumulatedError = 0
        self.lastError = 0
        self.lasttime = time.time()


    def update(self,currentValue,targetValue):
        newtime = time.time()
        dt = newtime - self.lasttime
        self.lasttime = newtime

        error = targetValue-currentValue
        derivative = self.D * (error-self.lastError)
        proportional = error * self.P
        self.cumulatedError+=error*dt
        #self.cumulatedError = min(max(-100,self.cumulatedError),100)
        integral = self.I *self.cumulatedError
        self.lastError = error

        #print "P:", proportional, " I:", integral, " D:", derivative
        return derivative + proportional + integral


class Controller:

    def __init__(self):
        self.interface = RobotInterface()
        self.qrfinder = QrFinder()
        self.interface.set_target(0,0,1,0)

        self.pidx = PidController(0.001,0,0)
        self.pidy = PidController(0.0003, 0.0001, 0.001)
        self.pidz = PidController(0.0001, 0, 0)

        while True:
            print("loop")
            time.sleep(0.05)

            ### esta funcao, do "tracker" faz o processamento da imagem
            img = self.interface.get_image_from_camera()
            self.qrfinder.find_code(img)
            ### esta funcao faz o controle do robo
            self.control()

            ## esta parte do codigo detecta se a barra de espaco foi pressionada, para parar a simulacao
            ch = cv2.waitKey(5) & 0xFF
            if ch == 27:
               break
            #print "loop done"

        self.interface.stop()
        #cv2.destroyAllWindows()

    def control(self):
        """
        Calcula o controle das rodas
        :return:
        """
        #print self.qrfinder.center, self.qrfinder.size

        targetx = 256
        targety = 256
        targetsize = 50

        dx = self.pidx.update(self.qrfinder.size[0],targetsize)
        dy = self.pidy.update(self.qrfinder.center[0],targetx)
        dz = self.pidz.update(self.qrfinder.center[1],targety)





        print(dx, dy, self.qrfinder.size)

        distancia = 1/(self.qrfinder.size[1]+0.00001)

        self.interface.move(dx,dy, dz, 0)



Controller()
