from re import S

import image_processor
import camera
import cv2
import time
from enum import Enum

from motion import SerialOmniMotion
from multiprocessing import Process,Pipe
from motion import SerialOmniMotion
import websockets
import asyncio
import json
from enums import *

class Side(Enum):
    BLUE = 1
    MAGENTA = 2

class Machine_state(Enum):
    FIND_BALL = 1 # ---> ball found 
    MOVE_TO_BALL = 2 # ---> ball in optimal pos
    ORBIT_BALL = 3 # (right) ---> Orbit ball till aligned with Right Basket or ball not in optimal pos
    ORBIT_BALL_LEFT = 4 # kui on korvi näha vasakul
    ORBIT_BALL_RIGHT = 5 # kui on korvi näha paremal
    THROW_BALL = 6 # ---> run ball throwing
    # let it be for now
    

    #p.join()

def main_loop(side):
    debug = False
    # 0 - magenta 1 -blue
    print("[RUNNING]")
    # default first thing find ball
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()
    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    STATE = Machine_state.FIND_BALL
    if(side is Side.MAGENTA):
        basket = processedData.basket_m 
    if(side is Side.BLUE):
        basket = processedData.basket_b 
    
    child, parent = Pipe()
    p = Process(target = refclient,args =(child))    
    p.start()
    try:
        while True:
            if(not p.is_alive):
                if parent.recv()[0]:
                    print("Signal still True")
                    pp = Process(target=refclient, args=(child,))
                    pp.start()
                else:
                    break
            # if signal == start (continue)
            # else break
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            if STATE is Machine_state.THROW_BALL:
                processedData = processor.process_frame(aligned_depth=True)
            else:
                processedData = processor.process_frame(aligned_depth=False)
            # praegu hard code 
            # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects

            frame_cnt +=1

            frame += 1
            if frame % 15 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                #print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                if len(processedData.balls) > 0:
                    # on pall
                    ball = processedData.balls[-1]
                    if STATE is Machine_state.FIND_BALL or STATE is Machine_state.MOVE_TO_BALL:
                        STATE = Machine_state.MOVE_TO_BALL
                    if (ball.x > 410 and ball.x < 450 and ball.y < 400 and ball.y > 340) or (STATE is Machine_state.ORBIT_BALL \
                            or STATE is Machine_state.ORBIT_BALL_LEFT or STATE is Machine_state.ORBIT_BALL_RIGHT):
                        print(basket.exists,"basket")
                        if basket.exists == 0:
                            print()
                            STATE = Machine_state.ORBIT_BALL
                        # orbit ball
                        if basket.exists: 
                            if basket.x > 450 :    # orb
                                STATE = Machine_state.ORBIT_BALL_LEFT
                            elif basket.x < 410 :
                                STATE = Machine_state.ORBIT_BALL_RIGHT
                            elif basket.x < 460 and basket.x > 410: # elif STATE is Machine_state.MOVE_TO_BALL: 
                                STATE = Machine_state.THROW_BALL
                    print(ball.x ,STATE, "FUCK")
                    if STATE is Machine_state.THROW_BALL and ((ball.x < 415 or ball.x > 445) and ball.y < 400 ):
                        STATE = Machine_state.FIND_BALL
                else:
                    # otsi palli
                    STATE = Machine_state.FIND_BALL
                print(processedData.basket_b.x ,"BASKET")
                move = SerialOmniMotion() # opens itself
                #Hakkame keerama ennast, et otsida palle.
                
                if STATE is not Machine_state.THROW_BALL:
                    sped = -1
                
                
                if STATE is Machine_state.FIND_BALL :
                    print("STATE",STATE)
                    move.spin(1.7)
                #Kui me leiame palli, mis on lähim, siis võrdleme neid oma seatud parameetritega ja liigume selle poole
                elif STATE is Machine_state.MOVE_TO_BALL:
                    print("State",STATE)
                    move.move_ing(ball.x,ball.y)
                #Orbitib palli, kui on leidnud ülesse, milline on talle kõige lähemal on ja hakkame korvi otsima
                elif STATE is Machine_state.ORBIT_BALL:
                    print('State',STATE)
                    move.orbit(-1)
                # Kui korv on meist paremale poole orbitime tollele poole
                elif STATE is Machine_state.ORBIT_BALL_LEFT:
                    print('State',STATE)
                    move.orbit(-0.4)
                #Kui korv on meist vasakule orbitime tollele poole
                elif STATE is Machine_state.ORBIT_BALL_RIGHT:
                    print('State',STATE)
                    move.orbit(0.4)
                #Kui oleme ennast sättinud heasse positsiooni korvi suhtes, siis viskame
                elif STATE is Machine_state.THROW_BALL:
                    print('State',STATE)
                    print(basket.distance," distance")
                    move.thro_shit(basket.distance)
                    
                    #move.send_to_robot(0,1,0,sped)
                            
                move.close()
            if debug:
                debug_frame = processedData.debug_frame
                cv2.imshow('debug', debug_frame)
                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    
                    break
    except KeyboardInterrupt:
        print("losing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()
    




async def refcommClient(signal):
    robotName = 'Valdis'
    connectTo = 'ws://192.168.3.82:8220/'
    async with websockets.connect(connectTo, ping_interval=None) as websocket:
        refcomm = await websocket.recv()
        print('Signal recieved...')
        refdict = json.loads(refcomm)
        print(refdict['targets'])
        if refdict['signal'] == 'start':
            print(refdict['baskets'])
            if robotName in refdict['targets']:
                if refdict['signal'] == 'start':
                    if refdict['baskets'][refdict['targets'].index(robotName)] == 'blue':
                        signal.send([True,Side.BLUE])
                        signal.end
                        return
                    elif refdict['baskets'][refdict['targets'].index(robotName)] == 'magenta':
                        signal.send([True,Side.MAGENTA])
                        signal.end
                        return
                    print('Starting competition!')
                
            elif refdict['signal'] == 'stop':
                print('Stopping competition!')
                signal.send([False,""])
                return
        else:
            print('Signal not directed at', robotName)
            
            
def refclient(signal):
    asyncio.run(refcommClient(signal))         
            
            
        
if __name__ == '__main__':
    child, parent = Pipe()
    p = Process(target = refclient, args=(child,))
    p.start()
    p.join()
    main_loop(parent.recv()[1])
    print("SLEEP")
            