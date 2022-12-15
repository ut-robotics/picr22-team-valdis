import image_processor
import camera
import cv2
import time
from enum import Enum
from motion import SerialOmniMotion

from multiprocessing import Process,Pipe
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
    throw_time = -1
    begin_boolean = True
    state = Machine_state.FIND_BALL
    
    CAMERA_WIDTH = cam.rgb_width
    CAMERA_HEIGHT = cam.rgb_height
    child, parent = Pipe()
    move = SerialOmniMotion(CAMERA_WIDTH,CAMERA_HEIGHT)
    ref_status = True
    pp = Process(target = refclient,args =(child,))    
    pp.start()
    try:
        while True:
            if not pp.is_alive():
                if parent.recv()[0]:  # --> kas siganl on true
                    ref_status = True
                    print("Signal True")
                    child, parent = Pipe()
                    pp = Process(target=refclient, args=(child,))
                    pp.start()
                else:
                    ref_status = False
                    print("Signal False")
                    child, parent = Pipe()
                    pp = Process(target=refclient, args=(child,))
                    pp.start()
                    
            if(ref_status):
                if state is Machine_state.THROW_BALL:
                    processedData = processor.process_frame(aligned_depth=True)
                else:
                    processedData = processor.process_frame(aligned_depth=False)
                    
                if(side[1] is Side.MAGENTA):
                    basket = processedData.basket_m 
                if(side[1] is Side.BLUE):
                    basket = processedData.basket_b 
                # praegu hard code 
                # This is where you add the driving behaviour of your robot. It should be able to filter out
                # objects of interest and calculate the required motion for reaching the objects

                if state is not Machine_state.THROW_BALL:
                    throw_time = -1

                if state is Machine_state.FIND_BALL and not begin_boolean:
                    
                    move.spin(-1.7)
                # Kui me leiame palli, mis on lähim, siis võrdleme neid oma seatud parameetritega ja liigume selle poole
                elif state is Machine_state.MOVE_TO_BALL:
                    begin_boolean = False
                    move.move_robot(ball.x, ball.y)
                    
                # Orbitib palli, kui on leidnud ülesse, milline on talle kõige lähemal on ja hakkame korvi otsima
                elif state is Machine_state.ORBIT_BALL:
                    
                    move.orbit(-1)
                # Kui korv on meist paremale poole orbitime tollele poole
                elif state is Machine_state.ORBIT_BALL_LEFT:
                    
                    move.orbit(-0.5)
                # Kui korv on meist vasakule orbitime tollele poole
                elif state is Machine_state.ORBIT_BALL_RIGHT:
                    
                    move.orbit(0.5)
                # Kui oleme ennast sättinud heasse positsiooni korvi suhtes, siis viskame
                elif state is Machine_state.THROW_BALL:
                    print(ball.y, " Y")
                    print(ball.x, " X")
                    print('State', state)
                    print(basket.distance, " distance")
                    print(basket.x)
                    move.throw_ball(basket.distance)

                    # move.send_to_robot(0,1,0,sped)
                
                if state is Machine_state.THROW_BALL and throw_time == -1:
                    print("STARTING THROWING MODE!")
                    throw_time = time.time()

                if state is Machine_state.THROW_BALL and throw_time != -1 and throw_time + 1 > time.time():
                    print("STAYING IN THROW", throw_time + 2, time.time())
                    state = Machine_state.THROW_BALL

                elif len(processedData.balls) > 0:
                    # on pall
                    ball = processedData.balls[-1]
                    if state is Machine_state.FIND_BALL or state is Machine_state.MOVE_TO_BALL or state is Machine_state.THROW_BALL:
                        state = Machine_state.MOVE_TO_BALL
                    # kui pall õiges kohas 
                    # ToDO make those values depend on a CAMERA_WIDTH and CAMERA_HEIGHT
                    # kas see oleks parem ? -> 
                    # if (ball.x > CAMERA_WIDTH/2 - 9 and ball.x < CAMERA_WIDTH/2 +21  and ball.y < CAMERA_HEIGHT - 40 and ball.y > CAMERA_HEIGHT-70)
                    if (ball.x > 0.489 * CAMERA_WIDTH and ball.x < 0.524 * CAMERA_WIDTH  and ball.y < 0.91 * CAMERA_HEIGHT and ball.y > 0.854 * CAMERA_HEIGHT) or (
                            state is Machine_state.ORBIT_BALL or state is Machine_state.ORBIT_BALL_LEFT or
                            state is Machine_state.ORBIT_BALL_RIGHT):
                        # print(basket.exists,"basket")
                        if basket.exists == 0:
                            state = Machine_state.ORBIT_BALL
                        # orbit ball
                        # ToDO make those values depend on a CAMERA_WIDTH and CAMERA_HEIGHT
                        if basket.exists:
                            if basket.x > 0.5 * CAMERA_WIDTH: 
                                state = Machine_state.ORBIT_BALL_LEFT
                            elif basket.x < 0.483 * CAMERA_WIDTH:
                                state = Machine_state.ORBIT_BALL_RIGHT
                            elif basket.x < 0.5 * CAMERA_WIDTH and basket.x >  0.483 * CAMERA_WIDTH:
                                state = Machine_state.THROW_BALL
                                move.send_to_robot(0,0,0,0)
                                if not (ball.x > 0.489 * CAMERA_WIDTH and ball.x < 0.524 * CAMERA_WIDTH  and ball.y < 0.91 * CAMERA_HEIGHT and ball.y > 0.854 * CAMERA_HEIGHT):
                                    state = Machine_state.MOVE_TO_BALL

                elif begin_boolean:
                    move.spin(1.2)
                    if basket.exists and (basket.x < 0.7 * CAMERA_WIDTH or basket.x > 0.413 * CAMERA_WIDTH ):
                        move.send_to_robot(0,1,0,0)
                else:
                    # otsi palli
                    state = Machine_state.FIND_BALL
            else:
                print("wait")
           
    except KeyboardInterrupt:
        print("losing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()
    




async def refcommClient(signal):
    robotName = 'Valdis'
    connectTo = 'ws://172.17.153.255:8222'
    print("going to connect with ws")
    async with websockets.connect(connectTo, ping_interval=None) as websocket:
        print("going to while loop")
        while True:
            refcomm = await websocket.recv()
            print('Signal recieved...')
            refdict = json.loads(refcomm)
            if robotName in refdict['targets']:
                if refdict['signal'] == 'start':
                    if refdict['baskets'][refdict['targets'].index(robotName)] == 'blue':
                        # attackingSide.value = Side.blue
                        signal.send([True, Side.BLUE])
                        signal.close()
                        break
                        

                    elif refdict['baskets'][refdict['targets'].index(robotName)] == 'magenta':
                        # attackingSide.value = Side.pink
                        signal.send([True, Side.MAGENTA])
                        signal.close()
                        break
                        
                    # robotState.value = State.automatic
                    print('Starting competition!')

                elif refdict['signal'] == 'stop':
                    # robotState.value = State.remote
                    signal.send([False, " "])
                    signal.close()
                    break

            else:
                print('Signal not directed at', robotName)
            
            
def refclient(signal):
    asyncio.run(refcommClient(signal))         
            
            
        
if __name__ == '__main__':
    child, parent = Pipe()
    p = Process(target = refclient, args=(child,))
    p.start()
    print("Procces started")
    p.join()
    main_loop(parent.recv())
    print("SLEEP")
            