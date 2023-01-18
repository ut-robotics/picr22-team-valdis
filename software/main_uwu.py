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
    STATE = Machine_state.FIND_BALL
    if(side is Side.MAGENTA):
        basket = processedData.basket_m 
    if(side is Side.BLUE):
        basket = processedData.basket_b 
    
    child, parent = Pipe()
    
    ref_status = True
    pp = Process(target = refclient,args =(child,))    
    pp.start()
    try:
        while True:
            processedData = processor.process_frame(aligned_depth=False)
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
                print("WORM")
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
                        signal.send([True, 'Blue'])
                        signal.close()
                        break
                        

                    elif refdict['baskets'][refdict['targets'].index(robotName)] == 'magenta':
                        # attackingSide.value = Side.pink
                        signal.value([True, 'Magenta'])
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
    main_loop(parent.recv()[1])
    print("SLEEP")
            