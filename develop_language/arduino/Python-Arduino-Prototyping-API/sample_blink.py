#import the lib
import sys
sys.path.insert(0, '/home/magic/work/gitwork/work_note/dev_tools/arduino/Python-Arduino-Prototyping-API')

from arduino import Arduino

import time

#specify the port as an argument
# my_board = Arduino('/dev/ttyUSB0')
my_board = Arduino('/dev/ttyACM0')

#declare output pins as a list/tuple
my_board.output([10,11,12,13])

#perform operations
i=0
while(i<1):
    for j in range(50):
        print('range', j)
        my_board.setHigh(13)
        # my_board.analogWrite(10, j)
        # my_board.analogWrite(11, 255-j)
        time.sleep(0.5)
        my_board.setLow(13)
        time.sleep(0.5)
    i+=1
