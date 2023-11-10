import serial 
import time
import struct
from pynput.keyboard import Key, Listener

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import matplotlib.patches as mpatches

import threading

from enum import Enum 

class robotState(Enum):
    STOP = 0
    FORWARD = 8
    REVERSE = 2
    LEFT = 4
    RIGHT = 6
    CCW = 7
    CW  = 9
    INCR = 3
    DECR = 1
    CUSTOM = 5


robotCurrentstate = robotState.STOP

exitFlag = False
pause = False
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200) 

# Placeholders for data
xs = [] #Time data
ys1 = [] #Speed data
zs1 = [] #Setpoint data
ys2 = [] #Speed data
zs2 = [] #Setpoint data
ys3 = [] #Speed data
zs3 = [] #Setpoint data
ys4 = [] #Speed data
zs4 = [] #Setpoint data
xs.append(0)
ys1.append(0)
zs1.append(0)
ys2.append(0)
zs2.append(0)
ys3.append(0)
zs3.append(0)
ys4.append(0)
zs4.append(0)


def on_press(key):
    global exitFlag, pause
    print('{0} pressed'.format(key))
    if key == Key.space:
        pause ^= True
        return
    if key == Key.esc:
        write_data(0.0,0.0,0.0)
        exitFlag = True
        return False
    try:
        if key.char in ['8', '2', '4', '6', '1', '3', '7', '9', '5','0', 'x']:
            if key.char == 'x':
                u = 0.0
                v = 0.0
                r = 0.0
                write_data(u,v,r)
                exitFlag = True
                return False
            elif key.char== '8' :
                u = 300  # Strictly not more than 300, beyond that ISR will flood the Arduino code
                v = 0.0
                r = 0.0
                write_data(u,v,r)
            elif key.char== '2' :
                u = -100
                v = 0.0
                r = 0.0
                write_data(u,v,r)
            elif key.char== '4' :
                u = 0.0
                v = 20.0
                r = 0.0
                write_data(u,v,r)
            elif key.char== '6' :
                u = 0.0
                v = -20.0
                r = 0.0
                write_data(u,v,r)
            elif key.char== '1' :
                u = 0.0
                v = 0.0
                r = 0.0
                write_data(u,v,r)
            elif key.char== '3' :
                u = 0.0
                v = 0.0
                r = 0.0
                write_data(u,v,r)
            elif key.char== '7' :
                u = 0.0
                v = 0.0
                r = 0.3
                write_data(u,v,r)
            elif key.char== '9' :
                u = 0.0
                v = 0.0
                r = -0.3
                write_data(u,v,r)
            elif key.char== '5' :
                u = 0.0
                v = 0.0
                r = 0.0
                write_data(u,v,r)
            elif key.char== '0' :
                u = 0.0
                v = 0.0
                r = 0.0
                write_data(u,v,r)
        else:
            print("Invalid input provided : {0}".format(key.char))
    except AttributeError:
        print('Invalid input - special key {0} pressed'.format(key))

# Not used for now
def on_release(key):
    print('{0} release'.format(
        key))
    if key == Key.esc:
        # Stop listener
        return False



# write data to  arduino
def write_data(u, v, r):
    arduino.write(bytes('<', 'utf-8')) 
    arduino.write(struct.pack('<f',u)) 
    arduino.write(struct.pack('<f',v)) 
    arduino.write(struct.pack('<f',r)) 
    arduino.write(bytes('>', 'utf-8')) 
    #time.sleep(0.05) 


#read data from arduino
def read_float(): 
    data = arduino.read(4)
    f_data, = struct.unpack('<f',data)
    return f_data 


def runGraph():
    # Create figure for plotting
    fig = plt.figure()
    ax  = fig.add_subplot(1, 1, 1)
    # Turn off axis lines and ticks of the big subplot
    ax.spines['top'].set_color('none')
    ax.spines['bottom'].set_color('none')
    ax.spines['left'].set_color('none')
    ax.spines['right'].set_color('none')
    ax.tick_params(labelcolor='w', top=False, bottom=False, left=False, right=False)

    ax1 = fig.add_subplot(2, 2, 1)
    ax2 = fig.add_subplot(2, 2, 2)
    ax3 = fig.add_subplot(2, 2, 3)
    ax4 = fig.add_subplot(2, 2, 4)
    
    # This function is called periodically from FuncAnimation
    def animate(i, ani, xs, ys1):
        if exitFlag == True:
            if plt.fignum_exists(fig.number):
                plt.close(fig)
            return
        if pause == True:
            return
        # Draw x and y lists
        ax1.clear()
        ax2.clear()
        ax3.clear()
        ax4.clear()
        ax1.plot(xs[-2000:], ys1[-2000:], label="Speed data M1")
        ax1.plot(xs[-2000:], zs1[-2000:], label="Setpoint data M1")
        ax2.plot(xs[-2000:], ys2[-2000:], label="Speed data M2")
        ax2.plot(xs[-2000:], zs2[-2000:], label="Setpoint data M2")
        ax3.plot(xs[-2000:], ys3[-2000:], label="Speed data M3")
        ax3.plot(xs[-2000:], zs3[-2000:], label="Setpoint data M3")
        ax4.plot(xs[-2000:], ys4[-2000:], label="Speed data M4")
        ax4.plot(xs[-2000:], zs4[-2000:], label="Setpoint data M4")
        #ax.plot(xs, ys, label="Speed data")
        #ax.plot(xs, zs, label="Setpoint data")
        #plt.ylim([-1.1,1.1])
        #ax1.autoscale(enable=True, axis='x')
        ax1.autoscale(enable=True)
        ax2.autoscale(enable=True)
        ax3.autoscale(enable=True)
        ax4.autoscale(enable=True)

        # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.subplots_adjust(bottom=0.30)
        ax.set_title('Motor Speeds and Setpoint vs Time Plot')
        ax.set_ylabel('Rotaional Velocity (in rad per second)')
        ax.set_xlabel('Time (in seconds)')
        ax1.set_title('M1')
        ax2.set_title('M2')
        ax3.set_title('M3')
        ax4.set_title('M4')
        orange_patch = mpatches.Patch(color='orange', label='Setpoint data')
        blue_patch = mpatches.Patch(color='blue', label='Speed data')
        ax.legend(handles=[orange_patch, blue_patch])
        #ax.legend()
        #plt.axis([1, None, -1.1, 1.1]) #Use for arbitrary number of trials
        #plt.axis([1, 100, 0, 1.1]) #Use for 100 trial demo

    ani = lambda: None
    ani = animation.FuncAnimation(fig, animate, fargs=(ani, xs, ys1), interval=1000, repeat=False)
    plt.show()
    print("Quitting the plot --------------  \n")  

def main():
    
    # Collect events until released
    listener = Listener(on_press=on_press)
    listener.start()

    # ----- User Manual for motor control -----------------
    print("\n\n")
    print("Press following keys to control the motor: \n")  
    print("8 - Move forward \n") 
    print("2 - Move reverse \n") 
    print("4 - Move Left \n") 
    print("6 - Move Right \n") 
    print("4 - Rotate Counter clockwise \n") 
    print("6 - Rotate Clockwise \n") 
    print("1 - Decrease speed \n")  
    print("3 - Increase speed \n")  
    print("0 - Stop Motor \n")  
    print("1 - Custom speed in the format <u,v,w> \n")  
    print("x - Exit Program \n")  
    print("\n\n")
    # ------------------ User Manual end ------------------
    

    while not exitFlag:
        if arduino.in_waiting:
            data =  arduino.read(1)
            try:
                if data.decode() != '*':
                    print(data.decode(),end='')
                    pass
                else :
                    time = read_float()
                    #print("time : {0:2.5f} ".format(time),end='\t') # printing the value 
                    speed1 = read_float()
                    setpoint1 = read_float()
                    speed2 = read_float()
                    setpoint2 = read_float()
                    speed3 = read_float()
                    setpoint3 = read_float()
                    speed4 = read_float()
                    setpoint4 = read_float()
                    #print("speed : {0:2.5f}".format(speed),end='\t') # printing the value 
                    #print("setpoint : {0:2.5f}".format(speed)) # printing the value 
                    xs.append(time)
                    ys1.append(speed1)
                    zs1.append(setpoint1)
                    ys2.append(speed2)
                    zs2.append(setpoint2)
                    ys3.append(speed3)
                    zs3.append(setpoint3)
                    ys4.append(speed4)
                    zs4.append(setpoint4)
            except UnicodeError:
                continue
                print("Discarding byte. Parse Error")
            
    
    listener.join()

if __name__ == "__main__":
    x = threading.Thread(target=runGraph)
    x.start()
    main()
    x.join()
