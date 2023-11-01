import serial 
import time
import struct
from pynput.keyboard import Key, Listener

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

import threading

exitFlag = False
pause = False
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200) 

# Placeholders for data
xs = [] #Time data
ys = [] #Speed data
zs = [] #Setpoint data
xs.append(0)
ys.append(0)
zs.append(0)

def on_press(key):
    global exitFlag, pause
    print('{0} pressed'.format(key))
    if key == Key.space:
        pause ^= True
        return
    try:
        if key.char in ['8', '2', '4', '6', '0', 'x']:
            if key.char == 'x':
                write_data(key.char)
                exitFlag = True
                return False
            else:
                write_data(key.char)
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
def write_data(command): 
    arduino.write(bytes(command, 'utf-8')) 
    time.sleep(0.05) 


#read data from arduino
def read_float(): 
    data = arduino.read(4)
    f_data, = struct.unpack('<f',data)
    return f_data 


def runGraph():
    # Create figure for plotting
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    
    # This function is called periodically from FuncAnimation
    def animate(i, ani, xs, ys):
        if exitFlag == True:
            if plt.fignum_exists(fig.number):
                plt.close(fig)
            return
        if pause == True:
            return
        # Draw x and y lists
        ax.clear()
        ax.plot(xs[-20000:], ys[-20000:], label="Speed data")
        ax.plot(xs[-20000:], zs[-20000:], label="Setpoint data")
        #ax.plot(xs, ys, label="Speed data")
        #ax.plot(xs, zs, label="Setpoint data")
        plt.ylim([-1.1,1.1])
        ax.autoscale(enable=True, axis='x')

        # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.subplots_adjust(bottom=0.30)
        plt.title('Motor Speed and Setpoint vs Time Plot')
        plt.ylabel('Rotaional Velocity (in rps)')
        plt.xlabel('Time (in seconds)')
        plt.legend()
        #plt.axis([1, None, -1.1, 1.1]) #Use for arbitrary number of trials
        #plt.axis([1, 100, 0, 1.1]) #Use for 100 trial demo

    ani = lambda: None
    ani = animation.FuncAnimation(fig, animate, fargs=(ani, xs, ys), interval=1000, repeat=False)
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
    print("4 - Decrease speed \n")  
    print("6 - Increase speed \n")  
    print("0 - Stop Motor \n")  
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
                    speed = read_float()
                    #print("speed : {0:2.5f}".format(speed),end='\t') # printing the value 
                    setpoint = read_float()
                    #print("setpoint : {0:2.5f}".format(speed)) # printing the value 
                    xs.append(time)
                    ys.append(speed)
                    zs.append(setpoint)
            except UnicodeError:
                continue
                print("Discarding byte. Parse Error")
            
    
    listener.join()

if __name__ == "__main__":
    x = threading.Thread(target=runGraph)
    x.start()
    main()
    x.join()
