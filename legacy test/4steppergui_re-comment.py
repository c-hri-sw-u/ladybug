#!/usr/bin/env python3  # Shebang line specifying this should be run with Python 3 interpreter

# Import all required libraries with detailed purpose
from numpy import * # Imports all numpy functions for array operations and scan parameter calculations
import random      # Used for generating random numbers in repeatability testing
import time        # Provides timing functions for delays and movement control
import math        # Provides mathematical functions for calculations
import os          # Enables file and directory operations for saving scan data
import tkinter as tk      # Main GUI framework for the application interface
from tkinter import font  # Provides font handling capabilities for GUI text
from tkinter import filedialog  # Enables file/directory selection dialogs
import RPi.GPIO as GPIO   # Interface for controlling Raspberry Pi GPIO pins
import subprocess # Enables running external commands, used for camera control
import sys        # Provides system-specific parameters and functions
import select     # Used for implementing timeouts and USB disconnect detection
import pickle     # Enables saving/loading of Python objects for scan data persistence

# Configure GPIO pin numbering mode
GPIO.setmode(GPIO.BOARD)  # Uses physical pin numbers rather than BCM numbering

# Global position tracking variables - maintain absolute position of each axis
GlobalX = 0  # Tracks X-axis position (camera/top motor) in steps from home
GlobalY = 0  # Tracks Y-axis position (sample/bottom motor) in steps from home
GlobalZ = 0  # Tracks Z-axis position (vertical movement) in steps from home
GlobalR = 0  # Tracks rotation position in steps from starting position

# Initialize scan boundary variables - these define the scanning volume
XScanMin = 0  # Minimum X position for scan range
XScanMax = 0  # Maximum X position for scan range
YScanMin = 0  # Minimum Y position for scan range
YScanMax = 0  # Maximum Y position for scan range
ZScanMin = 0  # Minimum Z position for scan range
ZScanMax = 0  # Maximum Z position for scan range
RScanMin = 0  # Minimum rotation position
RScanMax = 160  # Maximum rotation (160 steps = full rotation with current setup)

# Define step sizes for each axis during scanning
XScanStep = 150  # Distance between scan points in X direction (in steps)
YScanStep = 150  # Distance between scan points in Y direction (in steps)
ZScanStep = 1    # Distance between scan points in Z direction (in steps)
RScanNumber = 1  # Number of rotational positions to scan (1 = no rotation)

# Define valid rotation step options for GUI dropdown
FactorsOf160 = [1,2,4,5,8,10,16,20,32,40,80,160]  # Factors of 160 for evenly spaced rotations

## GPIO pin assignments
# GPIO pin assignments for motor direction control
YDIR = 26  # Direction control pin for Y-axis motor (sample/bottom)
XDIR = 18  # Direction control pin for X-axis motor (camera/top)
ZDIR = 40  # Direction control pin for Z-axis motor
RDIR = 21  # Direction control pin for rotation motor

# GPIO pin assignments for motor stepping
YSTEP = 24  # Step control pin for Y-axis motor
XSTEP = 16  # Step control pin for X-axis motor
ZSTEP = 38  # Step control pin for Z-axis motor
RSTEP = 19  # Step control pin for rotation motor

# GPIO pin for status indicator
BEEP = 35  # Pin for audible/visual feedback of system status

# GPIO pins for limit switches and position sensing
XLimit = 11  # Input pin for X-axis mechanical limit switch
YLimit = 13  # Input pin for Y-axis mechanical limit switch
ZLimit = 15  # Input pin for Z-axis optical limit switch (different from mechanical)


# -------------------------


# Maximum range limits for each axis in 8th microsteps
XMax = 1800  # Maximum allowed steps in X direction, limited by physical carriage size
YMax = 2000  # Maximum allowed steps in Y direction
ZMax = 3000  # Maximum allowed steps in Z direction
StepsPerRotation = 160  # Total steps for one full rotation using 8th microstepping (20 steps * 8)

# Direction constants for X axis
XFORWARD = 1   # Defines forward direction (away from home position) as 1
XBACKWARD = 0  # Defines backward direction (towards home position) as 0

# Direction constants for Y axis
YFORWARD = 1   # Defines forward direction as 1
YBACKWARD = 0  # Defines backward direction as 0

# Direction constants for Z axis
ZFORWARD = 1   # Defines forward (upward) direction as 1
ZBACKWARD = 0  # Defines backward (downward) direction as 0

# Direction constants for rotation
RFORWARD = 1   # Defines clockwise rotation (when looking at sample) as 1
RBACKWARD = 0  # Defines counterclockwise rotation as 0

# Movement speed presets (delay between steps in seconds)
FASTERER = 0.0003  # Ultra fast movement speed
FASTER = 0.0006    # Very fast movement speed
FAST = 0.002       # Fast movement speed
SLOW = 0.007       # Slow movement speed
SLOWER = 0.03      # Very slow movement speed
SLOWERER = 0.06    # Ultra slow movement speed

# Configure GPIO output pins
GPIO.setup(BEEP, GPIO.OUT)    # Set up beeper pin as output
GPIO.setup(XDIR, GPIO.OUT)    # Set up X direction pin as output
GPIO.setup(YDIR, GPIO.OUT)    # Set up Y direction pin as output
GPIO.setup(ZDIR, GPIO.OUT)    # Set up Z direction pin as output
GPIO.setup(RDIR, GPIO.OUT)    # Set up rotation direction pin as output
GPIO.setup(XSTEP, GPIO.OUT)   # Set up X stepping pin as output
GPIO.setup(YSTEP, GPIO.OUT)   # Set up Y stepping pin as output
GPIO.setup(ZSTEP, GPIO.OUT)   # Set up Z stepping pin as output
GPIO.setup(RSTEP, GPIO.OUT)   # Set up rotation stepping pin as output

# Configure GPIO input pins for limit switches
# Pull-up resistors are enabled for mechanical switches (X and Y)
GPIO.setup(YLimit, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Y limit switch with pull-up
GPIO.setup(XLimit, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # X limit switch with pull-up
GPIO.setup(ZLimit, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Z optical switch (no pull-up needed)

# Initialize main GUI window
win = tk.Tk()  # Create the main window object
myFont = tk.font.Font(family='Helvetica', size=12, weight='bold')      # Define standard bold font
myBigFont = tk.font.Font(family='Helvetica', size=20,weight='bold')    # Define larger bold font
font.families()  # Get available font families (not used but available for reference)


# ------------------------- Begin defining scanner guts


def beep(duration = 0.2, repeat = 1):
    """Basic feedback function for user notification
    Args:
        duration (float): Length of each beep in seconds
        repeat (int): Number of times to repeat the beep"""
    
    for i in range (repeat):
        GPIO.output(BEEP,GPIO.HIGH)      # Turn beeper on
        time.sleep(duration/2)           # Wait for half the duration
        GPIO.output(BEEP,GPIO.LOW)       # Turn beeper off
        time.sleep(duration/2)           # Wait for remaining half of duration

def ExampleScan():
    """Demonstration function showing basic scan capability
    Note: Comment indicates rotation behavior may be irregular"""
    ScanCoord = DefineScan(800,1000,1000,1200)  # Define scan area with fixed coordinates
    GridScan(ScanCoord)                         # Execute the scan with defined coordinates

def DemoMove(speed=1, delay=1):
    """Demonstration function showing system movement capabilities
    Args:
        speed (float): Movement speed multiplier
        delay (float): Delay between movement sequences in seconds"""
    
    # First movement sequence - forward movements
    for i in range(700):
        MoveX(XFORWARD,2,FASTERER/speed)    # Move X axis forward
        MoveY(YFORWARD,2,FASTERER/speed)    # Move Y axis forward
        MoveZ(ZFORWARD,4,FASTERER/speed)    # Move Z axis forward
        if i%5 == 0:                        # Every 5th iteration
            MoveR(RFORWARD,2,FASTERER/speed)  # Rotate forward

    time.sleep(delay/2)    # Pause between sequences
    
    # Second movement sequence - backward movements
    for i in range(700):
        MoveX(XBACKWARD,2,FASTERER/speed)   # Move X axis backward
        MoveY(YBACKWARD,2,FASTERER/speed)   # Move Y axis backward
        MoveZ(ZBACKWARD,4,FASTERER/speed)   # Move Z axis backward
        if i%5 == 0:                        # Every 5th iteration
            MoveR(RBACKWARD,2,FASTERER/speed) # Rotate backward
            
    time.sleep(delay)      # Full pause between sequences
    
    # X axis demonstration
    MoveX(XFORWARD,1500,FASTERER/speed)     # Large forward movement
    time.sleep(0.1)                         # Short pause
    MoveX(XBACKWARD, 1500, FASTERER/speed)  # Return movement
    time.sleep(0.1)                         # Short pause
    MoveX(XFORWARD,800,FASTERER/speed)      # Partial forward movement
    
    time.sleep(delay/2)    # Half pause
    
    # Y axis demonstration
    MoveY(YFORWARD, 1500, FASTERER/speed)   # Large forward movement
    time.sleep(0.1)                         # Short pause
    MoveY(YBACKWARD, 1500, FASTERER/speed)  # Return movement
    time.sleep(0.1)                         # Short pause
    MoveY(YFORWARD, 1400, FASTERER/speed)   # Partial forward movement
    
    # Z axis demonstration - multiple cycles
    for i in range(2):
        MoveZ(ZFORWARD, 2800, FASTERER/2/speed)   # Move up at half speed
        MoveZ(ZBACKWARD, 2800, FASTERER/2/speed)  # Move down at half speed
        
    MoveZ(ZFORWARD, 2800, FASTER/speed)    # One more up movement at different speed
    MoveZ(ZBACKWARD, 2800, FASTER/speed)   # One more down movement
    
    # Rotation demonstration
    for i in range(1):
        MoveR(RFORWARD, 320, FAST/speed)    # Full rotation forward
        time.sleep(delay/2)                 # Half pause
        MoveR(RBACKWARD, 320, FAST/speed)   # Full rotation backward
    time.sleep(delay/2)    
    MoveR(RFORWARD, 800, FAST/speed)        # Multiple rotations forward
    
    time.sleep(delay)
    HomeX()        # Return X axis to home position
    HomeY()        # Return Y axis to home position
    beep(0.25,2)   # Signal completion with double beep

def restart():
    """Function to restart the Raspberry Pi
    Uses sudo command to initiate system restart"""
    command = "/usr/bin/sudo /sbin/shutdown -r now"
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output = process.communicate()[0]
    print(output)


# ------------

def MoveX(direction, numsteps, delay):
    """Controls movement of X axis (camera/top motor)
    Args:
        direction: XFORWARD (1) or XBACKWARD (0)
        numsteps: Number of steps to move
        delay: Time between steps in seconds"""
    
    # Set direction pin for X motor
    GPIO.output(XDIR, direction)
        
    # Execute requested number of steps
    for i in range(numsteps):
        GPIO.output(XSTEP, GPIO.HIGH)    # Trigger one step (rising edge)
        time.sleep(delay)                # Wait specified delay
        GPIO.output(XSTEP, GPIO.LOW)     # Complete step (falling edge)
    
    # Update global position tracking
    global GlobalX
    if direction == XFORWARD:
        GlobalX += numsteps              # Add steps if moving forward
    else:
        GlobalX -= numsteps              # Subtract steps if moving backward
    
    # Update position display in GUI
    XPosition.configure(text="X: "+str(GlobalX) + "/" + str(XMax)) 

def MoveY(direction, numsteps, delay):
    """Controls movement of Y axis (sample/bottom motor)
    Args:
        direction: YFORWARD (1) or YBACKWARD (0)
        numsteps: Number of steps to move
        delay: Time between steps in seconds"""
    
    # Set direction pin for Y motor
    GPIO.output(YDIR, direction)    
        
    # Execute requested number of steps
    for i in range(numsteps):
        GPIO.output(YSTEP, GPIO.HIGH)    # Trigger one step
        time.sleep(delay)                # Wait specified delay
        GPIO.output(YSTEP, GPIO.LOW)     # Complete step
    
    # Update global position tracking
    global GlobalY
    if direction == YFORWARD:  
        GlobalY += numsteps              # Add steps if moving forward
    else:
        GlobalY -= numsteps              # Subtract steps if moving backward
    
    # Update position display in GUI
    YPosition.configure(text="Y: "+str(GlobalY) + "/" +str(YMax))

def MoveZ(direction, numsteps, delay):
    """Controls movement of Z axis (vertical movement)
    Args:
        direction: ZFORWARD (1) or ZBACKWARD (0)
        numsteps: Number of steps to move
        delay: Time between steps in seconds
    Note: No sleep pin control - careful with current to avoid motor heating"""
    
    # Set direction pin for Z motor
    GPIO.output(ZDIR, direction)
    global GlobalZ
         
    # Execute requested number of steps
    for i in range(numsteps):
        GPIO.output(ZSTEP, GPIO.HIGH)    # Trigger one step
        time.sleep(delay)                # Wait specified delay
        GPIO.output(ZSTEP, GPIO.LOW)     # Complete step
    
    # Update global position tracking
    if direction == ZFORWARD: 
        GlobalZ += numsteps              # Add steps if moving forward
    else:
        GlobalZ -= numsteps              # Subtract steps if moving backward
    
    # Update position display in GUI
    ZPosition.configure(text="Z: "+str(GlobalZ) + "/" + str(ZMax))

def MoveR(direction, numsteps, delay):
    """Controls rotation movement
    Args:
        direction: RFORWARD (1, clockwise) or RBACKWARD (0, counterclockwise)
        numsteps: Number of steps to rotate
        delay: Time between steps in seconds
    Note: Position tracking is problematic due to lack of absolute zero reference"""
    
    # Set direction pin for rotation motor
    GPIO.output(RDIR, direction)
    global GlobalR
    
    # Execute requested number of steps
    for i in range(numsteps):
        GPIO.output(RSTEP, GPIO.HIGH)    # Trigger one step
        time.sleep(delay)                # Wait specified delay
        GPIO.output(RSTEP, GPIO.LOW)     # Complete step
    
    # Note: Position tracking is incomplete for rotation
    # Missing position update code that other axes have
    # This could cause issues with position tracking

    '''Count information like the others could be inserted here. 
    But we don't really have a way to define our "zero", except for wherever it is when we start the scan,
    which is problematic because what if we restart with our R in a different place? 
    If there are undiscovered problems, I think they would revolved around this area. FYI. '''


def XGoTo(XDest, XMin=0):
    """Moves X axis to specified absolute position
    Args:
        XDest: Desired destination position in steps
        XMin: Minimum allowed position (defaults to 0)
    Returns:
        Error message if input is invalid"""
  
    global GlobalX
    global XMax
    
    # Validate input is an integer
    if not isinstance(XDest, int):
        return ('Please input an integer for X destination')
        
    # Check if destination is within valid range
    if XDest <= XMax and XDest >= XMin:
        distance = XDest - GlobalX       # Calculate required movement
        if distance > 0:                 # Need to move forward
            MoveX(XFORWARD, distance, FASTER)
        else:                           # Need to move backward
            MoveX(XBACKWARD, abs(distance), FASTER) 
    else:
        print ('Destination out of range')
      
def YGoTo(YDest, YMin=0):
    """Moves Y axis to specified absolute position
    Args:
        YDest: Desired destination position in steps
        YMin: Minimum allowed position (defaults to 0)
    Returns:
        Error message if input is invalid"""
  
    global GlobalY
    global YMax
    
    # Validate input is an integer
    if not isinstance(YDest, int):
        return ('Please input an integer for Y destination')
        
    # Check if destination is within valid range
    if YDest <= YMax and YDest >= YMin:
        distance = YDest - GlobalY       # Calculate required movement
        if distance > 0:                 # Need to move forward
            MoveY(YFORWARD, distance, FASTER)
        else:                           # Need to move backward
            MoveY(YBACKWARD, abs(distance), FASTER) 
    else:
        print ('Destination out of range')

def ZGoTo(ZDest, ZMin=0):
    """Moves Z axis to specified absolute position
    Args:
        ZDest: Desired destination position in steps
        ZMin: Minimum allowed position (defaults to 0)
    Note: At home position, approximately 2000 steps forward and 1000 steps back
          available for 1 micron per step
    Returns:
        Error message if input is invalid"""
    
    global GlobalZ
    global ZMax
    
    # Validate input is an integer
    if not isinstance(ZDest, int):
        return ('Please input an integer for Z destination')
        
    # Check if destination is within valid range
    if ZDest <= ZMax and ZDest >= ZMin:
        numsteps = ZDest - GlobalZ      # Calculate required movement
        if numsteps > 0:                # Need to move forward
            MoveZ(ZFORWARD, numsteps, FASTERER)
        else:                          # Need to move backward
            MoveZ(ZBACKWARD, abs(numsteps), FASTERER) 
    else:
        print ('Destination out of range')

def RGoTo(RDest, RMin=0):
    """Moves rotation axis to specified absolute position
    Args:
        RDest: Desired destination position in steps
        RMin: Minimum allowed position (defaults to 0)
    Note: Added later, possible source of issues due to lack of absolute reference
    Returns:
        Error message if input is invalid"""
  
    global GlobalR
    global StepsPerRotation
    
    # Validate input is an integer
    if not isinstance(RDest, int):
        return ('Please input an integer for R destination')
        
    # Check if destination is within valid range
    if RDest <= StepsPerRotation and RDest >= RMin:
        distance = RDest - GlobalR      # Calculate required movement
        if distance > 0:                # Need to move forward/clockwise
            MoveR(RFORWARD, distance, FAST)
        else:                          # Need to move backward/counterclockwise
            MoveR(RBACKWARD, abs(distance), FAST) 
    else:
        print ('I understand the desire to watch the motor spin around a lot... but between {} and {} please'.format(RMin, StepsPerRotation))
              

def CheckPress(PIN):
    '''checks whether specified GPIO pin has been pressed, since home procedures have to call multiple times'''
    input_state = GPIO.input(PIN)
    if input_state == False: #button press
            time.sleep(0.05) #debounce
            input_state = GPIO.input(PIN)
            if input_state == False: #still!
                return True #yep, button press
            
def HomeX():
    global GlobalX
    for i in range(XMax + 200): #some number that's noticably larger than the range, but also will eventually stop in case something goes wrong 
    
    #check if button is pressed
        
        
        if CheckPress(XLimit): #button pressed once. need to move forward and back again to ensure correct start position
            MoveX(XFORWARD,300,SLOW) #move forward
            for j in range(350): #move back and check again
                if CheckPress(XLimit): #again
                    
                
                    print('Button has been pressed after {} steps!'.format(i))
                    print('was already homed check: took {} out of 300 steps on the second bounce'.format(j))
                    GlobalX = 0
                    XPosition.configure(text="X: " +str(GlobalX) + "/" + str(XMax))
                    return (i) #break away essentially
                MoveX(XBACKWARD,1,SLOW)
            #do stepping protocol (placed second in case button already pressed)
        MoveX(XBACKWARD,1,FASTER)#dir dis delay

def HomeY():
    global GlobalY
    for i in range(YMax + 200): #some number that's noticably larger than the range, but also will eventually stop in case something goes wrong 
    
    #check if button is pressed
        
        
        if CheckPress(YLimit): #button pressed once. need to move forward and back again to ensure correct start position
            MoveY(YFORWARD,300,SLOW) #move forward
            for j in range(350): #move back and check again
                if CheckPress(YLimit): #again
                    
                
                    print('Button has been pressed after {} steps!'.format(i))
                    print('was already homed check: took {} out of 300 steps on the second bounce'.format(j))
                    GlobalY = 0
                    YPosition.configure(text="Y: "+str(GlobalY) + "/" + str(YMax))
                    return (i) #break away essentially
                MoveY(YBACKWARD,1,SLOW)
            #do stepping protocol (placed second in case button already pressed)
        MoveY(YBACKWARD,1,FASTER)#dir dis delay

def HomeZ():
    
    '''This is a bit different than X and Y, because the optical switch is tripped about a thousand steps up from the true bottom!
 It's more hardcoded with actual numbers. 
 This whole procedure should be replaced with a substituted-in copy of HomeY or X if using a regular physical switch.'''

    global GlobalZ
    
    for i in range(ZMax + 500): 
    
    #check if button is pressed
        
    
        if CheckPress(ZLimit): #button pressed once. need to move forward and back again to ensure correct start position
            MoveZ(ZFORWARD,1200,FAST) #move forward -- at least 1k b/c neg range
            for j in range(1400): #move back and check again
                if CheckPress(ZLimit): #again
                    
                    MoveZ(ZBACKWARD, 1000, FAST) #START AT MINIMUM RANGE for easier calculating

                    print('Optical switch has been tripped after {} steps!'.format(i))
                    print('was already homed check: took {} out of 1200 steps on the second bounce'.format(j))
                    
                    
                    GlobalZ = 0
                    ZPosition.configure(text="Z: "+str(GlobalZ) + "/" + str(ZMax))
                    return (i) #break away essentially
                MoveZ(ZBACKWARD,1,FAST)
            #do stepping protocol (second in case button already pressed)
        MoveZ(ZBACKWARD,1,FAST)#dir dis delay
   
def DefineScan(XMin, XMax, YMin, YMax, ZMin=0, ZMax=0, RMin=0, RMax=0, XSteps=100, YSteps=100, ZSteps=1, RSteps=1):
    """
    Used to generate a dictionary with four keys, each of which maps to a list containing
    the absolute positions of X,Y,Z, and R, for every point of a scan.
    So if you don't want to move Z and R, just for instance set Zmin=Zmax=0 and ZSteps = 1.
    
    Core, of 2-axis control, from stack exchange. https://stackoverflow.com/questions/20872912/raster-scan-pattern-python
    """
    
    # Add 1 to max values to make ranges inclusive and prevent errors when min=max
    XMax = XMax+1 
    YMax = YMax+1
    ZMax = ZMax+1 
    RMax = RMax+1

    # Define grids of points for each axis using numpy's arange
    # arange generates evenly spaced values from min to max with step size
    xgrid = arange(XMin, XMax, XSteps)  
    ygrid = arange(YMin, YMax, YSteps)
    zgrid = arange(ZMin, ZMax, ZSteps)
    rgrid = arange(RMin, RMax, RSteps)

    # Empty lists to store scan pattern coordinates
    xscan = []
    yscan = []

    # Generate raster scan pattern for X-Y plane
    for i, yi in enumerate(ygrid):
        # Alternate direction of X movement on each Y row
        xscan.append(xgrid[::(-1)**i]) # Reverse X direction when i is odd
        # Create corresponding Y coordinates 
        yscan.append(ones_like(xgrid) * yi)   

    # Combine X-Y coordinates into continuous vectors
    xscan = concatenate(xscan)
    yscan = concatenate(yscan)

    # Add Z dimension to scan pattern
    NewXScan = []
    NewYScan = []
    NewZScan = []
    
    # Repeat X-Y pattern for each Z height
    for i in range(len(zgrid)):
        for j in range(len(xscan)):
            NewXScan.append(xscan[j])
            NewYScan.append(yscan[j])
            NewZScan.append(zgrid[i])

    # Add rotation dimension
    NewNewXScan = []
    NewNewYScan = [] 
    NewNewZScan = []
    NewNewRScan = []
    
    # Repeat X-Y-Z pattern for each rotation angle
    for i in range(len(rgrid)):
        for j in range(len(NewXScan)):
            NewNewXScan.append(NewXScan[j])
            NewNewYScan.append(NewYScan[j])
            NewNewZScan.append(NewZScan[j])
            NewNewRScan.append(rgrid[i])
    
    # Create dictionary with complete scan coordinates
    ScanLocations = {
        'X': NewNewXScan,
        'Y': NewNewYScan,
        'Z': NewNewZScan,
        'R': NewNewRScan
    }
    
    return(ScanLocations)


def GridScan(ScanLocations, conditions='default'):
    """The main loop that carries out an actual scan. 
    Accepts the dictionary output of DefineScan, and an optional dictionary of conditions,
    which is necessary when performing a restart in the middle of the scan."""
     
    # Extract coordinate lists from dictionary
    XCoord = ScanLocations['X'] 
    YCoord = ScanLocations['Y']
    ZCoord = ScanLocations['Z']
    RCoord = ScanLocations['R']
    
    start_time = time.time()
    
    # Initialize scan parameters for new scan
    if conditions == 'default':
        # Get save location via GUI dialog
        save_location = filedialog.askdirectory()
        filetype = ".jpg"  # File format for saved images
        resolution = "640x480"  # Camera resolution
        timeallowed = 30  # Seconds to wait for USB reconnection
        num_failures = 0  # Counter for restart attempts
        original_pics = len(XCoord)  # Total number of pictures to take
        original_time = start_time
        original_locations = ScanLocations
        failed_pics = []  # Track which pictures failed
        failure_times = []  # Track when failures occurred
        
    else:
        # Restore parameters from previous failed scan
        save_location = conditions['save_location']
        filetype = conditions['filetype']
        resolution = conditions['resolution']
        timeallowed = 0  # No retry after restart
        num_failures = conditions['num_failures']
        failed_pics = conditions['failed_pics']
        failure_times = conditions['failure_times']
        original_pics = conditions['original_pics']
        original_time = conditions['original_time']
        original_locations = conditions['original_locations']
        
    num_pictures = len(XCoord)  # Remaining pictures to take
    NumberOfRotations = len(set(RCoord))  # Number of unique rotation angles
    stepsPerRotation = ((max(RCoord)-min(RCoord))/len(set(RCoord)))
    
    print("has failed and restarted {} times so far".format(str(num_failures)))
    
    # Move to initial position
    XGoTo(int(XCoord[0]))
    YGoTo(int(YCoord[0]))
    ZGoTo(int(ZCoord[0]))
    RGoTo(int(RCoord[0]))
    

    # Main scanning loop
    for i in range(num_pictures):
        
        # Progress indicator beep every 100 pictures
        if i % 100 == 0:
            print("{} of {} pictures remaining".format((num_pictures-i), original_pics))
            GPIO.output(BEEP, GPIO.HIGH) 
            time.sleep(0.3)
            GPIO.output(BEEP, GPIO.LOW)
            
        # Create new folder for each Z height and rotation angle combination
        folder = save_location + "/Z" + str(ZCoord[i]).zfill(4) + "R" + str(RCoord[i]).zfill(3) 
        if not os.path.exists(folder):
            os.makedirs(folder)
                
        # Move to next scan position    
        XGoTo(int(XCoord[i]))
        YGoTo(int(YCoord[i]))
        ZGoTo(int(ZCoord[i]))
        RGoTo(int(RCoord[i]))
            
        # Wait for vibrations to settle
        time.sleep(0.2)
               
        # Generate filename with position information
        name = "X" + str(XCoord[i]).zfill(4) + "Y" + str(YCoord[i]).zfill(4) + \
               "Z" + str(ZCoord[i]).zfill(4) + "R" + str(RCoord[i]).zfill(3) + \
               "of" + str(NumberOfRotations).zfill(3) + filetype

        # Try to take and save picture
        try:
            # Make 3 attempts to take picture
            for w in range(3):
                # Run fswebcam command to capture image
                proc = subprocess.Popen(["fswebcam", "-r " + resolution, "--no-banner", 
                                       folder + "/" + name, "-q"], 
                                       stdout=subprocess.PIPE)
                output = proc.communicate(timeout=10)[0]
                
                # Check if file was created successfully
                if os.path.isfile(folder + "/" + name):
                    if w > 0:  # USB was disconnected but reconnected
                        print('Okay thanks bozo. Restarting with {}'.format(name))
                    break  # Success - move to next picture
                
                elif (w <= 1):  # USB disconnect detected
                    print('HEY BOZO THE USB GOT UNPLUGGED UNPLUG IT AND PLUG IT BACK IN WITHIN {} SECONDS OR WE REBOOT'.format(timeallowed))
                    print('check if {} failed'.format(name))
                
                    GPIO.output(BEEP, GPIO.HIGH)  # Alert beep
                    time.sleep(timeallowed)
                    GPIO.output(BEEP, GPIO.LOW)
                
                else:  # Failed all 3 attempts - save state and restart
                    # Save remaining coordinates
                    UpdatedX = XCoord[i:]
                    UpdatedY = YCoord[i:]
                    UpdatedZ = ZCoord[i:]
                    UpdatedR = RCoord[i:]
                    
                    UpdatedScanLocations = {'X':UpdatedX, 'Y':UpdatedY, 
                                          'Z':UpdatedZ, 'R':UpdatedR}
                    
                    num_failures += 1
                    failed_pics.append(name)
                    failure_times.append(time.time())
                    
                    # Save scan state for restart
                    conditions = {
                        'save_location': save_location,
                        'R_Location': int(RCoord[i]),
                        'filetype': filetype,
                        'resolution': resolution,
                        'num_failures': num_failures,
                        'original_pics': original_pics,
                        'original_time': original_time,
                        'original_locations': original_locations,
                        'failed_pics': failed_pics,
                        'failure_times': failure_times
                    }
                        
                    scan_params = [UpdatedScanLocations, conditions]
                    print(scan_params)
                    time.sleep(2)
                    
                    # Save scan data to file
                    scan_file = open('/home/pi/Desktop/ladybug/scandata.pkl', 'wb')
                    pickle.dump(scan_params, scan_file)
                    scan_file.close()
                    
                    print('restarting sorryyyyyy')
                    restart()
                        
        except subprocess.TimeoutExpired:
            print ("{} failed :( ".format(name))
            proc.terminate()
            continue    
        
            
    # Print completion statistics
    print ('scan completed successfully after {} seconds! {} images taken and {} restarts'.format(
           time.strftime("%H:%M:%S", time.gmtime(time.time() - original_time)), 
           str(original_pics),
           str(num_failures)))
    
    try:
        # Rename scan data file to prevent infinite restart loop
        os.rename('/home/pi/Desktop/ladybug/scandata.pkl',
                 '/home/pi/Desktop/ladybug/scandataold.pkl') 
    except FileNotFoundError:  # No restart occurred during scan
        pass
    
    # Final completion beeps
    for i in range(5):
        GPIO.output(BEEP, GPIO.HIGH)
        time.sleep(0.2)
        GPIO.output(BEEP, GPIO.LOW)

    # Wait for user acknowledgment before closing
    a = input('press any key to exit')
    


#------------- end main scan procedure. 


def XRepeatTest(num_trials=100):
    """This will move a lot and then home, to check to make sure we're not skipping steps or something."""
  
    # Home the X axis three times to ensure reliable starting position
    HomeX()  # First home to get close
    HomeX()  # Second home for better accuracy
    HomeX()  # Third home for maximum precision/certainty
    
    # Access global maximum range for X axis
    global XMax
    # Initialize counter for total steps moved
    total_steps = 0
    # Store previous random location, start at 0
    ranlocold = 0
    # List to store all positions visited, including starting position 0
    ListOfLoc = [0]
    
    # Perform specified number of random movements
    for i in range(num_trials):
        # Generate random position with 100-step buffer from limits
        ranloc = random.randrange(100, XMax-100)
        # Add absolute distance moved to total step count
        total_steps += (abs(ranlocold-ranloc))
        # Update previous location for next iteration
        ranlocold = ranloc
        # Add new position to history list
        ListOfLoc.append(ranloc)
        
        # Move to the random location
        XGoTo(ranloc)
    
    # Return to home position and get steps taken
    StepsToHome = HomeX()  # Returns steps taken to reach home
   
    # Calculate positioning error
    imperfection = ranloc - StepsToHome
    
    # Print detailed test results
    print("after {} total steps, over {} movements, it took {} steps to home instead of {}, for an imperfection of {}".format(
        total_steps,    # Total distance covered
        num_trials,     # Number of movements made
        StepsToHome,    # Steps needed to return home
        ranloc,         # Expected distance from home
        imperfection    # Difference between expected and actual
    ))
    
    # Return list containing test metrics
    return([total_steps,      # Total steps moved
            imperfection,     # Final positioning error
            ListOfLoc])       # Complete position history 


def YRepeatTest(num_trials=100): 
    """Same as XRepeatTest but for Y axis"""
    HomeY()    # Triple home for precision
    HomeY()    
    HomeY()
    
    global YMax    # Get Y axis range
    total_steps = 0    # Step counter
    ranlocold = 0     # Previous position
    ListOfLoc = [0]   # Position history
    
    for i in range(num_trials):
        ranloc = random.randrange(100, YMax-100)  # Random position with buffer
        total_steps += (abs(ranlocold-ranloc))    # Add to total distance
        ranlocold = ranloc                        # Update previous position
        ListOfLoc.append(ranloc)                  # Record position
        YGoTo(ranloc)                            # Move to position
    
    StepsToHome = HomeY()                        # Return home
    imperfection = ranloc - StepsToHome          # Calculate error
     
    print("after {} total steps, over {} movements, it took {} steps to home instead of {}, for an imperfection of {}".format(
        total_steps, num_trials, StepsToHome, ranloc, imperfection))
    
    return([total_steps, imperfection, ListOfLoc])    

def MultiRepeatTest(num_repeats = 100):
    """Runs multiple X/Y repeat tests for statistical analysis"""
    
    X_History = []    # Store X test results
    Y_History = []    # Store Y test results
    
    for i in range(num_repeats):
        numpertrial = 100    # Movements per test
        
        XHomeData = XRepeatTest(numpertrial)    # Run X test
        YHomeData = YRepeatTest(numpertrial)    # Run Y test
        
        XHomeData.append(numpertrial)    # Add trial count to results
        YHomeData.append(numpertrial)
        
        X_History.append(XHomeData)    # Store results
        Y_History.append(YHomeData)
            
    return(X_History, Y_History)    # Return all test data

# Function to handle X-axis position input from GUI text entry
def XGet(event):
    '''Records enter key press from text box (XEntry) and calls "go to specified location" function'''
    try:
        # Get value from entry widget and convert to integer
        XDest = int(event.widget.get())
        # Call XGoTo function to move X axis to specified position
        XGoTo(XDest)
    except ValueError: # Catch non-integer input errors
        print ("hey dumbo enter an integer")    

# Function to handle Y-axis position input from GUI text entry
def YGet(event):
    '''Records enter key press from text box (YEntry) and calls "go to specified location" function'''
    try:
        # Get value from entry widget and convert to integer
        YDest = int(event.widget.get())
        # Call YGoTo function to move Y axis to specified position
        YGoTo(YDest)
    except ValueError: # Catch non-integer input errors
        print ("hey dumbo enter an integer")    

# Function to handle Z-axis position input from GUI text entry
def ZGet(event):
    '''Records enter key press from text box (ZEntry) and calls "go to specified location" function'''
    try:
        # Get value from entry widget and convert to integer
        ZDest = int(event.widget.get())
        # Call ZGoTo function to move Z axis to specified position
        ZGoTo(ZDest)
    except ValueError: # Catch non-integer input errors
        print ("hey dumbo enter an integer")    

# Function to initiate scan with current parameters
def GuiScan():
    # Convert number of R rotations to step size
    # Note: Temporary solution for R-axis movement
    RScanStep = 160/RScanNumber # Calculate step size based on total steps per rotation
    print(RScanStep)
    
    # Create scan grid with current parameters
    CallForGrid = DefineScan(XScanMin,XScanMax,
                           YScanMin,YScanMax,
                           ZScanMin,ZScanMax,
                           RScanMin,StepsPerRotation,
                           XScanStep,YScanStep,
                           ZScanStep,RScanStep)
                                        
    # Execute the scan with defined grid
    GridScan(CallForGrid)

# Function to set number of rotations from dropdown menu
def SetR():
    global RScanNumber # Access global variable
    RScanNumber = int(RSetVar.get()) # Convert selection to integer
    print ('viewing {} points of view'.format(str(RScanNumber)))

# X-axis movement functions - large steps
def MoveXLeftBig(): 
    MoveX(XBACKWARD,100,FAST) # Move 100 steps backward at fast speed
    print("X moved to da left a lot!")
    
# X-axis movement functions - small steps
def MoveXLeftSmall():
    MoveX(XBACKWARD,10,SLOW) # Move 10 steps backward at slow speed
    print("X moved to da left a little!")

def MoveXRightBig():
    MoveX(XFORWARD,100,FAST) # Move 100 steps forward at fast speed
    print("X moved to da right a lot!")
    
def MoveXRightSmall():
    MoveX(XFORWARD,10,SLOW) # Move 10 steps forward at slow speed
    print("X moved to da right a little!")

# Y-axis movement functions - large steps
def MoveYForwardBig():
    MoveY(YFORWARD,100,FAST) # Move 100 steps forward at fast speed
    print("Y moved forward a lot!")
    
# Y-axis movement functions - small steps
def MoveYForwardSmall():
    MoveY(YFORWARD,10,SLOW) # Move 10 steps forward at slow speed
    print("Y moved forward a little!")

def MoveYBackBig():
    MoveY(YBACKWARD,100,FAST) # Move 100 steps backward at fast speed
    print("Y moved back a lot!")
    
def MoveYBackSmall():
    MoveY(YBACKWARD,10,SLOW) # Move 10 steps backward at slow speed
    print("Y moved back a little!")

# Z-axis movement functions - large steps
def MoveZDownBig():
    MoveZ(ZBACKWARD,250,FASTER) # Move 250 steps down at faster speed
    print("Z moved down a lot!")
    
def MoveZDownSmall():
    MoveZ(ZBACKWARD,25,FAST) # Move 25 steps down at fast speed
    print("Z moved down a little!")

def MoveZUpBig():
    MoveZ(ZFORWARD,250,FASTER) # Move 250 steps up at faster speed
    print("Z moved up a lot!")
    
def MoveZUpSmall():
    MoveZ(ZFORWARD,25,FAST) # Move 25 steps up at fast speed
    print("Z moved up a little!")

def MoveRCWSmall():
    # Clockwise rotation when looking down at spindle/object
    MoveR(RFORWARD, 8, SLOW) # Move 8 steps clockwise at slow speed
    print("You rotated something clockwise a bit!")
    
def MoveRCCWSmall():
    # Counter-clockwise rotation
    MoveR(RBACKWARD, 8, SLOW) # Move 8 steps counter-clockwise at slow speed
    print("You rotated something counterclockwise a bit!")
    
# Program exit function
def exitProgram():
    print("Exit Button pressed")
    GPIO.cleanup() # Clean up GPIO settings
    win.quit() # Exit Tkinter window

#--------- BUTTONS FOR SETTING SCAN PARAMETERS

def SetXLowerBound():
    # Set minimum X coordinate for scan
    global XScanMin
    XScanMin = GlobalX # Use current X position as minimum
    print("Lower Boundary for X Scan has been set to {}".format(XScanMin))

def SetXUpperBound():
    # Set maximum X coordinate for scan
    global XScanMax
    XScanMax = GlobalX # Use current X position as maximum
    print("Upper Boundary for X Scan has been set to {}".format(XScanMax))

def SetYLowerBound():
    # Set minimum Y coordinate for scan
    global YScanMin
    YScanMin = GlobalY # Use current Y position as minimum
    print("Lower Boundary for Y Scan has been set to {}".format(YScanMin))

def SetYUpperBound():
    # Set maximum Y coordinate for scan
    global YScanMax
    YScanMax = GlobalY # Use current Y position as maximum
    print("Upper Boundary for Y Scan has been set to {}".format(YScanMax))

def SetZLowerBound():
    # Set minimum Z coordinate for scan
    global ZScanMin
    ZScanMin = GlobalZ # Use current Z position as minimum
    print("Lower Boundary for Z Scan has been set to {}".format(ZScanMin))

def SetZUpperBound():
    # Set maximum Z coordinate for scan
    global ZScanMax
    ZScanMax = GlobalZ # Use current Z position as maximum
    print("Upper Boundary for Z Scan has been set to {}".format(ZScanMax))

def SetZStep():
    # Set Z axis step size for scan
    global ZScanStep
    # Uses current Z position as step size (unconventional but minimalist approach)
    ZScanStep = GlobalZ 
    print("Step size for Z has been set to {}".format(ZScanStep))

def SetXStep():
    # Set X axis step size for scan
    global XScanStep
    XScanStep = GlobalX # Use current X position as step size
    print("Step size for X has been set to {}".format(YScanStep))
    
def SetYStep():
    # Set Y axis step size for scan
    global YScanStep
    YScanStep = GlobalY # Use current Y position as step size
    print("Step size for Y has been set to {}".format(YScanStep))

#Begin disgusting block of instructions for keypress. Complicated by fact we want to be able to hold Down.
#Modified from https://stackoverflow.com/questions/12994796/how-can-i-control-keyboard-repeat-delay-in-a-tkinter-root-window
#main changes are not displaying the Label and also only enabling key control when checkbox is checked (allow_keypress)
#works okay for hitton the key once but not so much for holding it down.  


# KEYBOARD CONTROL FUNCTIONS
# Modified from Stack Overflow for key repeat functionality
# Only active when checkbox is checked (allow_keypress)

def LeftStep(*event):
    # Handle left arrow key press
    MoveYForwardSmall()
    
    # If repeat is enabled, schedule next movement
    if LeftLabel._repeat_on:
        win.after(LeftLabel._repeat_freq, LeftStep)

def LeftStop(*event):
    # Handle left arrow key release
    if LeftLabel._repeat_on:
        LeftLabel._repeat_on = False
        win.after(LeftLabel._repeat_freq + 1, LeftStop)
    else:
        LeftLabel._repeat_on = True

def RightStep(*event):
    # Handle right arrow key press
    MoveYBackSmall()

    if RightLabel._repeat_on:
        win.after(RightLabel._repeat_freq, RightStep)

def RightStop(*event):
    # Handle right arrow key release
    if RightLabel._repeat_on:
        RightLabel._repeat_on = False
        win.after(RightLabel._repeat_freq + 1, RightStop)
    else:
        RightLabel._repeat_on = True

def UpStep(*event):
    # Handle up arrow key press
    MoveXLeftSmall()

    if UpLabel._repeat_on:
        win.after(UpLabel._repeat_freq, UpStep)

def UpStop(*event):
    # Handle up arrow key release
    if UpLabel._repeat_on:
        UpLabel._repeat_on = False
        win.after(UpLabel._repeat_freq + 1, UpStop)
    else:
        UpLabel._repeat_on = True

def DownStep(*event):
    # Handle down arrow key press
    MoveXRightSmall()

    # If repeat is enabled, schedule next movement
    if DownLabel._repeat_on:
        win.after(DownLabel._repeat_freq, DownStep)

def DownStop(*event):
    # Handle down arrow key release
    if DownLabel._repeat_on:
        DownLabel._repeat_on = False
        win.after(DownLabel._repeat_freq + 1, DownStop)
    else:
        DownLabel._repeat_on = True

def AStep(*event):
    # Handle 'a' key press for counter-clockwise rotation
    MoveRCCWSmall()

    if ALabel._repeat_on:
        win.after(ALabel._repeat_freq, AStep)

def AStop(*event):
    # Handle 'a' key release
    if ALabel._repeat_on:
        ALabel._repeat_on = False
        win.after(ALabel._repeat_freq + 1, AStop)
    else:
        ALabel._repeat_on = True

def DStep(*event):
    # Handle 'd' key press for clockwise rotation
    MoveRCWSmall()

    if DLabel._repeat_on:
        win.after(DLabel._repeat_freq, DStep)

def DStop(*event):
    # Handle 'd' key release
    if DLabel._repeat_on:
        DLabel._repeat_on = False
        win.after(DLabel._repeat_freq + 1, DStop)
    else:
        DLabel._repeat_on = True

def WStep(*event):
    # Handle 'w' key press for Z axis up movement
    MoveZUpSmall()

    if WLabel._repeat_on:
        win.after(WLabel._repeat_freq, WStep)

def WStop(*event):
    # Handle 'w' key release
    if WLabel._repeat_on:
        WLabel._repeat_on = False
        win.after(WLabel._repeat_freq + 1, WStop)
    else:
        WLabel._repeat_on = True

def SStep(*event):
    # Handle 's' key press for Z axis down movement
    MoveZDownSmall()

    if SLabel._repeat_on:
        win.after(SLabel._repeat_freq, SStep)

def SStop(*event):
    # Handle 's' key release
    if SLabel._repeat_on:
        SLabel._repeat_on = False
        win.after(SLabel._repeat_freq + 1, SStop)
    else:
        SLabel._repeat_on = True

def allow_keypress():
    # Enable/disable keyboard controls based on checkbox state
    if keypress_var.get(): # If checkbox is checked
        # Store binding references in global variables
        global Leftbound, Leftunbound, Rightbound, Rightunbound
        global Upbound, Upunbound, Downbound, Downunbound
        global Abound, Aunbound, Dbound, Dunbound
        global Wbound, Wunbound, Sbound, Sunbound
        
        # Bind keyboard events to their respective functions
        Leftbound = win.bind('<KeyPress-Left>', LeftStep)
        Leftunbound = win.bind('<KeyRelease-Left>', LeftStop)
        Rightbound = win.bind('<KeyPress-Right>', RightStep)
        Rightunbound = win.bind('<KeyRelease-Right>', RightStop)
        Upbound = win.bind('<KeyPress-Up>', UpStep)
        Upunbound = win.bind('<KeyRelease-Up>', UpStop)
        Downbound = win.bind('<KeyPress-Down>', DownStep)
        Downunbound = win.bind('<KeyRelease-Down>', DownStop)
        Abound = win.bind('<KeyPress-a>', AStep)
        Aunbound = win.bind('<KeyRelease-a>', AStop)
        Dbound = win.bind('<KeyPress-d>', DStep)
        Dunbound = win.bind('<KeyRelease-d>', DStop)
        Wbound = win.bind('<KeyPress-w>', WStep)
        Wunbound = win.bind('<KeyRelease-w>', WStop)
        Sbound = win.bind('<KeyPress-s>', SStep)
        Sunbound = win.bind('<KeyRelease-s>', SStop)
        
    else:
        # Unbind all keyboard events when checkbox is unchecked
        win.unbind('<KeyPress-Left>', Leftbound)
        win.unbind('<KeyRelease-Left>', Leftunbound)
        win.unbind('<KeyPress-Right>', Rightbound)
        win.unbind('<KeyRelease-Right>', Rightunbound)
        win.unbind('<KeyPress-Up>', Upbound)
        win.unbind('<KeyRelease-Up>', Upunbound)
        win.unbind('<KeyPress-Down>', Downbound)
        win.unbind('<KeyRelease-Down>', Downunbound)
        win.unbind('<KeyPress-a>', Abound)
        win.unbind('<KeyRelease-a>', Aunbound)   
        win.unbind('<KeyPress-d>', Dbound)
        win.unbind('<KeyRelease-d>', Dunbound)   
        win.unbind('<KeyPress-w>', Wbound)
        win.unbind('<KeyRelease-w>', Wunbound)   
        win.unbind('<KeyPress-s>', Sbound)
        win.unbind('<KeyRelease-s>', Sunbound)

#---------- BEGIN WHAT GOES ONSCREEN. Someone who knows Tkinter, please fix this. I've forgotten what everything does. 

# Main window setup
win.title("Raspberry Pi GUI")
win.geometry('1400x880')

# Create main frames for layout organization
LeftFrame = tk.Frame(win)
LeftFrame.pack(side = tk.LEFT)

RightFrame = tk.Frame(win)
RightFrame.pack(side = tk.RIGHT)

TopFrame = tk.Frame(win)
TopFrame.pack(side = tk.TOP)

BottomFrame = tk.Frame(win)
BottomFrame.pack(side = tk.BOTTOM)

# X-axis scan control buttons
XStepButton = tk.Button(LeftFrame, text = "Set XScan Stepsize ", font = myFont, command = SetXStep, height = 1, width =20 )
XStepButton.pack(side = tk.TOP, pady=5)
XLowerBoundButton = tk.Button(LeftFrame, text = "Set XScan Min", font = myFont, command = SetXLowerBound, height = 1, width =20 )
XLowerBoundButton.pack(side = tk.TOP, pady=5)
XUpperBoundButton = tk.Button(LeftFrame, text = "Set XScan Max", font = myFont, command = SetXUpperBound, height = 1, width =20 )
XUpperBoundButton.pack(side = tk.TOP, pady=5)
# Y-axis scan control buttons
YStepButton = tk.Button(TopFrame, text = "Set YScan Stepsize ", font = myFont, command = SetYStep, height = 1, width =20 )
YStepButton.pack(side = tk.BOTTOM, pady=5)
YUpperBoundButton = tk.Button(TopFrame, text = "Set YScan Max ", font = myFont, command = SetYUpperBound, height = 1, width =20 )
YUpperBoundButton.pack(side = tk.BOTTOM, pady=5)
YLowerBoundButton = tk.Button(TopFrame, text = "Set YScan Min ", font = myFont, command = SetYLowerBound, height = 1, width =20 )
YLowerBoundButton.pack(side = tk.BOTTOM, pady=5)
# Z-axis scan control buttons
ZStepButton = tk.Button(RightFrame, text = "Set ZScan Stepsize ", font = myFont, command = SetZStep, height = 1, width =20 )
ZStepButton.pack(side = tk.TOP, pady=5)
ZLowerBoundButton = tk.Button(RightFrame, text = "Set ZScan Min ", font = myFont, command = SetZLowerBound, height = 1, width =20 )
ZLowerBoundButton.pack(side = tk.TOP, pady=5)
ZUpperBoundButton = tk.Button(RightFrame, text = "Set ZScan Max ", font = myFont, command = SetZUpperBound, height = 1, width =20 )
ZUpperBoundButton.pack(side = tk.TOP, pady=5)
# Y-axis position display and control
YPosition = tk.Label(TopFrame, font=(myFont), height = 2, width=12) #use a Label widget, not Text
YPosition.pack(side = tk.TOP)
# Y-axis position entry box
YEntry = tk.Entry(TopFrame, width = 4)
YEntry.bind('<Return>', YGet)
YEntry.pack(side=tk.TOP)
# Y-axis movement buttons
YForwardBigButton = tk.Button(TopFrame, text = "⇑", font = myFont, command = MoveYForwardBig, height = 1, width =2 )
YForwardBigButton.pack(side = tk.TOP)

YForwardSmallButton = tk.Button(TopFrame, text = "↑", font = myFont, command = MoveYForwardSmall, height = 1, width =2 )
YForwardSmallButton.pack(side = tk.TOP)

YBackSmallButton = tk.Button(TopFrame, text = "↓", font = myFont, command = MoveYBackSmall, height = 1, width =2 )
YBackSmallButton.pack(side = tk.TOP)

YBackBigButton = tk.Button(TopFrame, text = "⇓", font = myFont, command = MoveYBackBig, height = 1, width =2 )
YBackBigButton.pack(side = tk.TOP)

# X-axis position display and control
XPosition = tk.Label(LeftFrame, font=(myFont), height = 2, width=12) #use a Label widget, not Text
XPosition.pack(side = tk.LEFT)
# X-axis position entry box
XEntry = tk.Entry(LeftFrame, width = 4)
XEntry.bind('<Return>', XGet)
XEntry.pack(side=tk.LEFT)
# X-axis movement buttons
XLeftBigButton = tk.Button(LeftFrame, text = "⟸", font = myFont, command = MoveXLeftBig, height = 1, width =2 )
XLeftBigButton.pack(side = tk.LEFT)

XLeftSmallButton = tk.Button(LeftFrame, text = "←", font = myFont, command = MoveXLeftSmall, height = 1, width =2 )
XLeftSmallButton.pack(side = tk.LEFT)

XRightSmallButton = tk.Button(LeftFrame, text = "→", font = myFont, command = MoveXRightSmall, height = 1, width =2 )
XRightSmallButton.pack(side = tk.LEFT)

XRightBigButton = tk.Button(LeftFrame, text = "⟹", font = myFont, command = MoveXRightBig, height = 1, width =2 )
XRightBigButton.pack(side = tk.LEFT)


# Z-axis position display and control
ZPosition = tk.Label(RightFrame, font=(myFont), height = 2, width=12) #use a Label widget, not Text
ZPosition.pack(side = tk.RIGHT)
# Z-axis position entry box
ZEntry = tk.Entry(RightFrame, width = 4)
ZEntry.bind('<Return>', ZGet)
ZEntry.pack(side=tk.RIGHT)
# Z-axis movement buttons
ZUpBigButton = tk.Button(RightFrame, text = "Z⇑", font = myFont, command = MoveZUpBig, height = 1, width =2 )
ZUpBigButton.pack(side = tk.RIGHT)

ZUpSmallButton = tk.Button(RightFrame, text = "Z↑", font = myFont, command = MoveZUpSmall, height = 1, width =2 )
ZUpSmallButton.pack(side = tk.RIGHT)

ZDownSmallButton = tk.Button(RightFrame, text = "Z↓", font = myFont, command = MoveZDownSmall, height = 1, width =2 )
ZDownSmallButton.pack(side = tk.RIGHT)

ZDownBigButton = tk.Button(RightFrame, text = "Z⇓", font = myFont, command = MoveZDownBig, height = 1, width =2 )
ZDownBigButton.pack(side = tk.RIGHT)
# Home position buttons
HomeXButton = tk.Button(BottomFrame, text = "HOME X", font = myFont, command = HomeX, height = 2, width =8 )
HomeXButton.pack(side = tk.BOTTOM,pady=5)

HomeYButton = tk.Button(BottomFrame, text = "HOME Y", font = myFont, command = HomeY, height = 2, width =8 )
HomeYButton.pack(side = tk.BOTTOM,pady=5)

HomeZButton = tk.Button(BottomFrame, text = "HOME Z", font = myFont, command = HomeZ, height = 2, width =8 )
HomeZButton.pack(side = tk.BOTTOM,pady=5)

# Rotation control buttons
RCWSmallButton = tk.Button(BottomFrame, text = "↻", font = myBigFont, command = MoveRCWSmall, height = 1, width = 2)
RCWSmallButton.pack(side = tk.BOTTOM, pady=5)

RCCWSmallButton = tk.Button(BottomFrame, text = "↺", font = myBigFont, command = MoveRCCWSmall, height = 1, width = 2)
RCCWSmallButton.pack(side = tk.BOTTOM,pady=5)
# Main scan button
ScanButton = tk.Button(BottomFrame, text = "SCAN!!!", font = myBigFont, command = GuiScan, height = 1, width = 20)
ScanButton.pack(side = tk.TOP, pady=110)
# Create secondary bottom frame for additional controls
SecondaryBottomFrame = tk.Frame(BottomFrame)
SecondaryBottomFrame.pack(side=tk.TOP)
# Rotation settings controls
RSetVar = tk.StringVar(SecondaryBottomFrame) #holds contents of dropdown? 
RSetVar.set(FactorsOf160[0])
RSetButton = tk.Button(SecondaryBottomFrame, text = "Set number of rotations", font = myFont, command = SetR, height = 1, width = 20)
RSetButton.pack(side = tk.LEFT, padx=5)
RSetDropdown = tk.OptionMenu(SecondaryBottomFrame, RSetVar, *FactorsOf160)
RSetDropdown.pack(side = tk.RIGHT, padx=5)
# Keyboard control enable/disable checkbox
keypress_var = tk.IntVar() #1 if button is pressed
keypress_button = tk.Checkbutton(SecondaryBottomFrame, text="ENABLE KEYBOARD CONTROLS", variable=keypress_var, command=allow_keypress)
keypress_button.pack(side = tk.RIGHT)

# Labels for keyboard control repeat functionality
# Each label stores repeat frequency and state
LeftLabel = tk.Label(win)
LeftLabel._repeat_freq = int(SLOW*1000*10) #holding Down key, milisecond per repeat.. Delay should be how long it actually takes to move
LeftLabel._repeat_on = True

RightLabel = tk.Label(win)
RightLabel._repeat_freq = int(SLOW*1000*10) 
RightLabel._repeat_on = True

UpLabel = tk.Label(win)
UpLabel._repeat_freq = int(SLOW*1000*10) 
UpLabel._repeat_on = True

DownLabel = tk.Label(win)
DownLabel._repeat_freq = int(SLOW*1000*10)  
DownLabel._repeat_on = True

# Labels for rotation controls (a and d keys)
ALabel = tk.Label(win) #a and d are rotation
ALabel._repeat_freq = int(SLOW*1000*10)  
ALabel._repeat_on = True

DLabel = tk.Label(win)
DLabel._repeat_freq = int(SLOW*1000*10)  
DLabel._repeat_on = True
# Labels for Z-axis controls (w and s keys)
WLabel = tk.Label(win)
WLabel._repeat_freq = int(SLOW*1000*10)  
WLabel._repeat_on = True

SLabel = tk.Label(win)
SLabel._repeat_freq = int(SLOW*1000*10)  
SLabel._repeat_on = True


"""BEGIN MAIN LOOP, beginning with an attempt to resume scan if it detects that a previous one failed 
(existence of scandata file)"""


try:
    # Initial beep pattern to indicate startup
    GPIO.output(BEEP, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(BEEP, GPIO.LOW)
    time.sleep(0.2)
    GPIO.output(BEEP, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(BEEP, GPIO.LOW)
    
    # Attempt to open previously interrupted scan data
    scan_file = open('/home/pi/Desktop/ladybug/scandata.pkl', 'rb')
    
    # Load the saved scan parameters
    scan_params = pickle.load(scan_file)
    scan_file.close()    
    
    # Home all axes except R
    HomeX() 
    HomeY()
    HomeZ()
    
    # Extract scan data from saved parameters
    locations = scan_params[0]  # Position data
    conditions = scan_params[1]  # Scan settings (save location, filetype, resolution, timeout, numfailures)
    
    # Restore R axis position
    # Note: R axis has no endstop, so we must trust the saved position
    GlobalR = conditions['R_Location']  
    
    # Resume the interrupted scan
    GridScan(locations, conditions)

except FileNotFoundError:
    # No interrupted scan found - fresh start
    print('No interrupted scans found. Welcome to the scanner.')
