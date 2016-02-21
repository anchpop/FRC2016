import sys
import wpilib
import logging
from math import *
from networktables import NetworkTable
from networktables.util import *

#----------------------------------------------------------------------------------
def clamp(n, low, high):
    return max(low, min(n, high))

#----------------------------------------------------------------------------------
class MyRobot(wpilib.IterativeRobot):
    #: update every 0.005 seconds/5 milliseconds (200Hz)
    kUpdatePeriod = 0.005
    
    def robotInit(self):
        self.robot_drive = wpilib.RobotDrive(0,1,2,3)
        self.maxspeed    = .7
        
        # joystick
        self.stick       = wpilib.Joystick(0)
        self.num_buttons = self.stick.getButtonCount();
        print('num buttons: ', self.num_buttons)
        sys.stdout.flush()

        logging.basicConfig(level=logging.DEBUG)         #to see messages from networktables
        self.raspi = NetworkTable.getTable('Pi')

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.auto_loop_counter = 0
        self.maxspeed = 1
        self.raspi_control = False

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous.
        
        Both outputMagnitude and curve are -1.0 to +1.0 values, where 0.0
        represents stopped and not turning. ``curve < 0`` will turn left and ``curve > 0``
        will turn right.
        
        drive(outputMagnitude, curve)
                :param outputMagnitude: The speed setting for the outside wheel in a turn,
                                        forward or backwards, +1 to -1.
                :param curve:           The rate of turn, constant for different forward speeds. Set
                                        ``curve < 0`` for left turn or ``curve > 0`` for right turn.
        """
        if self.raspi_control:
            pass
        else:
            self.auto_loop_counter += 1
        
            # Check if we've completed 100 loops (approximately 2 seconds)
            if self.auto_loop_counter < 100:
                self.robot_drive.drive(-0.4, 0) # Drive forwards at half speed
            elif self.auto_loop_counter < 200:
                self.robot_drive.drive(-0.4, 1) # Drive left at half speed
            elif self.auto_loop_counter < 300:
                self.robot_drive.drive(-0.4, 0) # Drive forwards at half speed
            else:
                self.robot_drive.drive(0, 0)    # Stop robot

    def stickDrive(self):
        self.robot_drive.arcadeDrive(clamp(-self.stick.getY(), -self.maxspeed, self.maxspeed),
                                     clamp(-self.stick.getX(), -self.maxspeed, self.maxspeed))

    def teleopInit(self):
        self.raspi_control = True
        self.auto_loop_counter = 0

    def teleopPeriodic(self): 
        """This function is called periodically during operator control."""
        if self.raspi_control:
            try:
                print('piTime: ', self.raspi.getNumber('piTime'))
                self.raspi.putNumber('robotTime', self.auto_loop_counter)
            except KeyError:
                print("piTime: n/a")
            sys.stdout.flush()
        else:
            self.stickDrive()
            
        self.auto_loop_counter += 1

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

#----------------------------------------------------------------------------------
if __name__ == "__main__":
    wpilib.run(MyRobot)
