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
        # 2 - back left 
        # 3 - front left
        # 0 - back right
        # 1 - front right
        self.robot_drive = wpilib.RobotDrive(0,1,2,3); self.robot_drive.setSafetyEnabled(False)
        self.shooter     = wpilib.TalonSRX(6)
        self.robot_shoot = wpilib.RobotDrive(4,5);     self.robot_shoot.setSafetyEnabled(False)
        self.servo       = wpilib.Servo(7);
        #self.robot_shoot.setInvertedMotor(2, True)
        #self.robot_shoot = wpilib.
        #self.robot_pitch = wpilib.RobotDrive(6,7); self.robot_pitch.setSafetyEnabled(False)
        
        self.maxspeed    = 1
        
        # joystick
        self.stick       = wpilib.Joystick(0)
        self.num_buttons = self.stick.getButtonCount();
        print('num buttons: ', self.num_buttons)

        logging.basicConfig(level=logging.DEBUG)         #to see messages from networktables
        self.raspi = NetworkTable.getTable('Pi')

        self.shoot_loop_counter = -1000000
        self.triggerDepressedLastFrame = False

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

    def stickDrive(self, c):

        power = 1 #if c % 4 < 2 else .5
        shouldActivateServo = False;

        if getTrigger():
            self.shoot_loop_counter = self.auto_loop_counter
            self.robot_shoot.arcadeDrive(1, 0)

        if (self.auto_loop_counter - self.shoot_loop_counter < 300 and !getTrigger()):
            self.servo.set(0)
            self.robot_shoot.arcadeDrive(1, 0)
        else:
            self.servo.set(1)
            self.robot_shoot.arcadeDrive(0, 0)

        if self.stick.getRawButton(2): self.robot_shoot.arcadeDrive(-1, 0)


        self.shooter.set(power/2 if self.stick.getRawButton(5) else (-power if self.stick.getRawButton(3) else 0))  #adjust height of shoot thingy

        self.robot_drive.arcadeDrive(clamp(-self.stick.getY(), -self.maxspeed, self.maxspeed),
                                     clamp(-self.stick.getX(), -self.maxspeed, self.maxspeed))

        
        #self.robot_shoot.arcadeDrive(-1 if self.stick.getRawButton(7) else (1 if self.stick.getRawButton(8) else 0), 0)

    def teleopInit(self):
        self.raspi_control = False
        self.auto_loop_counter = 0
        self.errorReached = False;

    def teleopPeriodic(self): 
        """This function is called periodically during operator control."""
        
        if self.raspi_control:
            try:
                print('piTime: ', self.raspi.getNumber('piTime'))
                self.raspi.putNumber('robotTime', self.auto_loop_counter)
            except KeyError:
                if not self.errorReached: 
                    print("piTime could not be retrieved from table. Is the pi connected?") 
                    self.errorReached = True;
        else:
            self.stickDrive(self.auto_loop_counter)
            
        self.auto_loop_counter += 1

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

#----------------------------------------------------------------------------------
if __name__ == "__main__":
    wpilib.run(MyRobot)
