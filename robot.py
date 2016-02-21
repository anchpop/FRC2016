import wpilib
from math import *

class MyRobot(wpilib.IterativeRobot):
    
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.robot_drive_left = wpilib.RobotDrive(0,2,1,3)
        #self.robot_drive_right = wpilib.RobotDrive(3,4)
        self.stick = wpilib.Joystick(0)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.auto_loop_counter = 0
        self.maxspeed = 1

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.auto_loop_counter += 1
        
        # Check if we've completed 100 loops (approximately 2 seconds)
        if self.auto_loop_counter < 100:
            self.robot_drive_left.drive(-0.4, 0) # Drive forwards at half speed
            #self.robot_drive_right.drive(-0.5, 0) # Drive forwards at half speed
        elif self.auto_loop_counter < 200:
            self.robot_drive_left.drive(-0.4, 1) # Drive forwards at half speed
        elif self.auto_loop_counter < 300:
            self.robot_drive_left.drive(-0.4, 0) # Drive forwards at half speed
        else:
            #self.robot_drive_right.drive(0, 0)    #Stop robot
            self.robot_drive_left.drive(0, 0)    #Stop robot


    def teleopInit(self):
        self.maxspeed = .7

    def teleopPeriodic(self): 
        """This function is called periodically during operator control."""
        self.robot_drive_left.arcadeDrive(max(min(-self.stick.getY(), self.maxspeed), -self.maxspeed), max(min(-self.stick.getX(), self.maxspeed), -self.maxspeed))#max(min(-self.stick.getX(), self.maxspeed), -self.maxspeed))

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

if __name__ == "__main__":
    wpilib.run(MyRobot)
