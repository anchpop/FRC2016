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
    

    def initializeCamera(self):
        self.camera = wpilib.USBCamera()
        self.camera.setExposureManual(50)
        self.camera.setBrightness(80)
        self.camera.updateSettings() # force update before we start thread

        self.server = wpilib.CameraServer.getInstance()
        self.server.startAutomaticCapture(self.camera)

    #def initializeTalon(self):
        

    def robotInit(self):
        # 2 - back left 
        # 3 - front left
        # 0 - back right
        # 1 - front right
        self.robot_drive = wpilib.RobotDrive(0,1,2,3); self.robot_drive.setSafetyEnabled(False)
        self.shooter     = wpilib.CANTalon(6)
        self.robot_shoot = wpilib.RobotDrive(4,5);     self.robot_shoot.setSafetyEnabled(False)
        self.servo       = wpilib.Servo(7);
        #self.robot_shoot.setInvertedMotor(2, True)
        #self.robot_shoot = wpilib.
        #self.robot_pitch = wpilib.RobotDrive(6,7); self.robot_pitch.setSafetyEnabled(False)

        self.maxspeed    = 1


        logger = logging.getLogger("robot")
        self.logger.info("Starting")
        
        # joystick
        self.stick       = wpilib.Joystick(0)
        self.num_buttons = self.stick.getButtonCount();

        logging.basicConfig(level=logging.DEBUG)         #to see messages from networktables
        self.raspi = NetworkTable.getTable('Pi')

        #initializeCamera()

        #initializeTalon()
        #self.shooter.changeControlMode(1); #Change control mode of talon, default is PercentVbus (-1.0 to 1.0). 1 is position, which is what we want
        self.shooter.setFeedbackDevice(0); #0 is quad controller
        self.shooter.setPID(150, 50, 50); #Set the PID constants (p, i, d)
        self.shooter.enableControl(); #Enable PID control on the talon
        self.shooter.reverseOutput(-1);

    def pLoop(target, p, bias, real):
        error = real - target
        return clamp(p * error + bias, -1, 1)

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

        power = .5 #if c % 4 < 2 else .5
        shouldActivateServo = False;

        if self.stick.getTrigger():
            self.shoot_loop_counter = self.auto_loop_counter
            self.robot_shoot.arcadeDrive(1, 0)

        elif (self.auto_loop_counter - self.shoot_loop_counter < 100):
            self.servo.set(0)
            self.robot_shoot.arcadeDrive(1, 0)
        else:
            self.servo.set(1)
            self.robot_shoot.arcadeDrive(0, 0)

        if self.stick.getRawButton(2): self.robot_shoot.arcadeDrive(-1, 0)


        #self.shooter.set(power if self.stick.getRawButton(5) else (-power/2 if self.stick.getRawButton(3) else 0))  #adjust height of shoot thingy
        
        #self.shooter.set(300)
        self.shooter.set(pLoop(300, .005, .2, self.shooter.getEncPosition()))
        self.robot_drive.arcadeDrive(clamp(-self.stick.getY(), -self.maxspeed, self.maxspeed),
                                     clamp(-self.stick.getX(), -self.maxspeed, self.maxspeed))


        currentPosition = self.shooter.getEncPosition();
        self.logger.info(currentPosition)

        
        #self.robot_shoot.arcadeDrive(-1 if self.stick.getRawButton(7) else (1 if self.stick.getRawButton(8) else 0), 0)

    def teleopInit(self):
        self.raspi_control = False
        self.auto_loop_counter = 0
        self.errorReached = False;


        self.shoot_loop_counter = -1000000
        self.triggerDepressedLastFrame = False

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
