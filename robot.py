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

    def initTalon(self):
        if self.use_pid_loop:
            self.arm_tilt.changeControlMode(1);       # Change control mode of talon, default is PercentVbus
                                                      # (-1.0 to 1.0). 1 is position, which is what we want
            self.arm_tilt.setFeedbackDevice(0);       # 0 is quad controller
            self.arm_tilt.setPID(10.0, 0.00001, 0);   # Set the PID constants (p, i, d)
            self.arm_tilt.enableControl();            # Enable PID control on the talon
        else:
            self.arm_tilt.changeControlMode(0);  
        
    def robotInit(self):
        # 0 - back right
        # 1 - front right
        # 2 - back left 
        # 3 - front left
        self.robot_drive    = wpilib.RobotDrive(0,1,2,3); self.robot_drive.setSafetyEnabled(False)
        self.arm_tilt       = wpilib.CANTalon(6)
        self.shooter_wheels = wpilib.RobotDrive(4,5);     self.shooter_wheels.setSafetyEnabled(False)
        self.servo          = wpilib.Servo(7);
        #self.shooter_wheels.setInvertedMotor(2, True)

        self.arm_tilt.reverseOutput(True);
        self.current_pid   = 00
        self.pid_increment = 5
        
        self.maxspeed    = 1
        
        # joystick
        self.stick       = wpilib.Joystick(0)
        self.num_buttons = self.stick.getButtonCount();
        print('num buttons: ', self.num_buttons)

        logging.basicConfig(level=logging.DEBUG)         # to see messages from networktables
        self.raspi = NetworkTable.getTable('Pi')

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.auto_loop_counter = 0
        self.maxspeed = 1
        self.raspi_control = False
        self.use_pid_loop  = False
        self.initTalon()
        

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
            
            first     = 160
            second    = 75
            raisemode = 10
            third     = 70
            fourth    = 30
            fith      = 30
            
            if self.auto_loop_counter < first:
                self.robot_drive.drive(0.4, 0) # Drive forwards at half speed
            elif self.auto_loop_counter < first + second:
                self.robot_drive.drive(0.6, .8) # turn left/go forward at half speed
            elif self.auto_loop_counter < first + second + raisemode: #raise the arm
                self.robot_drive.drive(0, 0)
                self.arm_tilt.set(.6)
            elif self.auto_loop_counter < first + second + raisemode + third: #spin wheels
                self.robot_drive.drive(0, 0)
                self.arm_tilt.set(.1)
                self.shooter_wheels.arcadeDrive(-1, 0)
            elif self.auto_loop_counter < first + second + raisemode + third + fourth: #shoot ball
                self.shooter_wheels.arcadeDrive(-1, 0)
                self.servo.set(0)
                self.arm_tilt.set(.1)
            elif self.auto_loop_counter < first + second + raisemode + third + fourth + fith: #continue shoothing ball
                self.shooter_wheels.arcadeDrive(-1, 0)
            else:
                self.servo.set(1)
                self.robot_drive.drive(0, 0)    # Stop robot
                self.shooter_wheels.arcadeDrive(0, 0)

    def stickDrive(self, c):
        shouldActivateServo = False;

        if self.stick.getTrigger():
            # trigger depressed - spin shooter wheels in shooting direction
            # -------------------------------------------------------------
            self.shoot_loop_counter = self.auto_loop_counter
            self.shooter_wheels.arcadeDrive(-1, 0)

        elif (self.auto_loop_counter - self.shoot_loop_counter < 100):
            # trigger released for less than 1/2 second - shoot and keep spinning shooter wheels in shooting direction
            # ----------------------------------------------------------------------------------------------
            self.servo.set(0)
            self.shooter_wheels.arcadeDrive(-1, 0)
            
        else:
            #  1/2 second after trigger released - retract servo, and turn wheels off
            # --------------------------------------------------------------------------------------
            self.servo.set(1)
            self.shooter_wheels.arcadeDrive(0, 0)

        # button 2 => spin shooter wheels inwards to retrieve ball
        # --------------------------------------------------------
        if self.stick.getRawButton(2):
            self.shooter_wheels.arcadeDrive(1, 0)

        # adjust angle of shooter
        # -----------------------
        up_btn   = self.stick.getRawButton(3)
        down_btn = self.stick.getRawButton(5)
        if self.use_pid_loop:
            self.current_pid += -self.pid_increment if down_btn else (self.pid_increment if up_btn else 0)
            self.current_pid  = clamp(self.current_pid, 0, 750)
            self.arm_tilt.set(self.current_pid)
        else:
            self.arm_tilt.set(-0.5 if down_btn else (1.0 if up_btn else 0)) 

        # activate wheels according to joystick position
        # ----------------------------------------------
        self.robot_drive.arcadeDrive(clamp(-self.stick.getY(), -self.maxspeed, self.maxspeed),
                                     clamp(-self.stick.getX(), -self.maxspeed, self.maxspeed))

    def log(self, str):
        print(str)

    def teleopInit(self):
        self.raspi_control = False
        self.auto_loop_counter = 0
        self.errorReached = False;
        self.shoot_loop_counter = -1000000
        self.shoot = 0
        self.use_pid_loop = True
        self.initTalon()

    def teleopPeriodic(self): 
        """This function is called periodically during operator control."""
        self.raspi_control = self.num_buttons == 0 or self.stick.getTrigger()
            
        if self.raspi_control:
            try:
                # get movement instructions from rapsberry Pi
                # -------------------------------------------
                pan    = self.raspi.getNumber('pan')
                tilt   = self.raspi.getNumber('tilt')
                _shoot = self.raspi.getNumber('shoot')
                self.log('pan = %d, tilt = %d, shoot=%d ' % (pan, tilt, _shoot))
            except KeyError:
                if not self.errorReached: 
                    self.log("info could not be retrieved from NetworkTables. Is the pi connected?") 
                    self.errorReached = True;

        if self.raspi_control and not self.errorReached:
            if _shoot == 1 or self.shoot == 1:
                # we are correctly aligned - shoot
                # --------------------------------
                if self.shoot == 0:
                    self.shoot = 1
                    self.shoot_loop_counter = self.auto_loop_counter
                    self.shooter_wheels.arcadeDrive(-1, 0)
                elif (self.auto_loop_counter - self.shoot_loop_counter < 100):
                    # less than 1/2 second after shoot command - keep spinning shooter wheels in shooting direction
                    # ----------------------------------------------------------------------------------------------
                    self.shooter_wheels.arcadeDrive(-1, 0)
                elif (self.auto_loop_counter - self.shoot_loop_counter < 120):
                    #  1/2 second after shoot command - activate servo to shoot ball
                    # --------------------------------------------------------------
                    self.servo.set(0)
                    self.shooter_wheels.arcadeDrive(-1, 0)
                else:
                    # turn wheels off, and give control back to Alex
                    # ----------------------------------------------
                    self.servo.set(1)
                    self.shooter_wheels.arcadeDrive(0, 0)
                    self.shoot = 0
                    self.raspi_control = False
            else:
                # adjust pan/tilt so that we point to the target
                # ----------------------------------------------
                self.robot_drive.arcadeDrive(0, clamp(pan / 10, -0.8, 0.8))
        else:
            self.stickDrive(self.auto_loop_counter)
            
        self.auto_loop_counter += 1

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        wpilib.LiveWindow.run()

#----------------------------------------------------------------------------------
if __name__ == "__main__":
    wpilib.run(MyRobot)
