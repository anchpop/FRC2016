from Adafruit_PWM_Servo_Driver import PWM

servo = PWM(0x40)
servo.setPWMFreq(50)

TILT = 0
PAN  = 1

def pan(deg):
    _turn(PAN, deg)

def tilt(deg):
    _turn(TILT, deg)

def _turn(channel, deg):
    ## freq of 50hz => each cycle lasts 1 / 50 = 20 milliseconds
    ## servo neutral pos at 1.5ms => pwm value = 4096 * 1.5 / 20 = 307 out of 4096
    ## 1 ms => 205
    ## 2 ms => 410
    ## doc for TowerPro SG90 give pulse .5ms to 2.4ms  => 102 to 491 out of 4096
    ## we use                                          => 116 to 464 out of 4096
    ## ------------------------------------------------------------------------
    pwm = 116.0 + (deg / 180.0) * 348.0
    pwm = int(pwm)

    # pwm should be a value between 0 and 4095
    servo.setPWM(channel, 0, pwm)
