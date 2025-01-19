from math import pi
from machine import Pin, PWM
import micropython
import time
micropython.alloc_emergency_exception_buf(200)

# Declare global variables
ena = None
in1 = None
in2 = None
enb = None
in3 = None
in4 = None
sensora = None
sensorb = None
pulseCountA = 0
pulseCountB = 0
wheelCircumference = 20.42035  # Example: in centimeters
pulsesPerRevolution = 37  # Number of pulses per wheel revolution
targetDistance = 0
kP = 8.0  # Proportional gain
kD = 0.5  # Derivative gain
lastError = 0
lastPulseTimeA = 0
lastPulseTimeB = 0
debounceInterval = 200  # Microseconds
pwm_ena = None
pwm_enb = None

def setup():
    global ena, in1, in2, enb, in3, in4, sensora, sensorb
    global pwm_ena, pwm_enb

    # Initialize Pins
    ena = Pin(5, Pin.OUT)
    pwm_ena = PWM(ena, freq=5000, duty=0)
    in1 = Pin(6, Pin.OUT)
    in2 = Pin(7, Pin.OUT)
    enb = Pin(8, Pin.OUT)
    pwm_enb = PWM(enb, freq=5000, duty=0)  # 5 kHz frequency, initial duty cycle 0
    in3 = Pin(9, Pin.OUT)
    in4 = Pin(10, Pin.OUT)
    sensora = Pin(3, Pin.IN, Pin.PULL_UP)
    sensorb = Pin(4, Pin.IN, Pin.PULL_UP)
    
    # Attach interrupt handlers
    sensora.irq(trigger=Pin.IRQ_RISING, handler=sensorAinterupt)
    sensorb.irq(trigger=Pin.IRQ_RISING, handler=sensorBinterupt)

def sensorAinterupt():
    global lastPulseTimeA, pulseCountA
    currenttime = time.ticks_us()
    if currenttime - lastPulseTimeA > debounceInterval:
        pulseCountA += 1
        lastPulseTimeA = currenttime

def sensorBinterupt():
    global lastPulseTimeB, pulseCountB
    currenttime = time.ticks_us()
    if currenttime - lastPulseTimeB > debounceInterval:
        pulseCountB += 1
        lastPulseTimeB = currenttime

def setMotorSpeed(motor: float, speed: float):
    speed = int(speed)
    if motor == 1:
        pwm_ena.duty(speed)
    elif motor == 2:
        pwm_enb.duty(speed)

def forward(distance, basespeed: int):
    global pulseCountA, pulseCountB, targetDistance, lastError

    targetDistance = distance
    pulseCountA = 0
    pulseCountB = 0

    setMotorSpeed(1, basespeed)
    setMotorSpeed(2, basespeed)

    # Set to drive forward
    in1.on()
    in2.off()
    in3.on()
    in4.off()

    lastError = 0

    while True:
        distanceA = (pulseCountA / pulsesPerRevolution) * wheelCircumference
        distanceB = (pulseCountB / pulsesPerRevolution) * wheelCircumference
        error = distanceA - distanceB
        correction = (error * kP) + ((error - lastError) * kD)

        newspeedA = basespeed - correction
        newspeedB = basespeed + correction

        setMotorSpeed(1, newspeedA)
        setMotorSpeed(2, newspeedB)

        lastError = error

        if distanceA >= targetDistance and distanceB >= targetDistance:
            stopmotor()
            break

def stopmotor():
    setMotorSpeed(1, 0)
    setMotorSpeed(2, 0)
    in1.off()
    in2.off()
    in3.off()
    in4.off()

def turn(angle:int, baseSpeed:int):
    lastDistanceA = 11.0 #maybe supposed to be zero?
    lastDistanceB = 11.0
    calibrationFactor = 0.85 # adjust to calibrate the turn
    turnCirumfrence = wheelCircumference * pi
    targetDistance = ((abs(angle) / 360.0)* turnCirumfrence) / 2
    targetDistance *= calibrationFactor
    pulseCountA = 0
    pulseCountB = 0
    while True:
        distanceA = (pulseCountA / pulsesPerRevolution) * wheelCircumference
        distanceB = (pulseCountB / pulsesPerRevolution) * wheelCircumference
        error = distanceA - distanceB if angle > 0 else distanceB - distanceA
        P = error * kP
        D = (error - lastError) * kD
        correction = P + D
        newspeedA = baseSpeed - correction
        newspeedB = baseSpeed + correction
        setMotorSpeed(1, newspeedA)
        setMotorSpeed(2, newspeedB)
        if angle > 0:
            in1.off()
            in2.on()
            in3.on()
            in4.off()
        else:
            in1.on()
            in2.off()
            in3.off()
            in4.on()
        lastError = error
        lastDistanceA = distanceA
        lastDistanceB = distanceB
        if distanceA >= targetDistance and distanceB >= targetDistance:
            stopmotor()
            break
def backward(speed: int):
    setMotorSpeed(1, speed)
    setMotorSpeed(2, speed)
    in1.on()
    in2.off()
    in3.on()
    in4.off()