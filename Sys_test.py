import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)

GPIO.setup(12, GPIO.OUT)

p = GPIO.PWM(12, 50)

p.start(0)
p.ChangeDutyCycle(0/18+2.5)
time.sleep(3)
p.ChangeDutyCycle(90/18+2.5)
time.sleep(3)
p.stop()