import RPi.GPIO as GPIO # type: ignore
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(40, GPIO.OUT)
GPIO.setup(38, GPIO.OUT)
GPIO.setup(37, GPIO.OUT)
GPIO.setup(36, GPIO.OUT)
GPIO.setup(35, GPIO.OUT)
GPIO.setup(33, GPIO.OUT)

count = 0
frequencyHz = 10000
prevtime = 0

try:
    print("starting 1000000 pulses")
    while count<=100000:
        current_time = time.perf_counter()

        if(current_time-prevtime >= 1/(frequencyHz*2)):
            if(count%2 == 1):
                GPIO.output(40, GPIO.HIGH)
                GPIO.output(38, GPIO.HIGH)
                GPIO.output(37, GPIO.HIGH)
                GPIO.output(36, GPIO.HIGH)
                GPIO.output(35, GPIO.HIGH)
                GPIO.output(33, GPIO.HIGH)
            else:
                GPIO.output(40, GPIO.LOW)
                GPIO.output(38, GPIO.LOW)
                GPIO.output(37, GPIO.LOW)
                GPIO.output(36, GPIO.LOW)
                GPIO.output(35, GPIO.LOW)
                GPIO.output(33, GPIO.LOW)

            count = count + 1
            prevtime = current_time

    print("ended")
    GPIO.cleanup()
except:
    print("stopped early")
    GPIO.cleanup()