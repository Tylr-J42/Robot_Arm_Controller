import pigpio
import time

pi = pigpio.pi()

pi.set_mode(12, pigpio.OUTPUT)

pi.set_PWM_frequency(12, 50)
pi.set_PWM_dutycycle(12, 90/180+2.5)
time.sleep(4)