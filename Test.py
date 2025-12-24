import lgpio
import time

h = lgpio.gpiochip_open(4)
lgpio.gpio_claim_output(h, 23)
lgpio.gpio_claim_output(h, 24)

# for 23(high torque) open: 0/180*10+2.5
# for 24 open: 185/180*10+2.5
# for 23(high torque) closed: 180/180*10+2.5
# for 24 closed: 0/180*10+2.5
lgpio.tx_pwm(h, 23, 50, 80/180*10+2.5)
time.sleep(10)

lgpio.gpiochip_close(h)