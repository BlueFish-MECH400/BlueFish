import gpiozero as gpio
import time

LED = gpio(17)

while 1:
    LED.on()
    sleep(5)
    LED.off()
    sleep(5)
