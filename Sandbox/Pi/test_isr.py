import gpiozero as gpio
import time

LED = gpio.LED(17)

while 1:
    LED.on()
    print('on')
    time.sleep(5)
    LED.off()
    print('off')
    time.sleep(5)
