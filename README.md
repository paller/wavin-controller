# wavin-controller
A controller for Wavin AHC 9000 written in Python with focus on home-automation.

##### Example
```
    # USB to RS485 converter at /dev/ttyUSB0
    In [1]: wavin = WavinControl('/dev/ttyUSB0')

    In [2]: wavin.get_indexes()
    Out[2]: [0, 1, 2, 3, 4]

    In [3]: kitchen = wavin.sensor(3)

    In [4]: kitchen.temp_air
    Out[4]: 22.4

    In [5]: kitchen.battery
    Out[5]: 80
```
There you have it; room temperature 22.4°C and the sensor got 80% battery.