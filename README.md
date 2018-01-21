# wavin-controller
A controller for Wavin AHC 9000 written in Python with focus for use in home-automation.

Example
```python
    # USB to RS485 converter at /dev/ttyUSB0
    # Default modbus id is 0x01
    wavin = WavinControl('/dev/ttyUSB0', 0x01)

    print(wavin.Sensor.temp_air(0))
    23.3
    print(wavin.Sensor.temp_air(1))
    22.7
```
