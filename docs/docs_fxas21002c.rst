.. module:: fxas21002c

*****************
FXAS21002C Module
*****************

This module contains the driver for NXP FXAS21002C gyroscope for measuring the angular velocity.
The FXAS21002C provides direct I2C communication and can be set on 4 different full-scale range and 8 different over sample rate values  (`datasheet <http://www.nxp.com/assets/documents/data/en/data-sheets/FXAS21002.pdf>`_).
    
.. class:: FXAS21002C(i2cdrv, addr=0x20, clk=400000)

    Creates an intance of a new FXAS21002C.

    :param i2cdrv: I2C Bus used '( I2C0, ... )'
    :param addr: Slave address, default 0x20
    :param clk: Clock speed, default 400kHz

    Example: ::

        from nxp.fxas21002c import fxas21002c

        ...

        fxas = fxas21002c.FXAS21002C(I2C0)
        fxas.start()
        fxas.init()
        gyro = fxas.get_gyro()

    
.. method:: init(fsr=0, odr=2, fs_exp=0)

        Initialize the FXAS21002C setting the mode value.

        :param fsr: select the full-scale range (from 0 to 3 - refer to page 39 of the FXAS21002C datasheet), default 0
        :param odr: set the over sample rate (from 0 to 7 - refer to page 46 of the FXAS21002C datasheet), default 2
        :param fs_exp: Full-scale range expansion enable (allowed values 0,1 - refer to page 49 of the FXAS21002C datasheet), default 0

========= ============ ==================
FSR Value FS_EXP Value Degrees per Second
========= ============ ==================
0         0            2000 dps          
1         0            1000 dps          
2         0            500  dps          
3         0            250  dps          
0         1            4000 dps          
1         1            2000 dps          
2         1            1000 dps          
3         1            500  dps          
========= ============ ==================

========= =========
ODR Value Frequency
========= =========
0         800  Hz
1         400  Hz
2         200  Hz
3         100  Hz
4         50   Hz
5         25   HZ
6         12.5 Hz
7         12.5 Hz
========= =========
        
.. method:: get_raw_gyro()

        Retrieves the current gyroscope data reading as a tuple of X, Y, Z, raw values 

        Returns [gyro_x, gyro_y, gyro_z]

        
.. method:: get_raw_int_temp()

        Retrieves the current device internal temperature data as raw value.

        Returns raw_t

        
.. method:: get_gyro(axis=None)

        Retrieves the current gyriscope data in degrees per second reading as a tuple of X, Y, Z values or single axis value if axis argument is provided.

        :param axis: select the axis (allowed values: "x" for x-axis, "y" for y-axis, "z" for z-axis); default None

        Returns [gyro_x, gyro_y, gyro_z] or gyro_x or gyro_y or gyro_z

        
.. method:: get_int_temp(unit="C")

        Retrieves the current device internal temperature calibrate value in Celtius, Kelvin or Fahrenheit degrees.

        :param unit: select the unit of measure for internal temperature (allowed values: "C" for Celtius degrees, "K" for Kelvin degrees, "F" Fahrenheit degrees); default "C"

        Returns int_temp

        
