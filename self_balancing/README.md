# MPU6050 complimentary filter Example

 
* This example will show you how to use MPU6050 module and display calculater roll/pitch calculated using complimentary filter.
 
    * read external sensor, here we use a MPU6050 for instance.

  
* Pin assignment:
 
    * master:
        * GPIO18 is assigned as the data signal of i2c master port(ESP32)
        * GPIO19 is assigned as the clock signal of i2c master port(ESP32)
 
* Connection:
 
    * connect sda/scl of MPU6050 with GPIO18/GPIO19
    * no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 
* Test items:
 
    * read the Acce, Gyro data from MPU6050.
    * i2c master(ESP32) will write data from MPU6050.
