This is the official Repository for SRA WALL-E 2.1

Instructions for setup :

1)Copy the SRA folder to esp_idf/components/

Instructions for the build process :

1)Go to the project directory and type 'make -j8' to build the source files
2)Type 'make menuconfig' to configure settings like USB-PORT, Baud Rate and example related configurations
3)Type 'make flash' after connecting the esp32, to flash the code into the device
4)Use 'make monitor' to check the Serial Monitor output



