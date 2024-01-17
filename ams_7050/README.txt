
Description of directory structure
----------------------------------
- doc: Documentation

- sample_code: Sample code

- bin: Pre-compiled binaries of the sample application for win_x86_64

- as7050-app-manager: AS7050 Application manager, handles the Bio Applications. Contains the code for the GSR application.

- as7050-chip-lib: AS7050 Chip Library, handles the register interface to the sensor via I2C.
                    src / inc: Chip Library Sources
                    osal: Operating System Abstraction Layer, used to communicate with the AS7050 EVK

- bio-app-handler: Code for the Bio applications HRM and SpO2. Used by AS7050 Application manager

- vitalsigns-algorithms: Underlying algorithms for calculating HRM and SpO2 values from the AS7050 raw data, used by the Bio App Handler
  (binaries are saved inside bio-app-handler to have all binaries in one directory)

- css-sw-utilities: various helper libraries specific to this AS7050 EVK implementation

- vital-signs-acc: Accelerometer interface. This component includes an implementation of this interface for LIS2DH12.

- as7050-linux-driver: AS7050 Linux sample driver

Getting Started
----------------
Please refer to the 'AS7050 Software Support Package User Guide'
(doc/AS7050_Software_Support_Package-User_Guide-v1.3.pdf)
