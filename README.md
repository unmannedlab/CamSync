# Camera Synchronisation for PointGrey BFLY-PGE-20E4C

Playing around with triggers, ChunkData and Acquisition Modes for the BlackFly cameras. All code running on the Spinnaker Python 3.0.0 driver.

## Driver Installation
Install the Spinnaker SDK by running (check the readme if you need further details)

    sudo sh install_spinnaker.sh

Then install the Python Wrapper by running (check the readme if you need further details)

    python3.10 -m pip install --user spinnaker_python-3.0.0.118-cp38-cp38-linux_x86_64.whl

## Utilities

*Callib_Image_Collectors* contains files to capture images by a software trigger for camera callibration

*hardware_trigger_sync* contains examples to view Image timestamps to check synchronisation delays for both hardware and software trigger modes. Can be scaled to multiple cameras, currently tested on 2.

*external_hard_trigger* contains files to trigger the hardware trigger by yet another external trigger signal like an IMU or a GPS. Currently in development.