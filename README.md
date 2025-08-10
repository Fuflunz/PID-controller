# PID-controller
This Project yields from my Physics Bachelors-Thesis done at Ruhr-Universität Bochum where I coded a PID-controller to regulate Temperature cryostate.

The main code is written for a certain circuit, wich was also build by me. The PID-class however is usuable for any case, where a PID-controller is needed.
This means that estimated parameters and the general calculation need to be adjusted depending on the system it runs on.
Additionally I wrote a reactive Plot, wich plots the data from a csv file, where the measurement data is wirtten in.

The code was run on a Raspberry Pi 4, using a PT100 Temperature Sensor, a OPA549 Amplifier, an ADS1115 ADC and a MPC4725 DAC as the Main components.

The code runs but the structure is still unfinished and all comments are still in german. A fix/update is unlikely, but not impossible.
