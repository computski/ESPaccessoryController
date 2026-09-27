# ESPaccessoryController
ESP based model railroad accessory controller supporting LocoNet
2026-09-27 still work in progress

Allows user to control 9 pins on an ESP8266 from Loconet to drive servos, aspects and MAS signals as well as configure pins as sensor inputs
Interworks with JMRI
Also supports I2C expansion into max 2 PCA9685 modules, giving a further 32 pins for servos or aspects
The unit connects over WiFi and is independent of the DCC track voltage, meaning shorts and shutdowns on the track do not prevent turnouts or signals from working

The ESP8266 nodeMCU ESP12 can be plugged into an L293 motor expansion board giving multiple 3-pin breakout connectors for SG90 servos.

Multiple such accessory controllers can be deployed under JMRI control, allowing literally hundreds of signals and turnouts to be supported at low cost.
