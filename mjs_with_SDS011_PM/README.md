Connected the SDS011 Particle Matter sensor to the MeetJeStad board. Pins used:<br>
- D6 - serial data from the PM (3.3V)<br>
- D7 - run/stop (TX is not used)<br>
<p>
Since there are now 2 serials (GPS and SDS011), SoftwareSerial was replaced by NeoSWserial.<br>
The PM sensor takes 10 measurements, of which the last half (5) are averaged for the result.<br>
An electronic circuit with 3 transistors and 3 resistors runs and stops the PM sensor.<br>
It is controlled by 3.3 V from the Arduino and sets the PM sensor supply to 5 V.<br>
For the mjs firmware v4 is used, but with the flags byte omitted.<br>
With DEBUG set to true, lots of logging is sent to the serial.<br>
