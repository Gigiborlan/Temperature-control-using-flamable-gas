# Temperature-control-using-flamable-gas

This system uses a PI controller with anti wind-up action to control the temperature of a gas oven. 
I connected a servo motor to the valve that controls the gas flow. 
For the temperature sensor I used a PT100 termoresistor with a Wheatstone bridge and INA128.
The microcontroller is a stmCortex F4 series.
The systm also has an MQ-02 sensor to identify gas leaks. In this case, the systems plays a sound through a buzzer of 30dB.

https://www.youtube.com/watch?v=f1_347bvGpM
