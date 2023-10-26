/**
MKS MINI FOC Motor Knob; Test Library：SimpleFOC 2.1.1; Test Hardware:MKS MINI FOC V1.0
Rotating one of the motors after power on can control the rotation speed of the other motor.
When using your own motor, remember to modify the default number of pole pairs，the value in BLDCMotor(7).
The default power supply voltage set by the program is 12V, please remember to modify the values 
in voltage_power_supply and voltage_limit variables if you use other voltages for power supply.
The motor targeted by the default PID is YT2804, and the encoder used is AS5600.
To use your own motor, you need to modify the PID parameters to achieve better results.
 */
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// Motor instance
BLDCMotor motor = BLDCMotor(7);                         //Modify the value in BLDCMotor() here according to the number of pole pairs of the selected motor
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(7);                        //Also modify the value in BLDCMotor() here
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 21);

//Define TROT gait variables
void setup() {
  I2Cone.begin(19, 18, 400000); 
  I2Ctwo.begin(23, 5, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  // driver configuration
  // Supply voltage [V]
  driver.voltage_power_supply = 12;               //The value of voltage_power_supply here needs to be modified according to the supply voltage
  driver.init();

  driver1.voltage_power_supply = 12;              //Also modify the value of voltage_power_supply here
  driver1.init();
  // connection driver
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  // Control loop mode setting
  motor.controller = MotionControlType::torque;
  motor1.controller = MotionControlType::velocity;

  // Motor voltage limit
  motor.voltage_limit = 12;                 //Modify the value of voltage_limit here according to the supply voltage
  motor1.voltage_limit = 12;                //Also modify the value of voltage_limit here
  
  motor1.LPF_velocity.Tf = 0.01;            //Modify these two lines of parameters according to the selected motor to achieve better results
  motor1.PID_velocity.I = 1;

  // Serial port settings
  Serial.begin(115200);
  // The following two lines are not needed and can be commented out
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);
  //Record brushless initial position

  
  //Initialize the motor
  motor.init();
  motor1.init();
  motor.initFOC();
  motor1.initFOC();


  Serial.println("Motor ready.");
  _delay(1000);
  
}

void loop() {

  motor.loopFOC();
  motor1.loopFOC();

  motor.move(5*(motor1.shaft_velocity/10 - motor.shaft_angle));
  motor1.move(10*dead_zone(motor.shaft_angle));
}

float dead_zone(float x){
  return abs(x) < 0.2 ? 0 : x;
}
