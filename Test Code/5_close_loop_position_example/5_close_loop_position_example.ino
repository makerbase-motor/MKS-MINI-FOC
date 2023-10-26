/*
MKS MINI FOC Closed Loop Position Control Example; Test Library：SimpleFOC 2.1.1; Test Hardware：MKS MINI FOC V1.0
Enter "T+Position" in the serial monitor to make the two motors rotate in closed loop
For example, input the radian system "T3.14" to let the two motors rotate 180°
When using your own motor, do remember to modify the default number of pole pairs, the value in BLDCMotor()
The default power supply voltage set by the program is 12V
Please remember to modify the values in voltage_power_supply and voltage_limit variables if you use other voltages for power supply
The motor targeted by the default PID is the YT2804 motor. To use your own motor.
You need to modify the PID parameters to achieve better results.
*/

#include <SimpleFOC.h>


MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

//Motor Parameters
BLDCMotor motor = BLDCMotor(7);                               //According to pole pairs of the selected motor, modify the value of BLDCMotor() here
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);

BLDCMotor motor1 = BLDCMotor(7);                              //Also modify the value of BLDCMotor() here
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26,27,14,21);

//Command Settings
//Enter "T+Position" in the serial monitor to make the two motors rotate in closed loop
//For example, input the radian system "T3.14" to let the two motors rotate 180°
float target_velocity = 0;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  I2Cone.begin(19,18, 400000); 
  I2Ctwo.begin(23,5, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  //Connect the Motor Object with the Sensor Object
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //Supply Voltage Setting [V]
  driver.voltage_power_supply = 12;                   //According to the supply voltage, modify the value of voltage_power_supply here
  driver.init();

  driver1.voltage_power_supply = 12;                  //Also modify the value of voltage_power_supply here
  driver1.init();
  //Connect the Motor and Driver Objects
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  //FOC Model Selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //Motion Control Mode Settings
  motor.controller = MotionControlType::angle;
  motor1.controller = MotionControlType::angle;

  //Speed PID Setting                                     
  motor.PID_velocity.P = 0.1;             //According to the selected motor, modify the PID parameters here to achieve better results
  motor1.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1;
  motor1.PID_velocity.I = 1;
  //Angle PID Setting 
  motor.P_angle.P = 20;
  motor1.P_angle.P = 20;
  //Motor Maximum Limit Voltage
  motor.voltage_limit = 1;                //According to the supply voltage, modify the value of voltage_limit here
  motor1.voltage_limit = 1;               //Also modify the value of voltage_limit here
  
  //Speed Low-pass Filter Time Constant
  motor.LPF_velocity.Tf = 0.01;
  motor1.LPF_velocity.Tf = 0.01;

  //Maximum Velocity Limit Setting
  motor.velocity_limit = 20;
  motor1.velocity_limit = 20;

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  
  //Initialize the Motor
  motor.init();
  motor1.init();
  //Initialize FOC
  motor.initFOC();
  motor1.initFOC();
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  
}



void loop() {
  Serial.print(sensor.getAngle()); 
  Serial.print(" - "); 
  Serial.print(sensor1.getAngle());
  Serial.println();
  motor.loopFOC();
  motor1.loopFOC();

  motor.move(target_velocity);
  motor1.move(target_velocity);
  
  command.run();
}
