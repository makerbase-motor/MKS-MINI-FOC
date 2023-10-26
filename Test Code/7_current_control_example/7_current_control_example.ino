
/**
MKS MINI FOC; FOC Current Control Example; Test Library：SimpleFOC 2.1.1; Test Hardware：MKS MINI FOC V1.0
Input in the serial port window: A+current control M0, B+current control M1, the current unit is A
The voltage limit and current limit set in the code can be changed or commented out
When using your own motor, please remember to modify the default number of pole pairs, that is, the value in BLDCMotor(7)
The default power supply voltage set by the program is 12V.
Please remember to modify the values in voltage_power_supply and voltage_limit variables if you use other voltages for power supply
The default PID parameters are aimed at the YT2804 motor. 
When using your own motor, you need to modify the PID parameters to achieve better results.
The speed limit can be modified according to the performance of the selected motor.
**/

#include <SimpleFOC.h>

//Motor Setting
BLDCMotor motor1 = BLDCMotor(7);                        //Modify the value in BLDCMotor() according to the selected motor
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);

BLDCMotor motor2 = BLDCMotor(7);                        //Also modify the value in BLDCMotor() here
BLDCDriver3PWM driver2  = BLDCDriver3PWM(26,27,14,21);

//Coder Setting
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);


// Current Detection Setting
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, 39, 36);
InlineCurrentSense current_sense2 = InlineCurrentSense(0.01, 50.0, 35, 34);

// Commander command Setting 
Commander command = Commander(Serial);
void doMotor1(char* cmd){ command.motor(&motor1, cmd); }
void doMotor2(char* cmd){ command.motor(&motor2, cmd); }

void setup() {
  // Coder Setting
  I2Cone.begin(19,18, 400000UL); 
  I2Ctwo.begin(23,5, 400000UL); 
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
  
  // According to the power supply voltage of the selected motor, modify the value of voltage_power_supply here
  // Drive Setting
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);

  //The voltage limit and current limit set in the code can be changed or commented out
  // Current Limit
    motor1.current_limit = 2;
    motor2.current_limit = 2;
  // Voltage Limit
    motor1.voltage_limit = 12;
    motor2.voltage_limit = 12;


  // Current Detection
  current_sense1.init();
  current_sense1.gain_b *= -1;
  current_sense1.gain_a *= -1;
//  current_sense1.skip_align = true;
  motor1.linkCurrentSense(&current_sense1);
  // current sense init and linking
  current_sense2.init();
  current_sense2.gain_b *= -1;
  current_sense2.gain_a *= -1;
//  current_sense2.skip_align = true;
  motor2.linkCurrentSense(&current_sense2);

  // Control Loop
  // Other Modes TorqueControlType::voltage TorqueControlType::dc_current 
  motor1.torque_controller = TorqueControlType::foc_current; 
  motor1.controller = MotionControlType::torque;
  motor2.torque_controller = TorqueControlType::foc_current; 
  motor2.controller = MotionControlType::torque;

  // FOC Current Control PID Parameters
  // According to the selected motor, modify the PID parameters here to achieve better results
   motor1.PID_current_q.P = 2;
   motor1.PID_current_q.I= 800;
   motor1.PID_current_d.P= 2;
   motor1.PID_current_d.I = 800;
   motor1.LPF_current_q.Tf = 0.002; // 1ms default
   motor1.LPF_current_d.Tf = 0.002; // 1ms default

   motor2.PID_current_q.P = 2;
   motor2.PID_current_q.I= 800;
   motor2.PID_current_d.P= 2;
   motor2.PID_current_d.I = 800;
   motor2.LPF_current_q.Tf = 0.002; // 1ms default
   motor2.LPF_current_d.Tf = 0.002; // 1ms default

    // Speed Loop PID Parameters
    // According to the selected motor, modify the PID parameters here to achieve better results
    motor1.PID_velocity.P = 0.1;
    motor1.PID_velocity.I = 1;
    motor1.PID_velocity.D = 0;

    motor2.PID_velocity.P = 0.1;
    motor2.PID_velocity.I = 1;
    motor2.PID_velocity.D = 0;
    // default voltage_power_supply
  
    // Speed Limit
    // The speed limit can be modified according to the performance of the selected motor
    motor1.velocity_limit = 40;
    motor2.velocity_limit = 40;


  // Monitor Interface settings
  Serial.begin(115200);
  // comment out if not needed
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // Monitor Related settings
  motor1.monitor_downsample = 0;
  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;
  motor2.monitor_downsample = 0;
  motor2.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;
  


  //Motor Initialization
  motor1.init();
  // align encoder and start FOC
  motor1.initFOC(); 
  
  motor2.init();
  // align encoder and start FOC
  motor2.initFOC(); 

  // Initial Target Value
  motor1.target = 0.05;
  motor2.target = 0.05;

  // Map Motors to Commander
  command.add('A', doMotor1, "motor 1");
  command.add('B', doMotor2, "motor 2");

  Serial.println(F("Double motor sketch ready."));
  
  _delay(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();

  // Iterative function setting the outter loop target
  motor1.move();
  motor2.move();

  // User Communication
  command.run();
  motor1.monitor();
  motor2.monitor();
}
