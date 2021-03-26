/*
 * BLDCDriver3PWM.cpp
 *
 *  Created on: 23 Mar 2021
 *  Ported from: https://github.com/simplefoc/Arduino-FOC/blob/v2.1/src/drivers/BLDCDriver3PWM.cpp
 */


#include "BLDCDriver3PWM.hpp"
#include "main.h"

BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en1) {//, int en2, int en3){
  // Pin initialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  // enable_pin pin
  enableA_pin = en1;
//  enableB_pin = en2;
//  enableC_pin = en3;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;

}

// enable motor driver
void  BLDCDriver3PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
	HAL_GPIO_WritePin(Enable_Pin8_GPIO_Port, Enable_Pin8_Pin, GPIO_PIN_SET);
    // set zero to PWM
    setPwm(0,0,0);
}

// disable motor driver
void BLDCDriver3PWM::disable()
{
  // set zero to PWM
	setPwm(0, 0, 0);
	// disable the driver - if enable_pin pin available
	HAL_GPIO_WritePin(Enable_Pin8_GPIO_Port, Enable_Pin8_Pin, GPIO_PIN_RESET);

}

// init hardware pins
int BLDCDriver3PWM::init() {
	  // a bit of separation
	  _delay(1000);

	//  // PWM pins
	//  pinMode(pwmA, OUTPUT);
	//  pinMode(pwmB, OUTPUT);
	//  pinMode(pwmC, OUTPUT);
	//  if( _isset(enableA_pin)) pinMode(enableA_pin, OUTPUT);


	  // sanity check for the voltage limit configuration
	  if(voltage_limit == NOT_SET || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

	  // Set the pwm frequency to the pins
	  // hardware specific function - depending on driver and mcu
	 // _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);
	  return 0;
}



//// Set voltage to the pwm pin
//void BLDCDriver3PWM::setPhaseState(int sa, int sb, int sc) {
//  // disable if needed
//  if( _isset(enableA_pin) &&  _isset(enableB_pin)  && _isset(enableC_pin) ){
//    digitalWrite(enableA_pin, sa == _HIGH_IMPEDANCE ? LOW : HIGH);
//    digitalWrite(enableB_pin, sb == _HIGH_IMPEDANCE ? LOW : HIGH);
//    digitalWrite(enableC_pin, sc == _HIGH_IMPEDANCE ? LOW : HIGH);
//  }
//}

// Set voltage to the pwm pin
void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {

  // limit the voltage in driver
  Ua = _constrain(Ua, 0.0, voltage_limit);
  Ub = _constrain(Ub, 0.0, voltage_limit);
  Uc = _constrain(Uc, 0.0, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0 , 1.0 );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0 , 1.0 );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0 , 1.0 );

  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle3PWM(dc_a, dc_b, dc_c, pwmA, pwmB, pwmC);
}
