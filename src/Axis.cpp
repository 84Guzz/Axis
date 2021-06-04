#include <Arduino.h>
#include <Axis.h>
#include <DCMotor.h>

void Axis::begin(AxisParam parameters){
    
    //DC motor specific pins
    _motorParam.enaPin = parameters.enaPin;
    _motorParam.enbPin = parameters.enbPin;
    _motorParam.pwmPin = parameters.pwmPin;
    _motorParam.startStopDelay = 0;
    
    //Initialize DC motor
    _motor.begin(_motorParam);

    //Axis specific settings
    //Ramps and speeds
	if (parameters.ramp > 0){
		_startRamp = parameters.ramp;
		_stopRamp = parameters.ramp;
	} else {
		_startRamp = parameters.startRamp;
		_stopRamp = parameters.stopRamp;		
	}
    _maxSpeed = parameters.maxSpeed * 1000;
    _homeSpeed = parameters.homeSpeed;
    if (_startRamp > 0) {
        _acc = _maxSpeed / _startRamp;
        _maxAccDist = (5 * _acc * sq(_startRamp / 10)) / 100;
    }
    if (_stopRamp > 0) {
        _dec = _maxSpeed / _stopRamp;
        _maxDecDist = (5 * _dec * sq(_stopRamp / 10)) / 100;
    }

    //Limits
    _limitA = parameters.limitA * 1000;
    _limitB = parameters.limitB * 1000;

    // Serial.print("Start ramp: ");
    // Serial.println(_startRamp); 
    // Serial.print("Stop ramp: ");
    // Serial.println(_stopRamp);
    // Serial.print("Acceleration: ");
    // Serial.println(_acc);       
    // Serial.print("Max acceleration distance: ");
    // Serial.println(_maxAccDist);
    // Serial.print("Deceleration: ");
    // Serial.println(_dec);       
    // Serial.print("Max deceleration distance: ");
    // Serial.println(_maxDecDist);

    //Initialize state machine
    _state = AXIS_IDLE;

    //Initialize position
    _position = 0;

}

void Axis::home(bool Direction = false){

    _cmdHomeA = Direction;
    _cmdHomeB = !Direction;

}

void Axis::home(unsigned long Speed, bool Direction = false){

    _homeSpeed = Speed;

    _cmdHomeA = Direction;
    _cmdHomeB = !Direction;


}

void Axis::moveAbs(long Target){

    //Set target position
    _target = Target * 1000;

    //Calculate profile
    _calculateProfile();

    //Set commands
    if (_accTime > 0 && _decTime > 0) {
        _cmdMoveA = _distance > 0;
        _cmdMoveB = _distance < 0;
    }
    
}

void Axis::moveRel(long Distance){

    //Calculate target position
    _target = _position + (Distance * 1000);

    //Calculate profile
    _calculateProfile();

    //Set commands
    if (_accTime > 0 && _decTime > 0) {
        _cmdMoveA = _distance > 0;
        _cmdMoveB = _distance < 0;
    }    

}

void Axis::update(){

    _update();

}

void Axis::update(bool limitSwA, bool limitSwB){

    _limitSwA = limitSwA;
    _limitSwB = limitSwB;

    _update();

}

void Axis::_update(){

    //Update motor
    _motor.update();
 
    //Transitions
    if((_state == AXIS_IDLE || _state == AXIS_ON_POS) && (_cmdMoveA || _cmdMoveB)) _nextState = AXIS_ACCELERATING;
    if((_state == AXIS_IDLE || _state == AXIS_ON_POS) && (_cmdHomeA || _cmdHomeB)) _nextState = AXIS_HOMING;
    if(_state == AXIS_HOMING && !(_cmdHomeA || _cmdHomeB)) _nextState = AXIS_ON_POS;
    if(_state == AXIS_HOMING && _stateTimer >= _cvTime) _nextState = AXIS_QUICK_STOP;
    if(_state == AXIS_ACCELERATING && _stateTimer >= _accTime && _cvTime > 0) _nextState = AXIS_CONSTANT_VELOCITY;
    if(_state == AXIS_ACCELERATING && _stateTimer >= _accTime && _cvTime == 0) _nextState = AXIS_DECELERATING;
    if(_state == AXIS_CONSTANT_VELOCITY && _stateTimer >= _cvTime) _nextState = AXIS_DECELERATING;
    if(_state == AXIS_DECELERATING && _stateTimer >= _decTime) _nextState = AXIS_ON_POS;
    if((_state >= AXIS_ACCELERATING && _state <= AXIS_DECELERATING) && (_limitSwA || _limitSwB)) _nextState = AXIS_QUICK_STOP;   

	//State machine
	_state = _nextState;
	_newState = _state != _prevState;
	if (_newState) _stateTimer = 0; 

    //AXIS HOMING
    if (_state == AXIS_HOMING){

        //State entry
        if (_newState){
            
            //Calculate setpoint
            _homeSetpoint = (_homeSpeed * 255) / (_maxSpeed / 1000);

            //Calculate maximum state time 2 * (limitA - limitB) / home speed
            _cvTime = 2 * ((_limitA - _limitB) / (_homeSpeed));
            
            if (_cmdHomeA) _motor.cmdA(100, _startRamp);
            if (_cmdHomeB) _motor.cmdB(100, _startRamp);

        }

        if (_cmdHomeA && _limitSwA){
            _motor.cmdStop(0);
            _position = _limitA;
            _cmdHomeA = false;
        }  
        if (_cmdHomeB && _limitSwB){
            _motor.cmdStop(0);
            _position = _limitB;
            _cmdHomeB = false;
        }

    }

    //AXIS ACCELERATING
    if (_state == AXIS_ACCELERATING){

        //State entry
        if (_newState){

            //Start motor
            if (_cmdMoveA) _motor.cmdA(255, _startRamp);
            if (_cmdMoveB) _motor.cmdB(255, _startRamp);   

            //Save start position
            _startPosition = _position;

        }
        //Continuous
        if (_cmdMoveA) _position = _startPosition + ((5 * (_acc/10) * sq(_stateTimer)) / 1000);
        if (_cmdMoveB) _position = _startPosition - ((5 * (_acc/10) * sq(_stateTimer)) / 1000);
    }

    //AXIS CONSTANT VELOCITY
    if (_state == AXIS_CONSTANT_VELOCITY){

         //State entry
        if (_newState){
            
            //Calculate position at start of state
            if (_cmdMoveA) _startPosition = _startPosition + (5 * (_acc / 10) * sq(_accTime)) / 1000;
            if (_cmdMoveB) _startPosition = _startPosition - (5 * (_acc / 10) * sq(_accTime)) / 1000;

        }

        //Continuous
        _position = _startPosition + (_maxSpeed / 1000 * (_stateTimer));

    }

    //AXIS DECELERATING
    if (_state == AXIS_DECELERATING){

         //State entry
        if (_newState){

            //Stop motor
            _motor.cmdStop(_stopRamp);

            //Calculate position
            if (_cmdMoveA){
                _startPosition = _startPosition + ((5 * (_acc / 10) * sq(_accTime)) / 1000);
                if(_cvTime > 0) _position = _position + (_maxSpeed / 1000 * (_cvTime));
            }
            if (_cmdMoveB){
                 _startPosition = _startPosition - ((5 * (_acc / 10) * sq(_accTime)) / 1000);
                if(_cvTime > 0) _position = _position - (_maxSpeed / 1000 * (_cvTime));               
            }

        }
        //Continuous
        if (_cmdMoveA) _position = _startPosition + ((5 * (_dec/10) * sq(_stateTimer)) / 1000);
        if (_cmdMoveB) _position = _startPosition - ((5 * (_dec/10) * sq(_stateTimer)) / 1000);

    }

    //AXIS ON POS
    if (_state == AXIS_ON_POS){

         //State entry
        if (_newState){

            //Set position
            _position = _target;

            //Reset commands
            _cmdMoveA = false;
            _cmdMoveB = false;

        }

    }

    //AXIS QUICK STOP
    if (_state == AXIS_QUICK_STOP){

         //State entry
        if (_newState){

            //Set position to limit
            if (_limitSwA && _cmdMoveA && _limitA != 2147483647) _position = _limitA;
            if (_limitSwB && _cmdMoveB && _limitB != -2147483648) _position = _limitB;

            //Reset commands
            _cmdMoveA = false;
            _cmdMoveB = false;

            //Stop motor
            _motor.cmdStop(0);

        }

    }

	//Save previous state
	_prevState = _state;

}

uint8_t Axis::getState(){

	return _state;

}

long Axis::getPosition(){

    return _position / 1000;
}

void Axis::_calculateProfile(){

    //Limit target
    _target = constrain(_target, _limitB, _limitA);

    //Calculate distance to travel
    _distance = _target - _position;

    //Determine profile: Triangular
    if (_maxAccDist + _maxDecDist >= abs(_distance)){

        //Calculate time spent accelerating
        _k = float(2 * _dec * abs(_distance) / 1000) / float(sq(_acc) + _acc * _dec);
        _accTime = round(sqrt(_k) * 1000.0);
        
        //Calculate time spent decelerating
        _k = float(2 * _acc * abs(_distance) / 1000) / float(sq(_dec) + _dec * _acc);
        _decTime = round(sqrt(_k) * 1000.0);

        //Constant velocity time = 0
        _cvTime = 0; 

    //Trapezoidal
    } else {

        //Acceleration and deceleration time = max
        _accTime = _startRamp;
        _decTime = _stopRamp;
        _cvTime = ((abs(_distance) - _maxAccDist - _maxDecDist) * 1000) / _maxSpeed; 

    }
    // Serial.print("Target: ");
    // Serial.print(_target);
    // Serial.println("u");    
    // Serial.print("Distance: ");
    // Serial.print(_distance);
    // Serial.println("u");
    // Serial.print("Acceleration time: ");
    // Serial.print(_accTime);
    // Serial.println("ms");
    // Serial.print("Constant velocity time: ");
    // Serial.print(_cvTime);
    // Serial.println("ms");    
    // Serial.print("Deceleration time: ");
    // Serial.print(_decTime);
    // Serial.println("ms");

}
