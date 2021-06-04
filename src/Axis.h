#ifndef Axis_h
#define Axis_h

#include <Arduino.h>
#include <inttypes.h>
#include <DCMotor.h>

#define AXIS_IDLE 0
#define AXIS_HOMING 1
#define AXIS_ACCELERATING 2
#define AXIS_CONSTANT_VELOCITY 3
#define AXIS_DECELERATING 4
#define AXIS_ON_POS 5
#define AXIS_ERROR 6
#define AXIS_QUICK_STOP 7

struct AxisParam {
	unsigned long startStopDelay = 0;
	unsigned long ramp = 0; //[ms -> 100%]
	unsigned long startRamp = 0; //[ms -> 100%]
	unsigned long stopRamp = 0; //[ms -> 100%]
	unsigned long positionMargin = 0; //[u]
	long limitA = 2147483647; //[u]
	long limitB = -2147483648; //[u]
	unsigned long maxSpeed = 0.0; // [u/s]
	unsigned long feedConstant = 0.0; // [u/rev]
	unsigned long breakAwaySpeed = 0.0; //[rev]
	unsigned long homeSpeed = 0.0; // [u/s]
	int8_t enaPin = -1;
	int8_t enbPin = -1;
	int8_t pwmPin = -1;
	int8_t diagPin = -1;
	int8_t csPin = -1;
};

class Axis{
public:

	//Constructors
	void begin(AxisParam  parameters);
	
	//Commands
	void home(bool Direction = false);
	void home(unsigned long Speed, bool Direction = false);
	void moveAbs(long Target);
	void moveRel(long Distance);
	void quickStop();
	
	//Update
	void update();
	void update(bool limitSwA, bool limitSwB);

	//Getters
	bool getInA();
	bool getInB();
	uint8_t getSpeed();
	uint8_t getState();
	long getPosition();
	bool positionWithinMargin();
	bool moving();

private:
	//DC Motor
	DCMotor _motor;
	DCMotorParam _motorParam;

	//State machine
	bool _newState = false;
	uint8_t _state = 0;
	uint8_t _nextState = 0;
	uint8_t _prevState = 0;
	elapsedMillis _stateTimer = 0;

	//Process values
	void _calculateProfile();
	void _update();
	long _startPosition = 0;
	long _position = 0;
	bool _limitSwA = false;
	bool _limitSwB = false;
	unsigned long _acc = 0;
	unsigned long _maxAccDist = 0;
	unsigned long _dec = 0;
	unsigned long _maxDecDist = 0;
	long _distance = 0;
	unsigned long _accTime = 0;
	unsigned long _cvTime = 0;
	unsigned long _decTime = 0;
	float _k = 0.0;
	elapsedMillis _timeStep = 0;
	unsigned int _homeSetpoint = 0;

	//Settings
	unsigned long _maxSpeed = 0; // [u/s] * 1000
	unsigned long _homeSpeed = 0; //[u/s] * 1000
	unsigned long _feedConstant = 0; // 
	unsigned long _startRamp = 0;
	unsigned long _stopRamp = 0;
	long _limitA = 0;
	long _limitB = 0;

	//Commands
	bool _cmdHomeA = false;
	bool _cmdHomeB = false;
	bool _cmdMoveA = false;
	bool _cmdMoveB = false;
	bool _cmdStop = false;
	bool _quickStop = false;

	//Setpoints
	long _target = 0;

};
#endif
