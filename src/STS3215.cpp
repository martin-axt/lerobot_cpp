/**
 * @file STS3215.cpp
 * @brief Feetech STS3215 series serial servo application layer implementation
 *
 * @details This file implements high-level control functions for Feetech STS3215
 * and STS series servo motors. It provides three operating modes with complete
 * read/write functionality and LSP-compliant initialization.
 *
 * **Implemented Features:**
 * - Mode 0: Position control with speed and acceleration
 * - Mode 1: Velocity control (closed-loop wheel mode)
 * - Mode 2: PWM control (open-loop wheel mode)
 * - Synchronized writes for multi-motor coordination
 * - Asynchronous writes with RegWriteAction
 * - Comprehensive feedback reading (position, speed, load, voltage, temp, current)
 * - EEPROM configuration management
 *
 * **Refactoring Improvements:**
 * - Uses ServoUtils for direction bit encoding/decoding (DRY principle)
 * - Uses SyncWriteBuffer for automatic memory management (RAII)
 * - Standardized error handling with ServoErrors
 *
 * @note All sync write operations use RAII-based buffer management
 * @see STS3215.h for class interface and usage examples
 */

#include <lerobot_cpp/INST.h>
#include <lerobot_cpp/STS3215.h>
#include <lerobot_cpp/SyncWriteBuffer.h>

/**
 * @brief Default constructor
 * 
 * Initializes STS3215 servo controller with default End byte (0).
 */
STS3215::STS3215()
{
	End = 0;
}

/**
 * @brief Constructor with custom End byte
 * 
 * @param End Protocol end byte (0 or 1)
 */
STS3215::STS3215(u8 End):SCSerial(End)
{
}

/**
 * @brief Constructor with End byte and response level
 * 
 * @param End Protocol end byte (0 or 1)
 * @param Level Response level (0=no response, 1=response for read/write commands)
 */
STS3215::STS3215(u8 End, u8 Level):SCSerial(End, Level)
{
}

/**
 * @brief Write position, speed, and acceleration to servo (extended command)
 * 
 * Sends single-servo position command with acceleration and speed control.
 * Negative positions are converted to absolute value with direction bit set.
 * 
 * @param ID Servo ID
 * @param Position Target position (0-4095 steps)
 * @param Speed Moving speed (0-3400 steps/s)
 * @param ACC Acceleration value (0-254)
 * @return 1 on success, 0 on failure
 */
int STS3215::WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC)
{
	u16 encodedPosition = ServoUtils::encodeSignedValue(Position, STS3215_DIRECTION_BIT_POS);

	u8 bBuf[7];
	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, encodedPosition);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, Speed);

	return genWrite(ID, STS3215_ACC, bBuf, 7);
}

/**
 * @brief Register write position command (executes on RegWriteAction)
 * 
 * Queues position/speed/acceleration command for later execution.
 * Use with RegWriteAction for synchronized multi-servo motion.
 * 
 * @param ID Servo ID
 * @param Position Target position (0-4095 steps)
 * @param Speed Moving speed (0-3400 steps/s)
 * @param ACC Acceleration value (0-254)
 * @return 1 on success, 0 on failure
 */
int STS3215::RegWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC)
{
	u16 encodedPosition = ServoUtils::encodeSignedValue(Position, STS3215_DIRECTION_BIT_POS);

	u8 bBuf[7];
	bBuf[0] = ACC;
	Host2SCS(bBuf+1, bBuf+2, encodedPosition);
	Host2SCS(bBuf+3, bBuf+4, 0);
	Host2SCS(bBuf+5, bBuf+6, Speed);

	return regWrite(ID, STS3215_ACC, bBuf, 7);
}

/**
 * @brief Synchronized position write for multiple servos
 * 
 * Sends position/speed/acceleration commands to multiple servos simultaneously.
 * All servos receive commands in single transmission for coordinated motion.
 * 
 * @param ID Array of servo IDs
 * @param IDN Number of servos
 * @param Position Array of target positions
 * @param Speed Array of speeds (NULL for 0 speed)
 * @param ACC Array of accelerations (NULL for 0 acceleration)
 */
void STS3215::SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
	SyncWriteBuffer buffer(IDN, 7);
	if(!buffer.isValid()){
		return;  // Allocation failed
	}
	for(u8 i = 0; i<IDN; i++){
		u16 encodedPosition = ServoUtils::encodeSignedValue(Position[i], STS3215_DIRECTION_BIT_POS);

		u8 bBuf[7];
		u16 V = Speed ? Speed[i] : 0;
		bBuf[0] = ACC ? ACC[i] : 0;
		Host2SCS(bBuf+1, bBuf+2, encodedPosition);
		Host2SCS(bBuf+3, bBuf+4, 0);
		Host2SCS(bBuf+5, bBuf+6, V);
		buffer.writeMotorData(i, bBuf, 7);
	}
	syncWrite(ID, IDN, STS3215_ACC, buffer.getBuffer(), 7);
}

/**
 * @brief Set servo operating mode
 * 
 * Configures servo for different operation modes:
 * - Mode 0: Servo position mode (default)
 * - Mode 1: Wheel closed-loop speed mode
 * - Mode 2: Wheel open-loop power mode
 * 
 * @param ID Servo ID
 * @param mode Operating mode (0-2)
 * @return 1 on success, 0 on failure (invalid mode or communication error)
 */
int STS3215::Mode(u8 ID, u8 mode)
{
	// Modes: 0 (servo), 1 (wheel closed-loop), 2 (wheel open-loop), 3 (stepper - not implemented)
	if(!(mode == STS3215_MODE_SERVO || mode == STS3215_MODE_WHEEL_CLOSED || mode == STS3215_MODE_WHEEL_OPEN)){
		Err = 1;
		return 0;
	}
	Err = 0;
	return writeByte(ID, STS3215_MODE, mode);
}

/**
 * @brief Initialize motor with operating mode and torque setting
 *
 * Convenience function that performs complete motor initialization:
 * 1. Unlocks EEPROM (to allow Operating_Mode write)
 * 2. Sets operating mode (0=position, 1=velocity, 2=PWM)
 * 3. Locks EEPROM (to save the mode setting)
 * 4. Optionally enables/disables torque
 *
 * This is the recommended way to initialize motors as it ensures the
 * Operating_Mode register (stored in EEPROM) is properly written and persists.
 *
 * @param ID Servo ID
 * @param mode Operating mode (0=servo, 1=wheel closed-loop, 2=wheel open-loop)
 * @param enableTorque 1 to enable torque, 0 to disable (default: 1)
 * @return 1 on success, 0 on failure
 */
int STS3215::InitMotor(u8 ID, u8 mode, u8 enableTorque)
{
	// Unlock EEPROM to allow writing to Operating_Mode register (address 33, stored in EEPROM)
	int ret = unLockEeprom(ID);
	if(ret == 0){
		Err = 1;
		return 0;
	}

	// Set operating mode
	ret = Mode(ID, mode);
	if(ret == 0){
		Err = 1;
		return 0;
	}

	// Lock EEPROM to save the Operating_Mode setting
	ret = LockEeprom(ID);
	if(ret == 0){
		Err = 1;
		return 0;
	}

	// Enable or disable torque as requested
	ret = EnableTorque(ID, enableTorque);
	if(ret == 0){
		Err = 1;
		return 0;
	}

	Err = 0;
	return 1;
}

/**
 * @brief Write speed command for wheel mode
 *
 * Controls servo speed in wheel mode. Negative speeds reverse direction.
 *
 * @param ID Servo ID
 * @param Speed Target speed (-3400 to +3400 steps/s, negative = reverse)
 * @param ACC Acceleration value (0-254)
 * @return 1 on success, 0 on failure
 */
int STS3215::WriteSpe(u8 ID, s16 Speed, u8 ACC)
{
	u16 encodedSpeed = ServoUtils::encodeSignedValue(Speed, STS3215_DIRECTION_BIT_POS);

	u8 bBuf[2];
	bBuf[0] = ACC;
	int ret = genWrite(ID, STS3215_ACC, bBuf, 1);
	if(ret == 0){
		Err = 1;
		return 0;
	}
	Host2SCS(bBuf+0, bBuf+1, encodedSpeed);

	return genWrite(ID, STS3215_GOAL_SPEED_L, bBuf, 2);
}

/**
 * @brief Register write speed command (executes on RegWriteAction)
 *
 * Queues speed command for later synchronized execution.
 *
 * @param ID Servo ID
 * @param Speed Target speed (-3400 to +3400 steps/s)
 * @param ACC Acceleration value (0-254)
 * @return 1 on success, 0 on failure
 */
int STS3215::RegWriteSpe(u8 ID, s16 Speed, u8 ACC)
{
	u16 encodedSpeed = ServoUtils::encodeSignedValue(Speed, STS3215_DIRECTION_BIT_POS);

	u8 bBuf[2];
	bBuf[0] = ACC;
	int ret = regWrite(ID, STS3215_ACC, bBuf, 1);
	if(ret == 0){
		Err = 1;
		return 0;
	}
	Host2SCS(bBuf+0, bBuf+1, encodedSpeed);

	return regWrite(ID, STS3215_GOAL_SPEED_L, bBuf, 2);
}

/**
 * @brief Synchronized speed write for multiple servos
 * 
 * Sends speed commands to multiple servos in wheel mode simultaneously.
 * 
 * @param ID Array of servo IDs
 * @param IDN Number of servos
 * @param Speed Array of target speeds
 * @param ACC Array of accelerations (NULL for 0 acceleration)
 */
void STS3215::SyncWriteSpe(u8 ID[], u8 IDN, s16 Speed[], u8 ACC[])
{
	SyncWriteBuffer buffer(IDN, 2);
	if(!buffer.isValid()){
		return;  // Allocation failed
	}
	for(u8 i = 0; i<IDN; i++){
		u16 encodedSpeed = ServoUtils::encodeSignedValue(Speed[i], STS3215_DIRECTION_BIT_POS);

		u8 bBuf[2];
		bBuf[0] = ACC ? ACC[i] : 0;
		genWrite(ID[i], STS3215_ACC, bBuf, 1);
		Host2SCS(bBuf+0, bBuf+1, encodedSpeed);
		buffer.writeMotorData(i, bBuf, 2);
	}
	syncWrite(ID, IDN, STS3215_GOAL_SPEED_L, buffer.getBuffer(), 2);
}

/**
 * @brief Write PWM output for open-loop mode
 * 
 * Directly controls motor PWM in open-loop wheel mode.
 * Negative values reverse direction.
 * 
 * @param ID Servo ID
 * @param Pwm PWM value (-1000 to 1000, negative = reverse)
 * @return 1 on success, 0 on failure
 */
int STS3215::WritePwm(u8 ID, s16 Pwm)
{
    u16 encodedPwm = ServoUtils::encodeSignedValue(Pwm, STS3215_LOAD_DIRECTION_BIT_POS);

    u8 bBuf[2];
    Host2SCS(bBuf+0, bBuf+1, encodedPwm);

    return genWrite(ID, STS3215_GOAL_TIME_L, bBuf, 2);
}

/**
 * @brief Register write PWM command (executes on RegWriteAction)
 * 
 * Queues PWM command for synchronized execution.
 * 
 * @param ID Servo ID
 * @param Pwm PWM value (-1000 to 1000)
 * @return 1 on success, 0 on failure
 */
int STS3215::RegWritePwm(u8 ID, s16 Pwm)
{
    u16 encodedPwm = ServoUtils::encodeSignedValue(Pwm, STS3215_LOAD_DIRECTION_BIT_POS);

    u8 bBuf[2];
    Host2SCS(bBuf+0, bBuf+1, encodedPwm);

    return regWrite(ID, STS3215_GOAL_TIME_L, bBuf, 2);
}

/**
 * @brief Synchronized PWM write for multiple servos
 * 
 * Sends PWM commands to multiple servos simultaneously.
 * 
 * @param ID Array of servo IDs
 * @param IDN Number of servos
 * @param Pwm Array of PWM values
 */
void STS3215::SyncWritePwm(u8 ID[], u8 IDN, s16 Pwm[])
{
	SyncWriteBuffer buffer(IDN, 2);
	if(!buffer.isValid()){
		return;  // Allocation failed
	}
	for(u8 i = 0; i<IDN; i++){
		u16 encodedPwm = ServoUtils::encodeSignedValue(Pwm[i], STS3215_LOAD_DIRECTION_BIT_POS);

		u8 bBuf[2];
		Host2SCS(bBuf+0, bBuf+1, encodedPwm);
		buffer.writeMotorData(i, bBuf, 2);
	}
	syncWrite(ID, IDN, STS3215_GOAL_TIME_L, buffer.getBuffer(), 2);
}

/**
 * @brief Enable or disable servo motor torque
 * 
 * Controls whether servo actively maintains position or is free-moving.
 * 
 * @param ID Servo ID
 * @param Enable 1 to enable torque, 0 to disable (free-moving)
 * @return 1 on success, 0 on failure
 */
int STS3215::EnableTorque(u8 ID, u8 Enable)
{
	return writeByte(ID, STS3215_TORQUE_ENABLE, Enable);
}

/**
 * @brief Unlock EEPROM for writing
 * 
 * Allows modification of EEPROM parameters (ID, baud rate, limits, etc.).
 * 
 * @param ID Servo ID
 * @return 1 on success, 0 on failure
 */
int STS3215::unLockEeprom(u8 ID)
{
	return writeByte(ID, STS3215_LOCK, 0);
}

/**
 * @brief Lock EEPROM to prevent accidental changes
 *
 * Protects EEPROM parameters from modification.
 *
 * @param ID Servo ID
 * @return 1 on success, 0 on failure
 */
int STS3215::LockEeprom(u8 ID)
{
	return writeByte(ID, STS3215_LOCK, 1);
}

/**
 * @brief Calibrate servo center position offset
 * 
 * Initiates automatic calibration of servo zero position.
 * Servo must be manually positioned at desired center before calling.
 * 
 * @param ID Servo ID
 * @return 1 on success, 0 on failure
 */
int STS3215::CalibrationOfs(u8 ID)
{
	return writeByte(ID, STS3215_TORQUE_ENABLE, STS3215_CALIBRATION_CMD);
}

/**
 * @brief Read all feedback data from servo into memory buffer
 *
 * Reads position, speed, load, voltage, temperature, movement status, and current.
 * Data is stored in Mem[] array and can be accessed via ReadPos(), ReadSpeed(), etc.
 *
 * @param ID Servo ID
 * @return 1 on success, 0 on failure
 */
int STS3215::FeedBack(int ID)
{
	int nLen = Read(ID, STS3215_PRESENT_POSITION_L, Mem, sizeof(Mem));
	if(nLen!=sizeof(Mem)){
		Err = 1;
		return 0;
	}
	Err = 0;
	return 1;
}

/**
 * @brief Read current servo position
 * 
 * Reads position from servo or from cached Mem[] buffer.
 * 
 * @param ID Servo ID, or -1 to read from cached buffer (after FeedBack())
 * @return Position value (-4095 to 4095), -1 on error
 */
int STS3215::ReadPos(int ID)
{
		if(ID == -1) {
			return ServoUtils::readSignedWordFromBuffer(
				Mem,
				STS3215_PRESENT_POSITION_L - STS3215_PRESENT_POSITION_L,
				STS3215_PRESENT_POSITION_H - STS3215_PRESENT_POSITION_L,
				STS3215_DIRECTION_BIT_POS
			);
		}
		Err = 0;
		return readSignedWord(ID, STS3215_PRESENT_POSITION_L, STS3215_DIRECTION_BIT_POS);
}

/**
 * @brief Read current servo speed
 * 
 * Reads speed from servo or from cached buffer.
 * 
 * @param ID Servo ID, or -1 to read from cached buffer
 * @return Speed value (-32767 to 32767), -1 on error
 */
int STS3215::ReadSpeed(int ID)
{
		if(ID == -1) {
			return ServoUtils::readSignedWordFromBuffer(
				Mem,
				STS3215_PRESENT_SPEED_L - STS3215_PRESENT_POSITION_L,
				STS3215_PRESENT_SPEED_H - STS3215_PRESENT_POSITION_L,
				STS3215_DIRECTION_BIT_POS
			);
		}
		Err = 0;
		return readSignedWord(ID, STS3215_PRESENT_SPEED_L, STS3215_DIRECTION_BIT_POS);
}

/**
 * @brief Read current servo load
 * 
 * Reads load torque from servo or from cached buffer.
 * 
 * @param ID Servo ID, or -1 to read from cached buffer
 * @return Load value (-1000 to 1000), -1 on error
 */
int STS3215::ReadLoad(int ID)
{
		if(ID == -1) {
			return ServoUtils::readSignedWordFromBuffer(
				Mem,
				STS3215_PRESENT_LOAD_L - STS3215_PRESENT_POSITION_L,
				STS3215_PRESENT_LOAD_H - STS3215_PRESENT_POSITION_L,
				STS3215_LOAD_DIRECTION_BIT_POS
			);
		}
		Err = 0;
		return readSignedWord(ID, STS3215_PRESENT_LOAD_L, STS3215_LOAD_DIRECTION_BIT_POS);
}

/**
 * @brief Read servo supply voltage
 * 
 * Reads operating voltage from servo or from cached buffer.
 * 
 * @param ID Servo ID, or -1 to read from cached buffer
 * @return Voltage in 0.1V units (e.g., 120 = 12.0V), -1 on error
 */
int STS3215::ReadVoltage(int ID)
{	
	int Voltage = -1;
	if(ID==-1){
		Voltage = Mem[STS3215_PRESENT_VOLTAGE-STS3215_PRESENT_POSITION_L];	
	}else{
		Err = 0;
		Voltage = readByte(ID, STS3215_PRESENT_VOLTAGE);
		if(Voltage==-1){
			Err = 1;
		}
	}
	return Voltage;
}

/**
 * @brief Read servo internal temperature
 * 
 * Reads temperature from servo or from cached buffer.
 * 
 * @param ID Servo ID, or -1 to read from cached buffer
 * @return Temperature in degrees Celsius, -1 on error
 */
int STS3215::ReadTemper(int ID)
{	
	int Temper = -1;
	if(ID==-1){
		Temper = Mem[STS3215_PRESENT_TEMPERATURE-STS3215_PRESENT_POSITION_L];	
	}else{
		Err = 0;
		Temper = readByte(ID, STS3215_PRESENT_TEMPERATURE);
		if(Temper==-1){
			Err = 1;
		}
	}
	return Temper;
}

/**
 * @brief Read servo movement status
 * 
 * Checks if servo is currently moving or has reached target position.
 * 
 * @param ID Servo ID, or -1 to read from cached buffer
 * @return 1 if moving, 0 if stopped, -1 on error
 */
int STS3215::ReadMove(int ID)
{
	int Move = -1;
	if(ID==-1){
		Move = Mem[STS3215_MOVING-STS3215_PRESENT_POSITION_L];	
	}else{
		Err = 0;
		Move = readByte(ID, STS3215_MOVING);
		if(Move==-1){
			Err = 1;
		}
	}
	return Move;
}

/**
 * @brief Read servo motor current
 * 
 * Reads motor current from servo or from cached buffer.
 * 
 * @param ID Servo ID, or -1 to read from cached buffer
 * @return Current value in mA, -1 on error
 */
int STS3215::ReadCurrent(int ID)
{
		if(ID == -1) {
			return ServoUtils::readSignedWordFromBuffer(
				Mem,
				STS3215_PRESENT_CURRENT_L - STS3215_PRESENT_POSITION_L,
				STS3215_PRESENT_CURRENT_H - STS3215_PRESENT_POSITION_L,
				STS3215_DIRECTION_BIT_POS
			);
		}
		Err = 0;
		return readSignedWord(ID, STS3215_PRESENT_CURRENT_L, STS3215_DIRECTION_BIT_POS);
}

