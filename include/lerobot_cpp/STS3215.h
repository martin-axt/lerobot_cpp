/**
 * @file STS3215.h
 * @brief Feetech STS3215 Series Serial Servo Application Layer
 *
 * @details This file provides the application programming interface for
 * controlling Feetech STS3215 series serial bus servo motors.
 * Supports three operating modes:
 * - Mode 0: Servo (position control)
 * - Mode 1: Closed-loop wheel (velocity control with feedback)
 * - Mode 2: Open-loop wheel (PWM control without feedback)
 */

#ifndef _STS3215_H
#define _STS3215_H

// Baud rate definition
#define	STS3215_1M 0      // 1Mbps
#define	STS3215_0_5M 1    // 0.5Mbps
#define	STS3215_250K 2    // 250kbps
#define	STS3215_128K 3    // 128kbps
#define	STS3215_115200 4  // 115200bps
#define	STS3215_76800 5   // 76800bps
#define	STS3215_57600 6   // 57600bps
#define	STS3215_38400 7   // 38400bps

//Memory table definition
//-------EEPROM (Read only)--------
#define STS3215_MODEL_L 3   // Model number (Low byte)
#define STS3215_MODEL_H 4   // Model number (High byte)

//-------EEPROM (Read and Write)--------
#define STS3215_ID 5                // Servo ID (0-253)
#define STS3215_BAUD_RATE 6         // Baud rate selector
#define STS3215_MIN_ANGLE_LIMIT_L 9  // Minimum angle limit (Low byte)
#define STS3215_MIN_ANGLE_LIMIT_H 10 // Minimum angle limit (High byte)
#define STS3215_MAX_ANGLE_LIMIT_L 11 // Maximum angle limit (Low byte)
#define STS3215_MAX_ANGLE_LIMIT_H 12 // Maximum angle limit (High byte)
#define STS3215_CW_DEAD 26           // Clockwise dead zone
#define STS3215_CCW_DEAD 27          // Counter-clockwise dead zone
#define STS3215_OFS_L 31             // Offset calibration (Low byte)
#define STS3215_OFS_H 32             // Offset calibration (High byte)
#define STS3215_MODE 33              // Operating mode (Servo/Wheel)

//-------SRAM (Read and Write)--------
#define STS3215_TORQUE_ENABLE 40    // Torque enable (0: disable, 1: enable)
#define STS3215_ACC 41              // Acceleration
#define STS3215_GOAL_POSITION_L 42  // Target position (Low byte)
#define STS3215_GOAL_POSITION_H 43  // Target position (High byte)
#define STS3215_GOAL_TIME_L 44      // Target time (Low byte)
#define STS3215_GOAL_TIME_H 45      // Target time (High byte)
#define STS3215_GOAL_SPEED_L 46     // Target speed (Low byte)
#define STS3215_GOAL_SPEED_H 47     // Target speed (High byte)
#define STS3215_LOCK 55             // EEPROM lock (0: unlocked, 1: locked)

//-------SRAM (Read only)--------
#define STS3215_PRESENT_POSITION_L 56    // Current position (Low byte)
#define STS3215_PRESENT_POSITION_H 57    // Current position (High byte)
#define STS3215_PRESENT_SPEED_L 58       // Current speed (Low byte)
#define STS3215_PRESENT_SPEED_H 59       // Current speed (High byte)
#define STS3215_PRESENT_LOAD_L 60        // Current load (Low byte)
#define STS3215_PRESENT_LOAD_H 61        // Current load (High byte)
#define STS3215_PRESENT_VOLTAGE 62       // Current voltage
#define STS3215_PRESENT_TEMPERATURE 63   // Current temperature
#define STS3215_MOVING 66                // Movement status (0: stopped, 1: moving)
#define STS3215_PRESENT_CURRENT_L 69     // Current current (Low byte)
#define STS3215_PRESENT_CURRENT_H 70     // Current current (High byte)

// Bit position constants for data encoding
#define STS3215_DIRECTION_BIT_POS 15    // Bit position for direction flag in position/speed
#define STS3215_LOAD_DIRECTION_BIT_POS 10  // Bit position for direction flag in load/PWM

// Operating mode values
#define STS3215_MODE_SERVO 0        // Servo mode (position control)
#define STS3215_MODE_WHEEL_CLOSED 1 // Wheel mode - closed loop (velocity control)
#define STS3215_MODE_WHEEL_OPEN 2   // Wheel mode - open loop (PWM control)
#define STS3215_MODE_STEPPER 3      // Stepper mode (not implemented in SDK)

// Special servo IDs
#define STS3215_BROADCAST_ID 0xFE   // Broadcast ID for all servos

// Calibration command
#define STS3215_CALIBRATION_CMD 128 // Command value for midpoint calibration

#include <lerobot_cpp/SCSerial.h>
#include <lerobot_cpp/INST.h>
#include <lerobot_cpp/ServoErrors.h>
#include <lerobot_cpp/ServoUtils.h>

/**
 * @class STS3215
 * @brief Application layer interface for STS3215 series serial servos
 *
 * @details Provides high-level control functions for Feetech STS3215 series
 * servo motors. Supports three operating modes with complete read/write functionality.
 *
 * **Operating Modes:**
 * - Mode 0: Servo mode (position control) - precise positioning
 * - Mode 1: Wheel mode closed-loop (velocity control) - speed feedback
 * - Mode 2: Wheel mode open-loop (PWM control) - direct motor power
 *
 * **Key Features:**
 * - Write operations: immediate, asynchronous (Reg), and synchronized (Sync)
 * - Comprehensive feedback: position, speed, load, voltage, temperature, current
 * - EEPROM management: lock/unlock for persistent configuration
 * - LSP compliant: uniform InitMotor() and Mode() methods
 *
 * **Usage Example:**
 * @code
 * STS3215 servo;
 * servo.begin(1000000, "/dev/ttyUSB0");
 * servo.InitMotor(1, 0, 1);  // ID=1, Mode=0 (servo), Enable torque
 * servo.WritePosEx(1, 2048, 1000, 50);  // Move to center position
 * @endcode
 *
 * @note Remember to call begin() before using any servo methods
 * @see SCSerial for serial communication layer methods
 */
class STS3215 : public SCSerial
{
public:
	/** @brief Default constructor */
	STS3215();
	/** @brief Constructor with protocol end byte
	 *  @param End Protocol end byte (0 or 1) */
	STS3215(u8 End);
	/** @brief Constructor with protocol end byte and response level
	 *  @param End Protocol end byte (0 or 1)
	 *  @param Level Response level (0=no response, 1=response enabled) */
	STS3215(u8 End, u8 Level);
	/** @brief Write position to single servo (Mode 0)
	 *  @param ID Servo ID (0-253, 254=broadcast)
	 *  @param Position Target position (0-4095 steps)
	 *  @param Speed Movement speed (0-3400 steps/s)
	 *  @param ACC Acceleration (0-254, units of 100 steps/s²)
	 *  @return 1 on success, 0 on failure */
	virtual int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);

	/** @brief Async write position (execute with RegWriteAction)
	 *  @param ID Servo ID
	 *  @param Position Target position
	 *  @param Speed Movement speed
	 *  @param ACC Acceleration
	 *  @return 1 on success, 0 on failure */
	virtual int RegWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);

	/** @brief Sync write position to multiple servos
	 *  @param ID Array of servo IDs
	 *  @param IDN Number of servos
	 *  @param Position Array of target positions
	 *  @param Speed Array of speeds (NULL for 0)
	 *  @param ACC Array of accelerations (NULL for 0) */
	virtual void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);

	/** @brief Set operating mode
	 *  @param ID Servo ID
	 *  @param mode 0=servo, 1=wheel closed-loop, 2=wheel open-loop
	 *  @return 1 on success, 0 on failure */
	virtual int Mode(u8 ID, u8 mode);

	/** @brief Initialize motor (unlock EEPROM → set mode → lock EEPROM → enable torque)
	 *  @param ID Servo ID
	 *  @param mode Operating mode (0/1/2)
	 *  @param enableTorque 1=enable, 0=disable (default: 1)
	 *  @return 1 on success, 0 on failure
	 *  @note LSP compliant - available on all servo classes */
	virtual int InitMotor(u8 ID, u8 mode, u8 enableTorque = 1);

	/** @brief Write speed to single servo (Mode 1)
	 *  @param ID Servo ID
	 *  @param Speed Target speed (-3400 to +3400 steps/s)
	 *  @param ACC Acceleration
	 *  @return 1 on success, 0 on failure */
	virtual int WriteSpe(u8 ID, s16 Speed, u8 ACC = 0);

	/** @brief Async write speed */
    virtual int RegWriteSpe(u8 ID, s16 Speed, u8 ACC = 0);

	/** @brief Sync write speed to multiple servos */
    virtual void SyncWriteSpe(u8 ID[], u8 IDN, s16 Speed[], u8 ACC[]);

	/** @brief Write PWM to single servo (Mode 2)
	 *  @param ID Servo ID
	 *  @param Pwm PWM duty cycle (±1000 = ±100%)
	 *  @return 1 on success, 0 on failure */
    virtual int WritePwm(u8 ID, s16 Pwm);

	/** @brief Async write PWM */
    virtual int RegWritePwm(u8 ID, s16 Pwm);

	/** @brief Sync write PWM to multiple servos */
    virtual void SyncWritePwm(u8 ID[], u8 IDN, s16 Pwm[]);

	/** @brief Enable/disable motor torque
	 *  @param ID Servo ID
	 *  @param Enable 1=enable, 0=disable (free-moving)
	 *  @return 1 on success, 0 on failure */
	virtual int EnableTorque(u8 ID, u8 Enable);

	/** @brief Unlock EEPROM for writing configuration
	 *  @param ID Servo ID
	 *  @return 1 on success, 0 on failure */
	virtual int unLockEeprom(u8 ID);

	/** @brief Lock EEPROM to protect configuration
	 *  @param ID Servo ID
	 *  @return 1 on success, 0 on failure */
	virtual int LockEeprom(u8 ID);

	/** @brief Calibrate servo midpoint position
	 *  @param ID Servo ID
	 *  @return 1 on success, 0 on failure
	 *  @note Sets offset register to 128 (different from InitMotor which sets operating mode) */
	virtual int CalibrationOfs(u8 ID);

	/** @brief Read all feedback data into internal buffer
	 *  @param ID Servo ID
	 *  @return 1 on success, 0 on failure
	 *  @note Call before using Read* methods with ID=-1 for cached reads */
	virtual int FeedBack(int ID);

	/** @brief Read current position
	 *  @param ID Servo ID, or -1 to read from cache (after FeedBack)
	 *  @return Position (0-4095), -1 on error */
	virtual int ReadPos(int ID);

	/** @brief Read current speed
	 *  @param ID Servo ID, or -1 for cached read
	 *  @return Speed (±3400 steps/s), -1 on error */
	virtual int ReadSpeed(int ID);

	/** @brief Read motor load
	 *  @param ID Servo ID, or -1 for cached read
	 *  @return Load (±1000 = ±100% PWM), -1 on error */
	virtual int ReadLoad(int ID);

	/** @brief Read supply voltage
	 *  @param ID Servo ID, or -1 for cached read
	 *  @return Voltage in 0.1V units, -1 on error */
	virtual int ReadVoltage(int ID);

	/** @brief Read internal temperature
	 *  @param ID Servo ID, or -1 for cached read
	 *  @return Temperature in °C, -1 on error */
	virtual int ReadTemper(int ID);

	/** @brief Read movement status
	 *  @param ID Servo ID, or -1 for cached read
	 *  @return 1=moving, 0=stopped, -1=error */
	virtual int ReadMove(int ID);

	/** @brief Read motor current
	 *  @param ID Servo ID, or -1 for cached read
	 *  @return Current in milliamps, -1 on error */
	virtual int ReadCurrent(int ID);
private:
	u8 Mem[STS3215_PRESENT_CURRENT_H-STS3215_PRESENT_POSITION_L+1];
};

#endif