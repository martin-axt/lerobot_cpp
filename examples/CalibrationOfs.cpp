/**
 * @file CalibrationOfs.cpp
 * @brief Mechanical center position calibration for STS3215 protocol servos
 * 
 * @details
 * This example demonstrates the center position offset calibration procedure for
 * Feetech STS3215 servos. This calibration compensates for mechanical assembly
 * tolerances by setting the current physical position as the logical zero point
 * (center position). This is essential for applications requiring precise absolute
 * positioning or when replacing/rebuilding servo mechanisms.
 * 
 * Hardware Requirements:
 * - Feetech STS3215 protocol servo (ID: 1)
 * - USB-to-Serial adapter or direct serial port
 * - Power supply appropriate for servo model
 * - Serial connection at 115200 baud
 * - Servo must be positioned at desired center/zero before running
 * 
 * Key Features Demonstrated:
 * - CalibrationOfs(): Set current position as logical zero (center)
 * - EEPROM write for persistent calibration storage
 * - Factory reset alternative using offset adjustment
 * 
 * Usage:
 * @code
 * # 1. Manually position servo to desired center/zero point
 * # 2. Run calibration
 * ./CalibrationOfs /dev/ttyUSB0
 * # 3. New center position is now saved to EEPROM
 * @endcode
 * 
 * Calibration Procedure:
 * 1. Physically position servo shaft to desired center/zero point
 * 2. Run this program
 * 3. CalibrationOfs() reads current position and calculates offset
 * 4. Offset is written to EEPROM (persists across power cycles)
 * 5. All future position commands are relative to this new center
 * 
 * Use Cases:
 * - Compensating for gear backlash or mechanical tolerances
 * - Setting custom zero points for joint mechanisms
 * - Correcting position drift after mechanical repairs
 * - Aligning multiple servos to common reference frame
 * - Factory-style calibration for production systems
 * 
 * @note This command writes to EEPROM. Excessive use (>100,000 cycles) may wear
 *       out EEPROM memory. Use sparingly for calibration, not in control loops.
 * 
 * @warning Ensure servo is at the desired center position BEFORE running this command.
 *          The current physical position becomes the new logical zero.
 * 
 * @warning After calibration, all previous position values may be offset. Recalibrate
 *          or adjust your control software accordingly.
 * 
 * @warning Servo must be powered and holding position during calibration. Unpowered
 *          or free-moving servos will result in incorrect calibration.
 * 
 * @see STS3215::CalibrationOfs()
 * @see STS3215::writeByte() for manual offset adjustment
 */
#include <iostream>
#include <lerobot_cpp/STS3215.h>

STS3215 sm_st;

int main(int argc, char **argv)
{
	if(argc<3){
		std::cout<<"argc error!"<<std::endl;
		return 0;
	}
	char* serial = argv[1];
	int serialSpeed = std::stoi(argv[2]);

	std::cout<<"serial: "<<serial<<std::endl;
	std::cout<<"serial speed: "<<serialSpeed<<std::endl;

	if(!sm_st.begin(serialSpeed, serial)){
		std::cout<<"Failed to init STS3215 motor!"<<std::endl;
		return 0;
	}

	for (int i = 0; i < 6; i++)
		sm_st.CalibrationOfs(i);
	std::cout<<"Calibration Ofs"<<std::endl;
	sm_st.end();
	return 1;
}

