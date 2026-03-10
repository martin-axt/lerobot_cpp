/**
 * @file Ping.cpp
 * @brief Servo connectivity test and ID verification for STS3215 protocol servos
 * 
 * @details
 * This example demonstrates the Ping command for verifying servo presence and communication
 * on the serial bus. Ping is essential for diagnostics, servo discovery, and verification
 * before attempting control commands. It confirms the servo is powered, properly connected,
 * configured with the expected ID, and responding to commands at the correct baud rate.
 * 
 * Hardware Requirements:
 * - Feetech STS3215 protocol servo (ID: 1)
 * - USB-to-Serial adapter or direct serial port
 * - Power supply appropriate for servo model
 * - Serial connection at 115200 baud
 * - Proper electrical connections (TX, RX, GND)
 * 
 * Key Features Demonstrated:
 * - Ping(): Send ping command to specific servo ID
 * - ID verification and response validation
 * - Basic communication diagnostics
 * - Error detection for missing or unresponsive servos
 * 
 * Usage:
 * @code
 * ./Ping /dev/ttyUSB0
 * # Output on success: "ID:1"
 * # Output on failure: "Ping servo ID error!"
 * @endcode
 * 
 * Ping Protocol:
 * - Sends ping packet to servo with target ID
 * - Servo responds with status packet containing its ID
 * - Returns servo ID on success, -1 on failure
 * - Timeout typically ~100ms for no response
 * 
 * Common Use Cases:
 * - Initial system setup and servo discovery
 * - Pre-flight checks before motion commands
 * - Debugging communication issues (baud rate, wiring, power)
 * - Scanning bus for all connected servos (loop through IDs)
 * - Verifying servo configuration after EEPROM programming
 * 
 * Troubleshooting Ping Failures:
 * - Check power supply (servo LED should be on)
 * - Verify baud rate matches servo configuration
 * - Check TX/RX wiring (may be swapped)
 * - Ensure ground connection between servo and controller
 * - Test with broadcast ID (254) if only one servo on bus
 * - Try different baud rates (115200, 1000000)
 * - Check for conflicting IDs on bus
 * 
 * @note Ping does not control the servo or verify its functional state, only communication.
 *       A successful ping means servo is responding, not necessarily ready for motion.
 * 
 * @note Broadcast ping (ID=254) only works reliably with a single servo on the bus.
 *       Multiple servos will cause response collisions.
 * 
 * @warning If ping fails, do NOT attempt control commands. Motion commands to non-existent
 *          servos will timeout and may cause unexpected behavior in control loops.
 * 
 * @see STS3215::Ping()
 * @see STS3215::begin() for baud rate configuration
 */
/*
Ping command test, tests whether the servo with the corresponding ID on the bus is ready. Broadcast command is only applicable when there is only one servo on the bus.
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
	int ID = sm_st.Ping(1);
	if(ID!=-1){
		std::cout<<"ID:"<<ID<<std::endl;
	}else{
		std::cout<<"Ping servo ID error!"<<std::endl;
	}
	sm_st.end();
	return 1;
}
