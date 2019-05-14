/** @file interface_main.cpp
 * 
 *  @author Andrew Jin
 *  @competition IARRC 2019
 */

// GLOBAL
#include <iostream>

// UART
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// LOCAL
#include "interface.hpp"

/** @brief starts the interface node
 *
 *  nothing happens until enable is true
 *
 *  @return NONE
 */ 
int main(int argc, char **argv)
{
	int uart0_filestream = -1;
	/* The flags (defined in fcntl.h):
		Access modes (use 1 of these):
			O_RDONLY - Open for reading only.
			O_RDWR - Open for reading and writing.
			O_WRONLY - Open for writing only.

		O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
												if there is no input immediately available (instead of blocking). Likewise, write requests can also return
												immediately with a failure status if the output can't be written immediately.

		O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	*/
	uart0_filestream = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);		//Open in non blocking read/write mode

	//ERROR - CAN'T OPEN SERIAL PORT
	if (uart0_filestream == -1) ROS_ERROR("Error - Unable to open UART.  Ensure it is not in use by another application\n");

	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B115200 | CS8 | PARENB | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
	char rx_buffer[256];
	memset(&rx_buffer, '\0', sizeof(rx_buffer));


	ros::init(argc, argv, "rr_interface");
	ros::NodeHandle nh;
	Interface interface(nh);

	ROS_INFO("Interface: Interface Node Initialized");
	ros::Rate r(20);
	int regulator = 0;

	// Infinite Loop
	while(ros::ok()) {
		if (uart0_filestream != -1) {
			if (regulator%2 == 0) {
				char payload[sizeof(interface.transmitter_)];
				//Method 1
				memcpy(&payload, &(interface.transmitter_), sizeof(payload));
				int n = write(uart0_filestream, payload, sizeof(payload));
				// Method 2
				// int n = write(uart0_filestream, htonl(payload), sizeof(payload));
				usleep ((sizeof(payload) + 25) * 100);
				regulator = 0;
			}
			int rx_length = read(uart0_filestream, &rx_buffer, sizeof(rx_buffer));		//Filestream, buffer to store in, number of bytes to read (max)
			if (rx_length == 0) {
				ROS_INFO("NO DATA WAITING");
			}
			else if (rx_length < 0) {
				ROS_WARN("DATA INVALID");
			}
			else {
				//Bytes received
				// Method 1
				//char payload[sizeof(interface.receiver_)];
				
				//memcpy(&interface.receiver_, &rx_length, sizeof(Interface::Receiver));
				// Method 2
				// interface.receiver_ = ntohl(rx_buffer);
				// rx_buffer[rx_length] = '\0';
				//std::string r = "" + interface.receiver_.payload;
				
				ROS_INFO("%i bytes read : %s\n", rx_length, rx_buffer);
			}
		}

		regulator++;
		ros::spinOnce();
		r.sleep();
	}

	//----- CLOSE THE UART -----
	close(uart0_filestream);
  
	return 0;
}
