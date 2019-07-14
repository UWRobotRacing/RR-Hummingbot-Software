/** @file interface_main.cpp
 * 
 *  @author Andrew Jin
 *  @author Waleed Ahmed
 *  @competition IARRC 2019
 */

// GLOBAL
#include <iostream>
#include <cctype>

// UART
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

// LOCAL
#include "interface.hpp"

// Change this for different serial values
const std::string serial_port = "/dev/ttyACM0";
int serial_port_filestream;

// Forward declaration
void setupCommunication();

/** @brief starts the interface node
 *
 *  nothing happens until enable is true
 *
 *  @return NONE
 */ 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rr_interface");
  ros::NodeHandle nh;
  Interface interface(nh);

  ROS_INFO("Interface: Interface Node Initialized");
  ros::Rate r(10);

  // Declared later to remove some preset communication stuff that doesnt need to be tweaked
  setupCommunication();

  int counter = 0;
  while (ros::ok())
  {
    // Writing 
    //uint8_t convert[8];
    interface.transmitter_.myFrame.startByte = 'a';
    interface.transmitter_.myFrame.endByte = 'n';

    //memcpy(convert, &interface.transmitter_, sizeof(Interface::Transmitter));
    int writen_bytes = write(serial_port_filestream, interface.transmitter_.serializedArray, sizeof(interface.transmitter_));
    //ROS_INFO("VALUE %d", sizeof(interface.transmitter_));
    // if (counter == 0)
    // {
    //   // Writing 
    //   Interface::Transmitter packet = interface.transmitter_;
    //   // Add Padding
    //   packet.padding = 65535;
    //   /*
    //   unsigned char *payload = new unsigned char [sizeof(packet)];
    //   unsigned char *convert = (unsigned char*)&packet;
    //   for (int i = 0; i < sizeof(packet); i++)
    //   {
    //     payload[i] = convert[sizeof(payload)-i-1];
    //   }
    //   int written_bytes = write(serial_port_filestream, payload, sizeof(packet));
    //   delete payload;
    //   payload = nullptr;
    //    */
    //   unsigned char *convert = (unsigned char*)&packet;
    //   int writeen_bytes = write(serial_port_filestream, convert, sizeof(convert));
    // }
    // else
    // {
    //   // Reading 
    //   // Allocate memory for read buffer, set size according to your needs
    //   char read_buf [sizeof(Interface::Receiver)];
    //   memset(&read_buf, '\0', sizeof(read_buf));
      
    //   // Read bytes. The behaviour of read() (e.g. does it block?,
    //   // how long does it block for?) depends on the configuration
    //   // settings above, specifically VMIN and VTIME
    //   int read_bytes = read(serial_port_filestream, &read_buf, sizeof(read_buf));

    //   // Ensures that there is always data flowing through
    //   if (read_bytes != -1)
    //   { 
    //     interface.receiver_ = interface.Deserialize(read_buf);
    //   }
      
    //   if (counter == 2)
    //   {
    //     counter = 0;
    //   }
    //   else 
    //   {
    //     counter++;
    //   }
    // }

    ros::spinOnce();
    r.sleep();
  }

  //----- CLOSE THE UART -----
  close(serial_port_filestream);
  ROS_INFO("Closed Serial Port");
  return 0;
}

void setupCommunication()
{
  // The following tutorial was followed to set up this serial UART communcation: 
  // https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

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
  std::string serial_port_name = serial_port;
  // Everything in linux is considered a file, hence why this varaible is referred to as a file stream
  serial_port_filestream = open(serial_port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  // Check for errors
  if (serial_port_filestream < 0) {
      ROS_ERROR("Error %i from open: %s\n", errno, strerror(errno));
  }
  ROS_INFO("Serial port %s opened in non-blocking read/write mode", serial_port_name.c_str());

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;
  memset(&tty, 0, sizeof tty);

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port_filestream, &tty) != 0) {
      ROS_ERROR("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }
  
  // Now we will adjust the termios settings to match our serial commuication specifications

  // Parity bits
  tty.c_cflag &= ~PARENB; // Clear parity bit, meaning we will not use a parity bit
  // Following code enables parity bit
  // tty.c_cflag |= PARENB;
  // tty.c_cflag |= PARODD;	// If set, odd parity used, otherwise default is even parity

  // Stop bits
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  // tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication

  // Data bits 
  tty.c_cflag |= CS8; // 8 bits per byte
  // tty.c_cflag |= CS5; // 5 bits per byte
  // tty.c_cflag |= CS6; // 6 bits per byte
  // tty.c_cflag |= CS7; // 7 bits per byte

  // Misc settings
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  tty.c_lflag &= ~ICANON;	// Disable canonical mode 
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

  // Setting VMIn and VTIME to 0 means no blocking, return immediately 
  // with what is availalbe on the port, which is what we want
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = sizeof(Interface::Receiver);

  // Baud Rate
  // UNIX compliant baud rates are: 
  // B0,  B50,  B75,  B110,  B134,  B150,  B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
  cfsetispeed(&tty, B115200);	// Input
  cfsetospeed(&tty, B115200);	// Output

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port_filestream, TCSANOW, &tty) != 0) {
      ROS_ERROR("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
  ROS_INFO("Successfully configured serial port settings for UART communication");

}
