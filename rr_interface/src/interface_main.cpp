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
const char serial_port[] = "/dev/ttyACM0";

int serialport_init(const char* serialport, int baud);
int serialport_write(int fd, uint8_t* str, int len);
int serialport_writebyte(int fd, uint8_t b);

/** @brief starts the interface node
 *
 *  nothing happens until enable is true
 *
 *  @return NONE
 */ 

int main(int argc, char **argv)
{
  int SERIAL_PORT_FS;
  ros::init(argc, argv, "rr_interface");
  ros::NodeHandle nh;
  Interface interface(nh);

  ROS_INFO("Interface: SERIAL Interface Node Initialized");
  ros::Rate r(5);

  // Declared later to remove some preset communication stuff that doesnt need to be tweaked
  SERIAL_PORT_FS = serialport_init(serial_port, 9600);

// delay sync
  ROS_INFO("Interface: SERIAL Begin");
  usleep(1000000);
  ROS_INFO("Interface: SYNC 1");
  serialport_writebyte(SERIAL_PORT_FS, char(254));
//   usleep(1000000);
//   ROS_INFO("Interface: SYNC 2");
//   serialport_writebyte(SERIAL_PORT_FS, char(255));
//   serialport_writebyte(SERIAL_PORT_FS, char(255));
//   serialport_writebyte(SERIAL_PORT_FS, char(255));
//   usleep(1000000);
  ROS_INFO("Interface: SYNC END");
  int counter = 0;
  int clear_cnter = 0;
  while (ros::ok())
  {
    // Writing 
    jetson_union_t send_buffer;
    interface.transmitter_.myFrame.startByte = 'a';
    interface.transmitter_.myFrame.endByte = 'n';
    counter ++;
    memcpy(&send_buffer, &interface.transmitter_.serializedArray, sizeof(jetson_union_t));
    // int writen_bytes = write(SERIAL_PORT_FS, send_buffer.serializedArray, sizeof(send_buffer));
    // ROS_INFO("VALUE %d", writen_bytes);
    // well well well, we fuking need 0xFF to pad the data to prevent corruption and allow detection on arduino
    uint8_t dummy[] = {0x0A};
    uint8_t dummy2[] = {(uint8_t)('\n')};
    bool result = true;
    // write(SERIAL_PORT_FS, dummy, 8);
    // serialport_write(SERIAL_PORT_FS, cmd, 8); 
    result &= serialport_write(SERIAL_PORT_FS, send_buffer.serializedArray, sizeof(jetson_union_t));
    result &= serialport_writebyte(SERIAL_PORT_FS, 0x0A);
    result &= serialport_writebyte(SERIAL_PORT_FS, '\r');
    result &= serialport_writebyte(SERIAL_PORT_FS, '\n');
    // result &= serialport_write(SERIAL_PORT_FS, send_buffer.serializedArray, sizeof(jetson_union_t));
    // result &= serialport_write(SERIAL_PORT_FS, dummy2, 8);
    // ROS_INFO("Ang: %d  Spd: %d   FLAG: %d   | %d :: %s", 
    //             interface.transmitter_.myFrame.data.jetson_ang,
    //             interface.transmitter_.myFrame.data.jetson_spd,
    //             interface.transmitter_.myFrame.data.jetson_flag,
    //             counter,
    //             (result?"SUCCESS":"FAILED"));
    if(!result)
    {
        // close(SERIAL_PORT_FS);
        // SERIAL_PORT_FS = serialport_init(serial_port, 9600);
        ROS_WRN("UNABLE TO WRITE");
        // serialport_writebyte(SERIAL_PORT_FS, char(254));
    }
    clear_cnter++;
    if (clear_cnter>=50)
    {
        tcflush(SERIAL_PORT_FS, TCIOFLUSH);//force to flush the buffer
        clear_cnter = 0;
    }

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
    //   int written_bytes = write(SERIAL_PORT_FS, payload, sizeof(packet));
    //   delete payload;
    //   payload = nullptr;
    //    */
    //   unsigned char *convert = (unsigned char*)&packet;
    //   int writeen_bytes = write(SERIAL_PORT_FS, convert, sizeof(convert));
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
    //   int read_bytes = read(SERIAL_PORT_FS, &read_buf, sizeof(read_buf));

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
  close(SERIAL_PORT_FS);
  ROS_INFO("Closed Serial Port");
  return 0;
}

int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
    
    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    // fd = open(serialport, O_RDWR | O_NONBLOCK );
    
    if (fd == -1)  {
        ROS_ERROR("serialport_init: Unable to open port ");
        return -1;
    }
    
    // Sets the read() function to return NOW and not wait for data to enter
    // buffer if there isn't anything there.
    fcntl(fd, F_SETFL, FNDELAY);

    //int iflags = TIOCM_DTR;
    //ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
    //ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR

    if (tcgetattr(fd, &toptions) < 0) {
        ROS_ERROR("serialport_init: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
    switch(baud) 
    {
      case 9600:   brate=B9600;   break;
      case 4800:   brate=B4800;   break;
      case 19200:  brate=B19200;  break;
      case 38400:  brate=B38400;  break;
      case 57600:  brate=B57600;  break;
      case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);
    cfsetspeed(&toptions,  brate);


    // toptions.c_cflag |= (CLOCAL | CREAD);
    // toptions.c_cflag &= ~PARENB;
    // toptions.c_cflag &= ~CSTOPB;
    // toptions.c_cflag &= ~CSIZE;
    // toptions.c_cflag |= CS8;
    // toptions.c_cflag &= ~CRTSCTS;
    // toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
    // toptions.c_oflag &= ~OPOST;



    // toptions.c_cflag = (toptions.c_cflag & ~CSIZE) | CS8;
    // toptions.c_iflag =  IGNBRK;
    // toptions.c_lflag = 0;
    // toptions.c_oflag = 0;
    // toptions.c_cflag |= CLOCAL | CREAD;
    // toptions.c_cc[VMIN] = 1;
    // toptions.c_cc[VTIME] = 5;
    // toptions.c_iflag &= ~(IXON|IXOFF|IXANY);
    // toptions.c_cflag &= ~(PARENB | PARODD);
    // toptions.c_cflag &= ~CSTOPB;
    /* Great reference for serial programming: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/ */
    // 8N1
    toptions.c_cflag = 0;
    toptions.c_iflag = 0;
    toptions.c_lflag = 0;
    toptions.c_oflag = 0;

    toptions.c_cflag &= ~PARENB; // no parity
    toptions.c_cflag &= ~CSTOPB; // no stop bit
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8; // 8 bit per byte
    // no flow control
    toptions.c_cflag &= ~CRTSCTS; // no flow control
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines

    //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset
    // no echo
    toptions.c_lflag &= ~ECHO; // Disable echo
    toptions.c_lflag &= ~ECHOE; // Disable erasure
    toptions.c_lflag &= ~ECHONL; // Disable new-line echo
    toptions.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw

    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    toptions.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    
    toptions.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    toptions.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // toptions.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // toptions.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

    see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;
    toptions.c_cc[VTIME] = 20;
    
    tcflush(fd, TCIFLUSH);
    if( tcsetattr(fd, TCSANOW, &toptions) != 0) {
        ROS_ERROR("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}


//
int serialport_writebyte(int fd, uint8_t b)
{
    int n = write(fd,&b,1);
    if( n!=1)
        return 0;
    return -1;
}

//
int serialport_write(int fd, uint8_t* str, int len)
{
    int n = write(fd, str, len);
    if( n!=len ) {
        ROS_ERROR("serialport_write: couldn't write whole string\n");
        return 0;
    }
    return 1;
}