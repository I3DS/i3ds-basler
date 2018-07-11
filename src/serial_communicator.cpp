#include <iostream>

#include <sys/poll.h>

#include <string.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include  <termios.h>
#include <errno.h>

#include <boost/program_options.hpp>

#include "../include/serial_communicator.hpp"


#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

namespace po = boost::program_options;
namespace logging = boost::log;


SerialCommunicator::SerialCommunicator (const char *serialPort)
{
  BOOST_LOG_TRIVIAL(info) << "Connecting to serial port: " << serialPort;

  memset (fds, 0, sizeof(pollfd));
  fds[0].fd = openSerial ((char *) serialPort);

}

SerialCommunicator::~SerialCommunicator ()
{
  BOOST_LOG_TRIVIAL(info) << "SerialCommunicator Destructor";
  closeSerialPort ();
}

void
SerialCommunicator::closeSerialPort ()
{
  BOOST_LOG_TRIVIAL(info) << "Closing Serial port";
  close (fds[0].fd);
}

void
SerialCommunicator::sendManualTrigger ()
{
  BOOST_LOG_TRIVIAL(info) << "Sending manual trigger";
  sendParameterString ("TR1");
}

/// This is a parameter string as described in the manual.
void
SerialCommunicator::sendParameterString (const char *parameterString)
{

  char buff[100];
  memset (buff, 0, sizeof(buff));
  BOOST_LOG_TRIVIAL(info) << "Sending parameter string: " << parameterString;

  char command[100];
  memset (command, 0, sizeof(command));
  strncpy (command, parameterString, 99);
  strcat (command, "\n");

  write (fds[0].fd, command, strlen (command));
  BOOST_LOG_TRIVIAL(info) << "strlen(command) " << strlen (command);
  tcdrain (fds[0].fd);

  // Waiting for response

  fds[0].events = POLLIN;
  fds[0].revents = 0;
  int n;
//  for (;;)
    {
      n = poll (fds, 1, 2000);

      if (n > 0)
	{
	  if (fds[0].revents & POLLIN)
	    { //got data, and look up which fd has data, but we just have one waiting for events
	      ssize_t length;
	      length = read (fds[0].fd, buff, sizeof(buff));
	      if (length == -1)
		{
		  // REMARK: Got "Resource temporary unavailable." sometimes.
		  // But, I think it disapaired when I removed a terminal listening to the same serial port.
		  BOOST_LOG_TRIVIAL(info) <<"Error read event: %s" << strerror (errno);
		}
	      buff[length] = 0;
	      BOOST_LOG_TRIVIAL(info) << "From poll: " << buff << "  Length:" << length;
	      if (buff[strlen (command) + 1] == '>')
		{
		  BOOST_LOG_TRIVIAL(info) << "Ok response.\n";
		}
	      else
		{
		  // Reformating error code to remove OK prompt at new line before sending it to client.
		  BOOST_LOG_TRIVIAL(info) << "Error in response: %s" << buff;
		  char *p;
		  p = strchr (buff, '\n');
		  *p = '\0';
		  BOOST_LOG_TRIVIAL(info) << "Error in response: %s" << buff;
		}
	    }
	  else
	    {
	      BOOST_LOG_TRIVIAL(info) <<"Other event type? Needs to be handed?";
	    }
	}
      else
	{
	  BOOST_LOG_TRIVIAL(info) << "No data within 2 seconds.";
	}
    }

}

//RTc,p,d,s,
//RTc,p,d,s,r (r is optional)
// Limits are described in manual
void
SerialCommunicator::sendConfigurationParameters (int c, float p, float d,
						 float s, float r)
{

  char buffer[100];

  BOOST_LOG_TRIVIAL(info) <<"Sending configuration parameters: " << "c:" << c << " "
      "p:" << p << " "
      "d:" << d << " ";

  if (r != -1.0)
    {
      BOOST_LOG_TRIVIAL(info) << "r: " << r;
    }


  if (r != -1.0)
    {

      sprintf (buffer, "RT%d,%g,%g,%g,%g", c, p, d, s, r);
    }
  else
    {
      sprintf (buffer, "RT%d,%g,%g,%g", c, p, d, s);
    }

  sendParameterString (buffer);

}

void
SerialCommunicator::setCommunicationParameters (int fd)
{
  BOOST_LOG_TRIVIAL(info) << "setConfiguration";
  struct termios Opt;
  tcgetattr (fd, &Opt);
  cfsetispeed (&Opt, B115200);
  cfsetospeed (&Opt, B115200);
  tcsetattr (fd, TCSANOW, &Opt);

  struct termios options;
  tcgetattr (fd, &options);
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  int retcode = tcsetattr (fd, TCSANOW, &options);
  BOOST_LOG_TRIVIAL(info) << "tcsetattr return code: " << retcode;
  return;
}

int
SerialCommunicator::openSerial (char *device)
{
  int fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY);  //| O_NOCTTY | O_NDELAY
  if (-1 == fd)
    {
      BOOST_LOG_TRIVIAL(info) << "Can't Open Serial Port";
      return -1;
    }
  else
    {
      setCommunicationParameters (fd);
      return fd;
    }

}
