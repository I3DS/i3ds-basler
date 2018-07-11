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

#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

namespace po = boost::program_options;
namespace logging = boost::log;

class SerialCommunicator
{
public:

  SerialCommunicator (const char *serialPort);
  ~SerialCommunicator ();

  void
  intialiseSerialPort ();
  void
  closeSerialPort ();
  void
  sendParameterString (const char *parameterString);
  void
  sendConfigurationParameters (int c, float p, float d, float s, float r);
  void
  sendManualTrigger ();

private:
  int
  openSerial (char *device);
  void
  setCommunicationParameters (int fd);

  struct pollfd fds[1];

};

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
						 float s, float r = -1.0)
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

// startup parameters
// set /dev
// Send manual trigger
// Send Configuration string
// send configuration parameters
// Report result?
// Standard parameters for rest of communication system.

int
main (int argc, char* argv[])
{
  std::string serialPort;
  std::string flashCommand;
  bool triggerFlash = false;
  bool dontRunProgram = false;
  std::vector<double> commandParameters;
  std::vector<double> remoteParameters;

  po::options_description desc ("Wide angle flash control options");
  desc.add_options () ("help,h", "Produce this message")
  //    ("node", po::value<unsigned int>(&node_id)->required(), "Node ID of camera")

  ("device,d", po::value<std::string> ()->default_value (DEFAULT_SERIAL_PORT),
   "Serial(USB) port the \"Wide angle flash\" is connected to")

  ("trigger-flash,t", "Trigger flash manually")

  ("command,c", po::value<std::string> (),
   "Manually send command string to interact with the flash.\n"
   "\tOne may use all command string described in the manual.\n"
   "\tE.g \"TR1\" for triggering flash.\n") (
      "remote,r",
      po::value < std::vector<double> > (&commandParameters)->multitoken (),
      "This is the command available remotely via i3ds."
      "Actually the used command is: \"RTc,p,d,s,r\"\n"
      "\tc=1 => Light strobe output.\n\tc=2 => Trigger output signal\n"
      "\tp= pulse width in milliseconds (0.01 to 3)\n"
      "\ts= setting in percent(0 to 100)\n"
      "\tr= re-trigger delay in milliseconds (an optional parameter)\n"
      "Settings are not keep during a power cycle.\n"
      "Use format: -c 1 0.1 200 100 12\n")

  //("vector_value", po::value<std::vector<double> >(&vecoption)->
  //     multitoken()->default_value(std::vector<double>{0, 1, 2}), "description");

  ("verbose,v", "Print verbose output") ("quite,q", "Quiet output");

  po::variables_map vm;
  po::store (po::parse_command_line (argc, argv, desc), vm);

  if (vm.count ("help"))
    {
      BOOST_LOG_TRIVIAL(info) << desc;
      return -1;
    }

  if (vm.count ("device"))
    {
      serialPort = vm["device"].as<std::string> ();
      BOOST_LOG_TRIVIAL(info) << "Using serial port: " << serialPort;
    }
  if (vm.count ("trigger-flash"))
    {
      // serialPort = vm["device"].as<std::string>();
      BOOST_LOG_TRIVIAL(info)<< "Manually trigger flash";
      triggerFlash = true;
      dontRunProgram = true;
    }

  if (vm.count ("command"))
    {
      flashCommand = vm["command"].as<std::string> ();
      BOOST_LOG_TRIVIAL(info) << "Manually send command to flash.";
      BOOST_LOG_TRIVIAL(info) << "Command:" << flashCommand;
      dontRunProgram = true;
    }

  if (vm.count ("remote"))
    {
      remoteParameters = vm["remote"].as<std::vector<double>> ();
      BOOST_LOG_TRIVIAL(info) << "Test remote parameters.";
      BOOST_LOG_TRIVIAL(info) <<  "Commandparameter[1]:" << flashCommand << remoteParameters[0];
      dontRunProgram = true;
    }

  if (vm.count ("quite"))
    {
      logging::core::get ()->set_filter (
	  logging::trivial::severity >= logging::trivial::warning);
    }
  else if (!vm.count ("verbose"))
    {
      logging::core::get ()->set_filter (
	  logging::trivial::severity >= logging::trivial::info);
    }

  po::notify (vm);

  /*	  i3ds::Context::Ptr context(i3ds::Context::Create());

   BOOST_LOG_TRIVIAL(info) << "Connecting to camera with node ID: " << node_id;
   i3ds::CameraClient camera(context, node_id);
   BOOST_LOG_TRIVIAL(trace) << "---> [OK]";
   */

  /*
   // Print config, this is the final command.
   if (vm.count("print"))
   {
   print_camera_settings(&camera);
   }
   */

  BOOST_LOG_TRIVIAL(info) << argv[0];

  SerialCommunicator *serialCommunicator = new SerialCommunicator (
      serialPort.c_str ());

  if (!flashCommand.empty ())
    {
      serialCommunicator->sendParameterString (flashCommand.c_str ());
    }

  if (!remoteParameters.empty ())
    {
      if (remoteParameters.size () == 4)
	{
      serialCommunicator->sendConfigurationParameters (
	  remoteParameters[0],
	  remoteParameters[1],
	  remoteParameters[2],
	  remoteParameters[3]
      );
    }
  if (remoteParameters.size () == 5)
    {
  serialCommunicator->sendConfigurationParameters (
      remoteParameters[0],
      remoteParameters[1],
      remoteParameters[2],
      remoteParameters[3],
      remoteParameters[4]
  );
}
}

if (triggerFlash)
{
serialCommunicator->sendManualTrigger ();
}

if (dontRunProgram)
{
return 0;
}
/*
 serialCommunicator->sendParameterString ("RT1,3,200,10.0");

 serialCommunicator->sendManualTrigger ();
 serialCommunicator->sendParameterString ("RT1d3,300.2,1d");
 sleep(1);
 serialCommunicator->sendManualTrigger ();
 sleep(1);

 serialCommunicator->sendConfigurationParameters (1, 0.2, 0.01, 90);
 sleep(1);
 serialCommunicator->sendManualTrigger ();
 */
serialCommunicator->closeSerialPort ();

return 0;
}
