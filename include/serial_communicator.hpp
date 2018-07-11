#include <sys/poll.h>



class SerialCommunicator
{
public:

  SerialCommunicator (const char *serialPort);
  ~SerialCommunicator ();

  void intialiseSerialPort ();
  void closeSerialPort ();
  void sendParameterString (const char *parameterString);
  void sendConfigurationParameters (int c, float p, float d, float s, float r = -1.0);
  void sendManualTrigger ();

private:
  int openSerial (char *device);
  void setCommunicationParameters (int fd);

  struct pollfd fds[1];

};
