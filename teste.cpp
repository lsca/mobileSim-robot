#include "Aria.h"
#include <iostream>
using namespace std;

int main(int argc, char **argv)
{
  Aria::init();
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();
  ArRobot robot;

  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "simpleConnect: Could not connect to the robot.");
    if(parser.checkHelpAndWarnUnparsed())
    {
        // -help not given
        Aria::logOptions();
        Aria::exit(1);
    }
  }

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

  ArLog::log(ArLog::Normal, "simpleConnect: Connected to robot.");

  double xi,yi,g,xf,yf;
  cout << "O programa comeca agora!!" << endl;
  cin >> xi;
  cin >> yi;
  cin >> g;
  cin >> xf;
  cin >> yf;
  printf("X inicial: %.1f \nY inicial: %.1f\nGrau: %.1f \nX Final: %.1f \nY Final: %.1f\n",xi,yi,g,xf,yf);
  /*robot.enableMotors();
  robot.runAsync(true);
  robot.lock();
  bool soc = robot.hasStateOfCharge();

  float battv = 0.0;
  if(soc)
    battv = robot.getStateOfCharge();
  else
    battv = robot.getBatteryVoltage();

  ArLog::log(ArLog::Normal, "simpleConnect: Pose=(%.2f,%.2f,%.2f), Trans.  Vel=%.2f, Battery=%.2f%c",
    robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), battv, soc?'%':'V');
  robot.unlock();

  robot.stopRunning();
  robot.waitForRunExit();*/
  Aria::exit(0);
  return 0;
}
