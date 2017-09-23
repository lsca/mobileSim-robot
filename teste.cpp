#include "Aria.h"

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

  robot.enableMotors();
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
  robot.waitForRunExit();
  Aria::exit(0);
  return 0;
}

