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

  double xInicial_Mapa,yInicial_Mapa,angulo,xFinal_Mapa,yFinal_Mapa;
  cout << "O programa comeca agora!!" << endl;
  cin >> xInicial_Mapa;
  cin >> yInicial_Mapa;
  cin >> angulo;
  cin >> xFinal_Mapa;
  cin >> yFinal_Mapa;
  printf("X inicial: %.1f \nY inicial: %.1f\nGrau: %.1f \nX Final: %.1f \nY Final: %.1f\n",xInicial_Mapa,yInicial_Mapa,angulo,xFinal_Mapa,yFinal_Mapa);

  robot.runAsync(true); //Roda no modo assicrono

  robot.lock(); // Bloqueia o robô durante a montagem
  robot.comInt(ArCommands::ENABLE, 1); //Liga os motores
  robot.setVel(0); //Seta a velocidade de translação para 200 mm/s
  robot.setRotVel(20); //Seta a velocidade rotacional para 20 graus/s
  robot.setVel2(200,200); //Seta a velocidade das rodas para 200 mm/s
  robot.setHeading(0); //30 graus relativo a posição inicial
  robot.setDeltaHeading(0); //60 graus relativo a orientação atual
  robot.unlock(); //Desbloqueia o robô

  cout << "Posicao X inicial do robo antes do set: " << robot.getX() << endl;
  cout << "Posicao Y inicial do robo antes do set: " << robot.getY() << endl;
  cout << "Transformando posicao do robo em relacao ao mapa..." << endl;
  robot.moveTo(ArPose(xInicial_Mapa,yInicial_Mapa,angulo),ArPose(robot.getX(),robot.getY(),robot.getTh()));
  cout << "Posicao X inicial do robo depois do set: " << robot.getX() << endl;
  cout << "Posicao Y inicial do robo depois do set: " << robot.getY() << endl;

  /*int count = 1;
  while (true) {
    robot.move(50); //Move 200 mm/s pra frente
    if(count%10000 == 0){
    cout << "Posicao X em relacao a ele: " << robot.getX() << endl;
    cout << "Posicao Y em relacao a ele: " << robot.getY() << endl << endl;
    }
    count++;
  }*/

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
