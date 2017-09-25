#include "Aria.h"
#include <iostream>
using namespace std;

/*
*--------------------------------------------------------
* Método Principal
*--------------------------------------------------------
*/
int main(int argc, char **argv)
{
  /*O Robô e seus devices*/
  Aria::init(); //Inicializa a biblioteca ARIA
  ArRobot robot; //Instancia o robô
  ArSonarDevice sonar; //Instancia o sonar
  robot.addRangeDevice(&sonar); //Adiciona o sonar ao robô

  ArArgumentParser parser(&argc, argv); //Instancia os argumentos no parser(analizador)

  /*Conexão do robô*/
  ArRobotConnector robotConnector(&parser, &robot);
  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "actionExample: Could not connect to the robot.");
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

  ArLog::log(ArLog::Normal, "actionExample: Connected to robot.");

  robot.runAsync(true); //Roda no modo assicrono

  robot.lock(); // Bloqueia o robô durante a montagem
  robot.comInt(ArCommands::ENABLE, 1); //Liga os motores
  robot.setVel(0); //Seta a velocidade de translação para 200 mm/s
  robot.setRotVel(20); //Seta a velocidade rotacional para 20 graus/s
  robot.setVel2(200,200); //Seta a velocidade das rodas para 200 mm/s
  robot.setHeading(0); //30 graus relativo a posição inicial
  robot.setDeltaHeading(0); //60 graus relativo a orientação atual
  robot.unlock(); //Desbloqueia o robô

  while (true) {
    robot.move(50); //Move 200m pra frente
  }


  Aria::exit(0); //Sai do Aria
}
