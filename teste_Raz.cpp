#include "Aria.h"
#include <iostream>
#include <vector>
#include <map>
#include <math.h>
#define pb push_back
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

  //struct que representa um nó que possui os atributos de coordenadas x e y para uma instância
  struct no_coord{
    int x;
    int y;
  };

  //estruturas de dados necessarias para implementação do A*
  /*vector<no_coord> closedSet;
  vector<no_coord> openSet;
  map<no_coord,no_coord> cameFrom;
  map<no_coord,int> g_Score;
  map<no_coord,int> f_Score;
  map<no_coord,vector<no_coord> > grafo;*/

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


  /*no_coord start;
  start.x = robot.getX();
  start.y = robot.getY();

  openSet.pb(start);
  g_Score[start] = 0;
  implementar função heuristica para essa atribuição local do map
  f_Score[start] = heuristic_cost_estimate(start,goal);



  grafo[start].pb(inserção de N pontos possiveis de atingir a partir de um determinado ponto atual);

  while(!openSet.size() == 0){
    no_coord atual = openSet[0];
    int apagar;
    for(int i=0;i<openSet.size();++i){
        if(f_Score[openSet[i]] < f_Score[atual]) atual = openSet[i]; apagar = i;
    }
    if(atual == objetivo) return reconstruct_path(cameFrom,)
    openSet.erase(openSet.begin()+apagar);
    closedSet.pb(atual);

    for(int i=0;i<grafo[atual].size();++i){
      vizinho = grafo[atual][i];
      if(in(closedSet,vizinho) continue;
      if(!in(openSet,vizinho) openSet.pb(vizinho);
      tentative_gScore = g_Score[atual] + dist_between(atual,vizinho); //dist_between() é uma função a ser feita ainda
                                                                       // que calcula a distancia entre dois pontos
      if(tentative_gScore >= g_Score[vizinho]) continue;

      cameFrom[vizinho] = atual;
      g_Score[vizinho] = tentative_gScore;
      f_Score[vizinho] = g_Score[vizinho] + heuristic_cost_estimate(vizinho,objetivo);
    }
  }*/


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

int heuristic_cost_estimate(no_coord noI, no_coord noD){
  return sqrt(pow(noI.x - noD.x,2)+pow(noI.y - noD.y,2));
}
int dist_between(no_coord no1, no_coord no2){
  return sqrt(pow(no1.x - no2.x,2)+pow(no1.y - no2.y,2));
}


bool in(vector<no_coord> vec, no_coord valor){
  for(int i=0;i<vec.size();++i){
    if(vec[i] == valor) return 1;
  }
  return 0;
}
