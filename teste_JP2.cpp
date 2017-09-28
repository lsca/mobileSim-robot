#include "Aria.h"
#include <iostream>
using namespace std;
int main(int argc, char** argv)
{
  double xInicial_Mapa,yInicial_Mapa,angulo,xFinal_Mapa,yFinal_Mapa;
  cout << "O programa comeca agora!!" << endl;
  cin >> xInicial_Mapa;
  cin >> yInicial_Mapa;
  cin >> angulo;
  cin >> xFinal_Mapa;
  cin >> yFinal_Mapa;
  printf("X inicial: %.1f \nY inicial: %.1f\nGrau: %.1f \nX Final: %.1f \nY Final: %.1f\n",xInicial_Mapa,yInicial_Mapa,angulo,xFinal_Mapa,yFinal_Mapa);
//Inicio obrigatorio do Aria
 Aria::init();
//Objeto conexão(simpleConnector) com simulador ou robô
ArSimpleConnector simpleConnector(&argc,argv);
//Objeto robo(robot)
 ArRobot robot;
//Testa argumentos
 if(!simpleConnector.parseArgs() || argc>1)
{
simpleConnector.logOptions();
exit(1);
}
//Testa conexao com simulador ou robo
if(!simpleConnector.connectRobot(&robot))
{
printf("Nao conseguiu conectar ao robo\n");
Aria::exit(1);
}
//Cria objeto sonar
ArSonarDevice sonar;
//Conecta o sonar ao robo
 robot.addRangeDevice(&sonar);
//Inicializa variaveis de velocidade(vel), distancia(dist) e leiturade sonar(valorsonar)
 int vel,valorsonar,dist;
vel = 200;
dist = 200;
//Inicializa robo
 robot.runAsync(true);

//Habilita motores
robot.comInt(ArCommands::ENABLE,1);

//Determina a velocidade máxima de translacao do robo
robot.setTransVelMax(vel);
//Faz com que o robo se movimente com velocidade vel
robot.setVel(vel);
 //Realiza medida do menor valor apresentado pelos sonares frontais
valorsonar=robot.getClosestSonarRange(-90,90);
//Seta a posição do robô em relação ao mapa
cout << "Posicao X inicial do robo antes do set: " << robot.getX() << endl;
cout << "Posicao Y inicial do robo antes do set: " << robot.getY() << endl;
cout << "Transformando posicao do robo em relacao ao mapa..." << endl;
robot.moveTo(ArPose(xInicial_Mapa,yInicial_Mapa,angulo),ArPose(robot.getX(),robot.getY(),robot.getTh()));
cout << "Posicao X inicial do robo depois do set: " << robot.getX() << endl;
cout << "Posicao Y inicial do robo depois do set: " << robot.getY() << endl;
cout << "Posicao X de destino: " << xFinal_Mapa << endl;
cout << "Posicao Y de destino: " << yFinal_Mapa << endl;
cout << "\n";
int direcao; //-1 => Esquerda; 1 => Direita
if(xFinal_Mapa > xInicial_Mapa){
  direcao = 1;
}
else{
  direcao = -1;
}
switch (direcao) {
  case 1:
      robot.stop();
      robot.setDeltaHeading(robot.getTh()*-1);
      while(!robot.isHeadingDone());
      robot.setVel(vel);
      valorsonar=robot.getClosestSonarRange(-90,90);
      robot.moveTo(ArPose(robot.getX(),robot.getY(),robot.getTh()));
      break;
  case -1:
      robot.stop();
      robot.setDeltaHeading(180-robot.getTh());
      while(!robot.isHeadingDone());
      robot.setVel(vel);
      valorsonar=robot.getClosestSonarRange(-90,90);
      robot.moveTo(ArPose(robot.getX(),robot.getY(),robot.getTh()));
      break;
}
while(true){
//Realiza medidas do sonar ate que se torne menor do que dist
while(valorsonar>dist){
  valorsonar=robot.getClosestSonarRange(-90,90);
}

//Pára o robo
robot.stop();
cout << "Posicao X atual do robo: " << robot.getX() << endl;
cout << "Posicao Y atual do robo: " << robot.getY() << endl;
cout << "\n";
//Faz com que o robo retorne 500mm
robot.move(-300);
//Aguarda realizacao do movimento
while(!robot.isMoveDone());
//Faz com que o robô gire 90 graus
robot.setDeltaHeading(90);
//Aguarda termino de giro
while(!robot.isHeadingDone());

//Faz com que o robo se movimente com velocidade vel
robot.setVel(vel);
//Realiza menor medida dos sonares frontais
valorsonar=robot.getClosestSonarRange(-90,90);
robot.moveTo(ArPose(robot.getX(),robot.getY(),robot.getTh()));
}
Aria::exit(0);
}
