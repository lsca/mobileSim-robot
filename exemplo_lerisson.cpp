#include "Aria.h"
int main(int argc, char** argv)
{
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

//Inicializa robo
 robot.runAsync(true);

//Habilita motores
robot.comInt(ArCommands::ENABLE,1);
printf("Entre com a distancia minima da parede: ");
scanf("%d",&dist);
printf ("Entre com a velocidade: ");
scanf("%d",&vel);

//Determina a velocidade máxima de translacao do robo
robot.setTransVelMax(vel);
//Faz com que o robo se movimente com velocidade vel
robot.setVel(vel);
 //Realiza medida do menor valor apresentado pelos sonares frontais
valorsonar=robot.getClosestSonarRange(-90,90);
while(true){
//Realiza medidas do sonar ate que se torne menor do que dist
while(valorsonar>dist)
valorsonar=robot.getClosestSonarRange(-90,90);
//Pára o robo
robot.stop();
//Faz com que o robo retorne 500mm
robot.move(-500);
//Aguarda realizacao do movimento
while(!robot.isMoveDone());
//Faz com que o robô gire 45 graus
robot.setDeltaHeading(45);
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
