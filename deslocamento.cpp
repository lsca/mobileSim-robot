#include "Aria.h"
#include "Dstar.h"
#include <iostream>
using namespace std;
#define itera list<state>::iterator

Dstar::Dstar() {

  maxSteps = 80000;  // expansão de nós antes de desistir
  C1       = 1;      // custo de uma célula invisível

}

/* flutuante Dstar :: keyHashCode (state u)
 * --------------------------
 * Retorna o código de hash da chave para o estado u, isso é usado para comparar
 * um estado que foi atualizado
 */
float Dstar::keyHashCode(state u) {

  return (float)(u.k.first + 1193*u.k.second);

}

/* bool Dstar :: isValid (state u)
 * --------------------------
 * Retorna verdadeiro se o estado estiver na lista aberta ou não, verificando se
 * está na tabela hash.
 */
bool Dstar::isValid(state u) {

  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return false;
  if (!close(keyHashCode(u), cur->second)) return false;
  return true;

}

/* void Dstar::getPath()
 * --------------------------
 * Retorna o caminho criado pelo replan ()
 */
list<state> Dstar::getPath() {
  return path;
}

/* bool Dstar::occupied(state u)
 * --------------------------
 * retorna verdadeira se a célula estiver ocupada (não transpassável), falso
 * de outra forma. não atraentes são marcados com um custo <0.
 */
bool Dstar::occupied(state u) {

  ds_ch::iterator cur = cellHash.find(u);

  if (cur == cellHash.end()) return false;
  return (cur->second.cost < 0);
}

/* void Dstar::init(int sX, int sY, int gX, int gY)
 * --------------------------
 * 
 * Init dstar com coordenadas de início e objetivo, o descanso é como por
 * [S. Koenig, 2002]
 */
void Dstar::init(int sX, int sY, int gX, int gY) {

  cellHash.clear();
  path.clear();
  openHash.clear();
  while(!openList.empty()) openList.pop();

  k_m = 0;

  s_start.x = sX;
  s_start.y = sY;
  s_goal.x  = gX;
  s_goal.y  = gY;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;

}
/* void Dstar::makeNewCell(state u)
 * --------------------------
 * Verifica se uma célula está na tabela de hash, se não a adiciona.
 */
void Dstar::makeNewCell(state u) {

  if (cellHash.find(u) != cellHash.end()) return;

  cellInfo tmp;
  tmp.g       = tmp.rhs = heuristic(u,s_goal);
  tmp.cost    = C1;
  cellHash[u] = tmp;

}

/* double Dstar::getG(state u)
 * --------------------------
 * Retorna o valor G para o estado u.
 */
double Dstar::getG(state u) {

  if (cellHash.find(u) == cellHash.end())
    return heuristic(u,s_goal);
  return cellHash[u].g;

}

/* double Dstar::getRHS(state u)
 * --------------------------
 * Retorna o valor rhs para o estado u.
 */
double Dstar::getRHS(state u) {

  if (u == s_goal) return 0;

  if (cellHash.find(u) == cellHash.end())
    return heuristic(u,s_goal);
  return cellHash[u].rhs;

}

/* void Dstar::setG(state u, double g)
 * --------------------------
 * Define o valor G para o estado
 */
void Dstar::setG(state u, double g) {

  makeNewCell(u);
  cellHash[u].g = g;
}

/* void Dstar::setRHS(state u, double rhs)
 * --------------------------
 * 
Define o valor de rhs para o estado u
 */
double Dstar::setRHS(state u, double rhs) {

  makeNewCell(u);
  cellHash[u].rhs = rhs;

}

/* double Dstar::eightCondist(state a, state b)
 * --------------------------
 * 
Retorna a distância de 8 vias entre o estado a e o estado b.
 */
double Dstar::eightCondist(state a, state b) {
  double temp;
  double min = abs(a.x - b.x);
  double max = abs(a.y - b.y);
  if (min > max) {
    double temp = min;
    min = max;
    max = temp;
  }
  return ((M_SQRT2-1.0)*min + max);
}

/* int Dstar::computeShortestPath()
 * --------------------------
 * Conforme [S. Koenig, 2002] com exceção de 2 modificações principais:
 * 1. Paramos de planejar após uma série de etapas, 'maxsteps' fazemos isso
 * porque este algoritmo pode planear para sempre se o início for
 * cercado por obstáculos.
 * 2. Nós preguiçosamente removemos os estados da lista aberta, então nunca precisamos
 * iterar através dele.
 */
int Dstar::computeShortestPath() {

  list<state> s;
  list<state>::iterator i;

  if (openList.empty()) return 1;

  int k=0;
  while ((!openList.empty()) &&
         (openList.top() < (s_start = calculateKey(s_start))) ||
         (getRHS(s_start) != getG(s_start))) {

    if (k++ > maxSteps) {
      fprintf(stderr, "At maxsteps\n");
      return -1;
    }


    state u;

    bool test = (getRHS(s_start) != getG(s_start));

    // remove preguiçosos
    while(1) {
      if (openList.empty()) return 1;
      u = openList.top();
      openList.pop();

      if (!isValid(u)) continue;
      if (!(u < s_start) && (!test)) return 2;
      break;
    }

    ds_oh::iterator cur = openHash.find(u);
    openHash.erase(cur);

    state k_old = u;

    if (k_old < calculateKey(u)) { // u está desatualizado
      insert(u);
    } else if (getG(u) > getRHS(u)) { // precisa de atualização (melhorou)
      setG(u,getRHS(u));
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
    } else {   // g <= rhs, o estado piorou
      setG(u,INFINITY);
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
      updateVertex(u);
    }
  }
  return 0;
}

/* bool Dstar::close(double x, double y)
 * --------------------------
 * 
Retorna verdadeiro se x e y estiverem dentro de 10E-5, falso caso contrário
 */
bool Dstar::close(double x, double y) {

  if (isinf(x) && isinf(y)) return true;
  return (fabs(x-y) < 0.00001);

}

/* void Dstar::updateVertex(state u)
 * --------------------------
 * Conforme [S. Koenig, 2002]
 */
void Dstar::updateVertex(state u) {

  list<state> s;
  list<state>::iterator i;

  if (u != s_goal) {
    getSucc(u,s);
    double tmp = INFINITY;
    double tmp2;

    for (i=s.begin();i != s.end(); i++) {
      tmp2 = getG(*i) + cost(u,*i);
      if (tmp2 < tmp) tmp = tmp2;
    }
    if (!close(getRHS(u),tmp)) setRHS(u,tmp);
  }

  if (!close(getG(u),getRHS(u))) insert(u);

}

/* void Dstar::insert(state u)
 * --------------------------
 * Insere o estado na OpenList e no openHash.
 */
void Dstar::insert(state u) {

  ds_oh::iterator cur;
  float csum;

  u    = calculateKey(u);
  cur  = openHash.find(u);
  csum = keyHashCode(u);
  // retorna se a célula já estiver na lista. TODO: isso deve ser
  // descomente, exceto que ele apresenta um bug, eu suspeito que há um
  // bug em outro lugar e tendo duplicatas na fila openList
  // esconde o problema ...
  //if ((cur != openHash.end()) && (close(csum,cur->second))) return;

  openHash[u] = csum;
  openList.push(u);
}

/* void Dstar::remove(state u)
 * --------------------------
 * 
Remove o estado do openHash. O estado é removido do
 * openList lazilily (no replan) para economizar computação.
 */
void Dstar::remove(state u) {

  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return;
  openHash.erase(cur);
}


/* double Dstar::trueDist(state a, state b)
 * --------------------------
 * Custo euclidiano entre estado a e estado b.
 */
double Dstar::trueDist(state a, state b) {

  float x = a.x-b.x;
  float y = a.y-b.y;
  return sqrt(x*x + y*y);

}

/* double Dstar::heuristic(state a, state b)
 * --------------------------
 * Pretentido auto explicativo, o herestico que usamos é a distância de 8 vias
 * escalado por uma constante C1 (deve ser configurado para <= custo mínimo).
 */
double Dstar::heuristic(state a, state b) {
  return eightCondist(a,b)*C1;
}

/* state Dstar::calculateKey(state u)
 * --------------------------
 * Conforme [S. Koenig, 2002]
 */
state Dstar::calculateKey(state u) {

  double val = fmin(getRHS(u),getG(u));

  u.k.first  = val + heuristic(u,s_start) + k_m;
  u.k.second = val;

  return u;

}

/* double Dstar::cost(state a, state b)
 * --------------------------
 * Retorna o custo de mover do estado a para o estado b. Isto pode ser
 * ou o custo de deslocamento do estado a ou para o estado b, fomos com
 * o antigo. Este é também o custo de 8 vias.
 */
double Dstar::cost(state a, state b) {

  int xd = abs(a.x-b.x);
  int yd = abs(a.y-b.y);
  double scale = 1;

  if (xd+yd>1) scale = M_SQRT2;

  if (cellHash.count(a) == 0) return scale*C1;
  return scale*cellHash[a].cost;

}
/* void Dstar::updateCell(int x, int y, double val)
 * --------------------------
 * Conforme [S. Koenig, 2002]
 */
void Dstar::updateCell(int x, int y, double val) {

   state u;

  u.x = x;
  u.y = y;

  if ((u == s_start) || (u == s_goal)) return;

  makeNewCell(u);
  cellHash[u].cost = val;

  updateVertex(u);
}

/* void Dstar::getSucc(state u,list<state> &s)
 * --------------------------
 * Retorna uma lista de estados sucessores para o estado, uma vez que este é um
 * Gráfico de 8 vias, esta lista contém todos os vizinhos das células. A menos que
 * a célula está ocupada, caso em que não tem sucessores.
 */
void Dstar::getSucc(state u,list<state> &s) {

  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  if (occupied(u)) return;

  u.x += 1;
  s.push_front(u);
  u.y += 1;
  s.push_front(u);
  u.x -= 1;
  s.push_front(u);
  u.x -= 1;
  s.push_front(u);
  u.y -= 1;
  s.push_front(u);
  u.y -= 1;
  s.push_front(u);
  u.x += 1;
  s.push_front(u);
  u.x += 1;
  s.push_front(u);

}

/* void Dstar::getPred(state u,list<state> &s)
 * --------------------------
 * Retorna uma lista de todos os estados predecessores para o estado. Desde a
 * isto é para um gráfico conectado de 8 direções, a lista contém todos os
 * vizinhos para o estado u. Os vizinhos ocupados não são adicionados ao
 * Lista.
 */
void Dstar::getPred(state u,list<state> &s) {

  s.clear();
  u.k.second = -1;
  u.k.first  = -1;
  u.x += 1;

  u.y += 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  if (!occupied(u)) s.push_front(u);

}

/* void Dstar::updateStart(int x, int y)
 * --------------------------
 *
Atualize a posição do robô, isso não força um replan.
 */
void Dstar::updateStart(int x, int y) {

  s_start.x = x;
  s_start.y = y;

  k_m += heuristic(s_last,s_start);

  s_start = calculateKey(s_start);
  s_last  = s_start;

}

/* void Dstar::updateGoal(int x, int y)
 * --------------------------
 * Isso é um pouco de um hack, para mudar a posição do objetivo que nós
 * primeiro salve todos os não vazios no mapa, limpe o mapa, mova o
 * meta e re-adicionar todas as células não vazias. Como a maioria dessas células
 * não estão entre o começo e o objetivo, isso não parece doer
 * desempenho demais. Também é uma grande quantidade de memória que nós
 * provavelmente não usa mais.
 */
void Dstar::updateGoal(int x, int y) {

  list< pair<ipoint2, double> > toAdd;
  pair<ipoint2, double> tp;

  ds_ch::iterator i;
  list< pair<ipoint2, double> >::iterator kk;

  if (!close(i->second.cost, C1)) {
  for(i=cellHash.begin(); i!=cellHash.end(); i++) {
      tp.first.x = i->first.x;
      tp.first.y = i->first.y;
      tp.second = i->second.cost;
      toAdd.push_back(tp);
    }
  }

  cellHash.clear();
  openHash.clear();

  while(!openList.empty())
    openList.pop();

  k_m = 0;

  s_goal.x  = x;
  s_goal.y  = y;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;

  for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
    updateCell(kk->first.x, kk->first.y, kk->second);
  }


}

/* bool Dstar::replan()
 * --------------------------
 * 
Atualiza os custos para todas as células e calcula o caminho mais curto para
 * meta. Retorna verdadeiro se um caminho for encontrado, falso caso contrário. O caminho é
 * calculado fazendo uma busca gananciosa sobre os valores de custo + g em cada
 * células. Para resolver o problema do robô, tome um
 * caminho que está perto de um ângulo de 45 graus para atingir, nós rompemos laços com base em
 * a métrica euclidiana (estado, objetivo) + euclidiana (estado, início).
 */
bool Dstar::replan() {

  path.clear();

  int res = computeShortestPath();
  //printf("res: %d ols: %d ohs: %d tk: [%f %f] sk: [%f %f] sgr: (%f,%f)\n",res,openList.size(),openHash.size(),openList.top().k.first,openList.top().k.second, s_start.k.first, s_start.k.second,getRHS(s_start),getG(s_start));
  if (res < 0) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }
  list<state> n;
  list<state>::iterator i;

  state cur = s_start;

  if (isinf(getG(s_start))) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }

  while(cur != s_goal) {

    path.push_back(cur);
    getSucc(cur, n);

    if (n.empty()) {
      fprintf(stderr, "NO PATH TO GOAL\n");
      return false;
    }

    double cmin = INFINITY;
    double tmin;
    state smin;

    for (i=n.begin(); i!=n.end(); i++) {

      //if (occupied(*i)) continue;
      double val  = cost(cur,*i);
      double val2 = trueDist(*i,s_goal) + trueDist(s_start,*i); // (Euclidean) cost to goal + cost to pred
      val += getG(*i);

      if (close(val,cmin)) {
        if (tmin > val2) {
          tmin = val2;
          cmin = val;
          smin = *i;
        }
      } else if (val < cmin) {
        tmin = val2;
        cmin = val;
        smin = *i;
      }
    }
    n.clear();
    cur = smin;
  }
  path.push_back(s_goal);
  return true;
}

int rotacao(int th_atual, int x_atual, int y_atual, int x_final, int y_final)
{
  int th_final;
  if(th_atual == 0 || th_atual == 360 || th_atual == 180)
  {
    if(y_final > y_atual)
    {
      th_final = 90 - th_atual;
    }
    else
    {
      th_final = 270 - th_atual;
    }
  }
  else
  {
    if(x_final > x_atual)
    {
      th_final = 360 - th_atual;
    }
    else
    {
      th_final = 180 - th_atual;
    }
  }
  return th_final;
}

int main(int argc, char** argv)
{
  double xInicial_Mapa,yInicial_Mapa,angulo,xFinal_Mapa,yFinal_Mapa;
  cin >> xInicial_Mapa;
  cin >> yInicial_Mapa;
  cin >> angulo;
  cin >> xFinal_Mapa;
  cin >> yFinal_Mapa;

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
  vel = 0;
  dist = 600;
  //Inicializa robo
   robot.runAsync(true);

  //Habilita motores
  robot.comInt(ArCommands::ENABLE,1);

  //Determina a velocidade máxima de translacao do robo
  //robot.setTransVelMax(vel);
  //Faz com que o robo se movimente com velocidade vel
  robot.setVel(vel);
   //Realiza medida do menor valor apresentado pelos sonares frontais
  valorsonar=robot.getClosestSonarRange(-30,30);
  //Seta a posição do robô em relação ao mapa
  robot.moveTo(ArPose(xInicial_Mapa,yInicial_Mapa,angulo),ArPose(robot.getX(),robot.getY(),robot.getTh()));

  Dstar *dstar = new Dstar();
  list<state> mypath;
  dstar->init(xInicial_Mapa,yInicial_Mapa,xFinal_Mapa,yFinal_Mapa);
  dstar->replan();               // plan a path
  mypath = dstar->getPath();     // retrieve path

  while(!(xFinal_Mapa-500 < robot.getX() && robot.getX() < xFinal_Mapa + 500)
  && !(yFinal_Mapa-500 < robot.getY() && robot.getY() < yFinal_Mapa + 500))
  {
    int count = 1;
    itera i = mypath.begin();
    valorsonar=robot.getClosestSonarRange(-30,30);
    while(i!=mypath.end()){
      if(count % 500 == 0)
      {
        if(valorsonar<=dist)
        {
          robot.setDeltaHeading(rotacao(robot.getTh(),robot.getX(),robot.getY(),xFinal_Mapa,yFinal_Mapa));
          while(!robot.isHeadingDone());
          if(valorsonar>dist)
          {
            robot.move(500);
            while(!robot.isMoveDone());
          }
          dstar->updateStart(robot.getX(),robot.getY());
          dstar->replan();               // plan a path
          mypath = dstar->getPath();     // retrieve path
          i = mypath.begin();
        }
        else
        {
          if(i->x > robot.getX()){ //Se o X destino for maior que o X atual, gira o robo pra grau 0 e anda 1mm
            robot.setDeltaHeading(360-robot.getTh());
            while(!robot.isHeadingDone());
            valorsonar=robot.getClosestSonarRange(-30,30);
            if(valorsonar<=dist)
            {
              robot.setDeltaHeading(rotacao(robot.getTh(),robot.getX(),robot.getY(),xFinal_Mapa,yFinal_Mapa));
              while(!robot.isHeadingDone());
            }
            else{
              robot.move(500);
              while(!robot.isMoveDone());
            }
          }
          else if(i->x < robot.getX()){ //Se o X destino for menor que o X atual, gira o robo pra grau 180 e anda 1mm
            robot.setDeltaHeading(180-robot.getTh());
            while(!robot.isHeadingDone());
            valorsonar=robot.getClosestSonarRange(-30,30);
            if(valorsonar<=dist)
            {
              robot.setDeltaHeading(rotacao(robot.getTh(),robot.getX(),robot.getY(),xFinal_Mapa,yFinal_Mapa));
              while(!robot.isHeadingDone());
            }
            else{
              robot.move(500);
              while(!robot.isMoveDone());
            }
          }
          if(i->y > robot.getY()){ //Se o Y destino for maior que o Y atual, gira o robo para grau 90 e anda 1mm
            robot.setDeltaHeading(90-robot.getTh());
            while(!robot.isHeadingDone());
            valorsonar=robot.getClosestSonarRange(-30,30);
            if(valorsonar<=dist)
            {
              robot.setDeltaHeading(rotacao(robot.getTh(),robot.getX(),robot.getY(),xFinal_Mapa,yFinal_Mapa));
              while(!robot.isHeadingDone());
            }
            else{
              robot.move(500);
              while(!robot.isMoveDone());
            }
          }
          else if(i->y < robot.getY()){ //Se o Y destino for maior que o Y atual, gira o robo para grau 270 e anda 1mm
            robot.setDeltaHeading(270-robot.getTh());
            while(!robot.isHeadingDone());
            valorsonar=robot.getClosestSonarRange(-30,30);
            if(valorsonar<=dist)
            {
              robot.setDeltaHeading(rotacao(robot.getTh(),robot.getX(),robot.getY(),xFinal_Mapa,yFinal_Mapa));
              while(!robot.isHeadingDone());
            }
            else{
              robot.move(500);
              while(!robot.isMoveDone());
            }
          }
        }
        valorsonar=robot.getClosestSonarRange(-30,30);
      }
      i++;
      count++;
    }
    dstar->init(robot.getX(),robot.getY(),xFinal_Mapa,yFinal_Mapa);
    dstar->replan();               // plan a path
    mypath = dstar->getPath();     // retrieve path
  }
  Aria::exit(1);
  return 0;
}
