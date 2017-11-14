/*
 *    Copyright (C) 2017 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <math.h>   


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
    state=State::IDLE;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    
    innerModel = new InnerModel("/home/svenminds/robocomp/files/innermodel/simpleworld.xml");
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
  try
  {
      
      RoboCompLaser::TLaserData tLaser = laser_proxy->getLaserData(); 
      RoboCompDifferentialRobot::TBaseState bState;
      differentialrobot_proxy->getBaseState(bState);
      innerModel->updateTransformValues("base",bState.x,0,bState.z,0,bState.alpha,0);
      QVec ini;
        
      switch(state)
      {
          
           case State::IDLE:
               if (pick.isActive())
               {
                   qDebug()<<"Paso1";
                   ini=QVec::vec3(bState.x,0,bState.z);
                   linea=QLine2D(ini, pick.getAux());
                   state = State::GOTO;
               }
               break;
           case State::GOTO:
               qDebug()<<"Paso2";
               gotoTarget(tLaser);             
               break;
           case State::TURN:
               qDebug()<<"Paso3";
               turne(tLaser,bState);
               break;
           case State::BUG:
               qDebug()<<"Paso4";
               bug(tLaser,bState);
               break;

          
    }
      
}
catch(const Ice::Exception &exc)
{
    std::cout << exc << std::endl;
}
    
}

void SpecificWorker::gotoTarget(const TLaserData &tLaser)
{

    QVec target = innerModel->transform ( "base",pick.getAux(),"world" );

	float angulo = atan2( target.x(),target.z() );
	float distancia = target.norm2();

    //Comprobamos si hemos llegado al objetivo
    if ( distancia <= 100 )
	{
		pick.setActive ( false );
		qDebug() << "Hemos llegado al final.";
		state= State::IDLE;
		differentialrobot_proxy->stopBase();
		return;
	}

    //Comprobamos si el laser detecta algun tipo de obstaculo en las proximidades de nuestro robot
	if( obstacle ( tLaser ) )
	{
		state=State::TURN;
		qDebug() << "Obstaculo detectado, procedemos a girar";
		return;
	}

	if ( abs ( angulo ) > 0.05 )
		angulo = 0;
	if( distancia > 300) distancia = 300;

	try
	{
		differentialrobot_proxy->setSpeedBase(distancia, angulo);
	}
	catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }
}


void SpecificWorker::turne (const TLaserData& tLaser,const TBaseState& bState)
{
    
    QVec posi = QVec::vec3(bState.x, 0., bState.z);
	distanciaAnterior = fabs(linea.perpendicularDistanceToPoint(posi));
    
	if( obstacle(tLaser) == false)
	{
		state = State::BUG;
		qDebug() << "from TURN to BUG";
		return;
	}
	try
	{
		differentialrobot_proxy->setSpeedBase(0, -0.3);
	}
	catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; 
        
    }
}

    

void SpecificWorker::bug(const TLaserData &tLaser,const TBaseState &bState )
{
    
    
	//float distanciaObstaculo = obstacleRight(tLaser);
	float distanciaRestante = distanciaLinea(bState);
	
    //Comprueba si el objetivo esta a la vista
    if ( targetAtSight (tLaser) )
	{
		state = State::GOTO;
		qDebug() << "Target visible: from BUG to GOTO";
		return;
	}
	
	//Comprobamos si nos encontramos en las proximidades de la linea que une al objetivo
	if (distanciaAnterior < 100 and distanciaRestante < 0)
	{
		state = State::GOTO;
		qDebug() << "Crossing the line: from BUG to GOTO";
		return;
	}
	
	//Comprobamos si hay algun obstaculo en las proximidades
	if ( obstacle (tLaser) )
	{
		state = State::TURN;
		qDebug() << "from BUG to BUGINIT";
		return;
	}

	
	float vrot = setSignmoide(distanciaRestante);
	float vadv = setGauss(0.3,vrot,0.9)*setSignmoide(distanciaRestante)*350;		
    differentialrobot_proxy->setSpeedBase ( vadv ,vrot );

}


/**
 * Metodo para detectar obstaculos
 */

bool SpecificWorker::obstacle(TLaserData tLaser)
{
    const int minDist = 350;
    std::sort ( tLaser.begin() + 20, tLaser.end()- 20 , [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ){	return a.dist < b.dist;});
    return ( tLaser[30].dist < minDist );
}


/**
 * Metodo para detectar distancia obstaculo derecho
 */

float SpecificWorker::obstacleRight(const TLaserData& tlaser)
{
	const int laserpos = 95;
	float min = tlaser[laserpos].dist;
	for(int i=laserpos-2; i<laserpos+2;i++)
	{
		if (tlaser[i].dist < min)
			min = tlaser[i].dist;
	}
	return min;
}
    
/**
 * Metodo para saber si el objetivo se encuentra a la vista
 */

bool SpecificWorker::targetAtSight(TLaserData tLaser)
{
   QPolygon poligono;
	for ( auto l: tLaser )
	{
		QVec r = innerModel->laserTo ( "world","laser",l.dist,l.angle );
		QPoint p ( r.x(),r.z() );
		      poligono << p;
	}
	QVec targetInRobot = innerModel->transform("base", pick.getAux(), "world");
	float dist = targetInRobot.norm2();
	int veces = int(dist / 200);  //number of times the robot semilength fits in the robot-to-target distance
	float landa = 1./veces;
	
	QList<QPoint> points;
	points << QPoint(pick.getAux().x(),pick.getAux().z());  //Add target
	
	//Add points along lateral lines of robot
	for (float i=landa; i<= 1.; i+=landa)
	{
		QVec point = targetInRobot*(T)landa;
		QVec pointW = innerModel->transform("world", point ,"base");
		points << QPoint(pointW.x(), pointW.z());
		
		pointW = innerModel->transform("world", point - QVec::vec3(200,0,0), "base");
		points << QPoint(pointW.x(), pointW.z());
		
		pointW = innerModel->transform("world", point + QVec::vec3(200,0,0), "base");
		points << QPoint(pointW.x(), pointW.z());
		
	}
	foreach( QPoint p, points)
	{
		if( poligono.containsPoint(p , Qt::OddEvenFill) == false)
			return false;
	}
	return true;
}



/**Metodos auxiliares**/


float SpecificWorker::setGauss(float vADV, float vROT,float h)
{
    
    float lambda = -(pow(vADV, 2.0))/log(h);
    return exp(-pow(vROT, 2.0)/lambda);
    
}

float SpecificWorker::setSignmoide(float distancia)
{
    
    return 1/(1+exp(-distancia))-0.5;
    
}



float SpecificWorker::distanciaLinea(const TBaseState& bState)
{
	QVec posicion = QVec::vec3(bState.x, 0., bState.z);
	float distanciaActual = fabs(linea.perpendicularDistanceToPoint(posicion));
	float diferenciaDistancia = distanciaActual - distanciaAnterior;
	distanciaAnterior = distanciaActual;
	return diferenciaDistancia;
}


void SpecificWorker::stopRobot()
{
    
    differentialrobot_proxy->stopBase();

}
void SpecificWorker::setPick(const Pick &mypick)
{
    qDebug() << "New target selected: " << mypick.x << mypick.z;
    pick.copy ( mypick.x,mypick.z );
    pick.setActive ( true );
    state = State::IDLE;
}
  
void SpecificWorker::go(const string &nodo, const float x, const float y, const float alpha)
{
    pick.copy (x,y);
    pick.setActive (true);
    state = State::IDLE;
}

void SpecificWorker::turn(const float speed)
{
    differentialrobot_proxy->setSpeedBase(0,speed);
}

bool SpecificWorker::atTarget()
{
    return !pick.isActive();
}

void SpecificWorker::stop()
{
    differentialrobot_proxy->setSpeedBase(0,0);
}







