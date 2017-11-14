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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	float giro;
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
    float setGauss(float Vx,float Vr,float h);
    float setSignmoide(float distancia);
	void setPick(const Pick &mypick); 
    void stopRobot();
    void gotoTarget(const TLaserData &tLaser);
    void turne (const TLaserData& tLaser,const TBaseState& bState);
    void bug(const TLaserData &tLaser,const TBaseState &bState );
    bool obstacle(TLaserData tLaser);
	float obstacleRight( const TLaserData& tlaser);    
    bool targetAtSight(TLaserData tLaser);
    float distanciaLinea(const TBaseState& bState);
    void stop();
    bool atTarget();
    void turn(const float speed);
    void go(const string &nodo, const float x, const float y, const float alpha);

public slots:
    
	void compute(); 	

private:
    
    enum class State {IDLE,GOTO,BUG,TURN};
    
    struct Target
		{
			mutable QMutex m;
			QVec pose = QVec::zeros(3);
			bool active = false;
            
			void setActive(bool newActive)
			{
				QMutexLocker lm(&m);
				active = newActive;
			}
			
			void copy(float x, float z)
			{
				QMutexLocker lm(&m);
				pose.setItem(0,x);
				pose.setItem(1,0);
				pose.setItem(2,z);
			}
			
			QVec getAux()
			{
				QMutexLocker lm(&m);
				return pose;
			}
			
			bool isActive()
            {
            
                QMutexLocker lm(&m);
				return active;
            }
            
		};
        
		InnerModel* innerModel;
		Target pick;
        QLine2D linea;
        State state = State::IDLE;
        float distanciaAnterior;
	
};

#endif

