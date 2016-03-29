// Spark.cpp: implementation of the ASpark class.
//
//////////////////////////////////////////////////////////////////////

#include "aSpark.h"
#include <math.h>

#ifndef GRAVITY
#define GRAVITY 9.8f
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ASpark::ASpark()
{
	//coefficients of restitution equals 0.25
	m_COR = 0.25f;
	m_mass = 1.0;
}

ASpark::ASpark(float* color): AParticle()
{
	for (int i = 0; i < 3; i++)
		m_color[i] = color[i];
 
	//coefficients of restitution equals 0.25
	m_COR = 0.25f;
}

ASpark::~ASpark()
{

}

//Set attractor position
void ASpark::setAttractor(vec3 position)
{
	m_attractorPos = position;
}

//Set repeller position
void ASpark::setRepeller(vec3 position)
{
	m_repellerPos = position;
}

void ASpark::setWind(vec3 wind)
{
	m_windForce = wind;
}

void ASpark::display()
{
	float fadeTime = 3.0;
	if (m_alive)
	{
		float alpha = 1.0;
		if (m_state[10] < fadeTime)
		{
			alpha = m_state[10] / 10.0f;
		}
		float scale = 1.0;

		glPushMatrix();
		glColor4f(m_color[0], m_color[1], m_color[2], alpha);
		glTranslatef(m_state[0], m_state[1], m_state[2]);
		glScalef(scale, scale, scale);
		glutSolidSphere(1.0, 10, 10);
		glPopMatrix();
	}

}
	


void ASpark::update(float deltaT, int extForceMode)
{
	m_deltaT = deltaT;
	if (m_state[10] <= 0.0)
	{
		m_alive = false;
		return;
	}

	if (!(extForceMode & EXT_SPARKFORCES_ACTIVE))
		extForceMode = 0;
	
	computeForces(extForceMode);
	
	updateState(deltaT, EULER);

	resolveCollisions();
	
	
}


 
void ASpark::computeForces(int extForceMode)
//	computes the forces applied to this spark
{
	// zero out all forces
	m_state[6] = 0.0;
	m_state[7] = 0.0;
	m_state[8] = 0.0;

	// gravity force
	addForce(m_mass*m_gravity);


	// wind force
	if (extForceMode & WIND_ACTIVE)
	{
		addForce(m_windForce);
	}

	if (extForceMode & DRAG_ACTIVE)
	{
		vec3 drag(0.f);
		float c = 0.001;
		drag[0] = -m_state[3] * m_state[3] * c;
		drag[1] = -m_state[4] * m_state[4] * c;
		drag[2] = -m_state[5] * m_state[5] * c;
		addForce(drag);
	}


	// attractor force
	if (extForceMode & ATTRACTOR_ACTIVE)
	{
		vec3 att(0.f);
		att[0] = 0.1f * (m_attractorPos[0] - m_state[0]);
		att[1] = 0.1f * (m_attractorPos[1] - m_state[1]);
		addForce(att);
	}

	// repeller force
	if (extForceMode & REPELLER_ACTIVE)
	{
		vec3 rep(0.f);
		rep[0] = 1.f / ((m_repellerPos[0] - m_state[0]));
		rep[1] = 1.f / ((m_repellerPos[1] - m_state[1]));
		addForce(rep);
	}

	// random force
	if (extForceMode & RANDOM_ACTIVE)
	{
		vec3 random(0.f);
		random[0] = 10.f * ((float)rand()) / (float)RAND_MAX - 0.5f;
		random[1] = 10.f * ((float)rand()) / (float)RAND_MAX - 0.5f;
		addForce(random);
	}

}

void ASpark::resolveCollisions()
// resolves collisions of the spark with the ground
{
	if (m_state[1] <= 0.f) {
		m_state[4] *= -1.f * m_COR;
	}
}


















