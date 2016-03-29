#include "aBehaviorController.h"
#include "aBehaviors.h"
#include "aVector.h"
#include "aRotation.h"
#include <Windows.h>
#include <algorithm>

#define Truncate(a, b, c) (a = std::max<double>(std::min<double>(a,c),b))

double BehaviorController::gMaxSpeed = 1000.0; 
double BehaviorController::gMaxAngularSpeed = 200.0;  
double BehaviorController::gMaxForce = 2000.0;  
double BehaviorController::gMaxTorque = 2000.0;
double BehaviorController::gKNeighborhood = 500.0;   
double BehaviorController::gOriKv = 1.0;    
double BehaviorController::gOriKp = 1.0;  
double BehaviorController::gVelKv = 1.0;    
double BehaviorController::gAgentRadius = 80.0;  
double BehaviorController::gMass = 1;
double BehaviorController::gInertia = 1;
double BehaviorController::KArrival = 1.0; 
double BehaviorController::KDeparture = 12000.0;
double BehaviorController::KNoise = 15.0;
double BehaviorController::KWander = 80.0;   
double BehaviorController::KAvoid = 600.0;  
double BehaviorController::TAvoid = 1000.0;   
double BehaviorController::KSeparation = 12000.0; 
double BehaviorController::KAlignment = 1.0;  
double BehaviorController::KCohesion = 1.0;  

const double M2_PI = M_PI * 2.0;

BehaviorController::BehaviorController() 
{
	m_state.resize(m_stateDim);
	m_stateDot.resize(m_stateDim);
	m_controlInput.resize(m_controlDim);

	vec3 m_Pos0 = vec3(0, 0, 0);
	vec3 m_Vel0 = vec3(0, 0, 0);
	vec3 m_lastVel0 = vec3(0, 0, 0);
	vec3 m_Euler = vec3(0, 0, 0);
	vec3 m_VelB = vec3(0, 0, 0);
	vec3 m_AVelB = vec3(0, 0, 0);
	
	m_Vdesired = vec3(0, 0, 0);
	m_lastThetad = 0.0;

	m_Active = true; 
	mpActiveBehavior = NULL;
	mLeader = false;
	
	reset();
}

void BehaviorController::createBehaviors(std::vector<BehaviorController>& agentList, std::vector<Obstacle>& obstacleList)
{
	m_AgentList = &agentList;
	mObstacleList = &obstacleList;

	m_BehaviorList.clear();
	m_BehaviorList[SEEK] = new Seek(m_Target);
	m_BehaviorList[FLEE] = new Flee(m_Target);
	m_BehaviorList[ARRIVAL] = new Arrival(m_Target);
	m_BehaviorList[DEPARTURE] = new Departure(m_Target);
	m_BehaviorList[WANDER] = new Wander();
	m_BehaviorList[COHESION] = new Cohesion(agentList);
	m_BehaviorList[ALIGNMENT] = new Alignment(m_Target, agentList);
	m_BehaviorList[SEPARATION] = new Separation(m_Target, agentList);
	m_BehaviorList[LEADER] = new Leader(m_Target, agentList);
	m_BehaviorList[FLOCKING] = new Flocking(m_Target, agentList);
	m_BehaviorList[AVOID] = new Avoid(m_Target, obstacleList);
}

BehaviorController::~BehaviorController()
{
	mpActiveBehavior = NULL;
}

void BehaviorController::reset()
{
	vec3 startPos;
	startPos[0] = ((double)rand()) / RAND_MAX;
	startPos[1] =  ((double)rand()) / RAND_MAX, 
	startPos[2] = ((double)rand()) / RAND_MAX;
	startPos = startPos - vec3(0.5, 0.5, 0.5);

	startPos[1] = 0; // set equal to zero for 2D case (assume y is up)

	m_Guide.setLocalTranslation(startPos * 500.0);
	
	for (int i = 0; i < m_stateDim; i++)
	{
		m_state[i] = 0.0;
		m_stateDot[i] = 0.0;
	}

	m_force = 0.0;
	m_torque = 0.0;
	m_thetad = 0.0;
	m_vd = 0.0;
}

///////////////////////////////////////////////////

inline void ClampAngle(double& angle)
{
	while (angle > M_PI)
	{
		angle -= M2_PI;
	}
	while (angle < -M_PI)
	{
		angle += M2_PI;
	}
}


void BehaviorController::sense(double deltaT)
{
	if (mpActiveBehavior)
	{
		// find the agents in the neighborhood of the current character.
	}
	
}

void BehaviorController::control(double deltaT)
// Given the active behavior this function calculates a desired velocity vector (Vdesired).  
// The desired velocity vector is then used to compute the desired speed (vd) and direction (thetad) commands

{
	if (mpActiveBehavior)
	{ 
		m_Vdesired = mpActiveBehavior->calcDesiredVel(*this);
		m_Vdesired[_Y] = 0;

		m_force = 0.0;
		m_torque = 0.0;

		//m_thetad = m_state[1][_Y] + (Dot(m_Vdesired.Normalize(), m_Vel0.Normalize()));
		m_thetad = atan2(m_Vdesired[2], m_Vdesired[0]);
		if (m_Vdesired[2] < 0.f && m_state[2][_Z] > 0.f) m_thetad -= M_PI * 2.f;
		if (m_Vdesired[2] > 0.f && m_state[2][_Z] < 0.f) m_thetad += M_PI * 2.f;
		m_vd = m_Vdesired.Length();
		//m_thetad = atan2(m_Vdesired[2], m_Vdesired[0]) + m_lastThetad;
		//std::cout <<"\n" << m_Vdesired << "  "<< m_thetad;

		//if (fabs(m_Vdesired[2]) < 1.f) {
		//	if (m_Vdesired[0] < 0.f) {
		//		m_thetad = -M_PI_2;
		//	}
		//	else {
		//		m_thetad = M_PI_2;
		//	}
		//	
		//}
		//std::cout << "  " << m_thetad;

		



		//ClampAngle(m_thetad);

		float tSettle_f = 0.4;
		//float Kv_f = 1.f / (tSettle_f / 4.f);
		float Kv_f = 10.f;
		m_force = gMass * Kv_f * (m_Vdesired - m_state[2]);
		//m_force = gMass * (m_Vdesired - m_state[2]);

		float tSettle_t = 0.25;
		//float Kv_t = 1.f / (tSettle_t / 4.f);
		float Kv_t = 35.f;
		float Kp_t = 300.f;
		m_torque = gInertia * (-Kv_t * m_state[3] + Kp_t * (m_thetad - m_state[1][1]));
		//m_torque = gInertia * (-m_state[3] + (m_thetaD - m_state[1]));


		// Check if I need to truncate (max variables exist but no mention in instructions)
		Truncate(m_force[0], -gMaxForce, gMaxForce);
		Truncate(m_force[2], -gMaxForce, gMaxForce);
		Truncate(m_torque[1], -gMaxTorque, gMaxTorque);
		
		// when agent desired agent velocity and actual velocity < 2.0 then stop moving
		if (m_vd < 2.0 &&  m_state[VEL][_Z] < 2.0)
		{
			m_force = 0.0;
			m_torque = 0.0;
		}
	}
	else
	{
		m_force = 0.0;
		m_torque = 0.0;
	}

	// set control inputs to current force and torque values
	m_controlInput[0] = m_force;
	m_controlInput[1] = m_torque;
}

void BehaviorController::act(double deltaT)
{
	computeDynamics(m_state, m_controlInput, m_stateDot, deltaT);
	
	int EULER = 0;
	int RK2 = 1;
	updateState(deltaT, EULER);
}


void BehaviorController::computeDynamics(vector<vec3>& state, vector<vec3>& controlInput, vector<vec3>& stateDot, double deltaT)

{
	for (unsigned int i = 0; i < m_stateDim; i++)
		stateDot[i] = 0.0;

	vec3& force = controlInput[0];
	vec3& torque = controlInput[1];

	// TODO Compute the stateDot vector given the values of the current state vector and control input vector

	stateDot[0] = state[2];
	stateDot[1] = state[3];
	stateDot[2] = controlInput[0] / gMass;
	stateDot[3] = controlInput[1] / gInertia - state[3];

}

void BehaviorController::updateState(float deltaT, int integratorType)
{
//  Update the state vector given the m_stateDot vector using Euler (integratorType = 0) or RK2 (integratorType = 1) integratio
//  this should be similar to what you implemented in the particle system assignment.

	// TODO: add your code here

	m_state[0] = m_state[0] + deltaT * m_stateDot[0];
	m_state[1] = m_state[1] + deltaT * m_stateDot[1];
	m_state[2] = m_state[2] + deltaT * m_stateDot[2];
	m_state[3] = m_state[3] + deltaT * m_stateDot[3];


	Truncate(m_state[2][0], -gMaxSpeed, gMaxSpeed); // Vx
	Truncate(m_state[2][2], -gMaxSpeed, gMaxSpeed); // Vz
	Truncate(m_state[3][1], -gMaxAngularSpeed, gMaxAngularSpeed); // w

//  given the new values in m_state, these are the new component state values 
	m_Pos0  = m_state[POS];  // new world position
	m_Vel0 = m_stateDot[POS]; // new world velocity
	m_Euler = m_state[ORI];  // new orientation
	m_VelB  = m_state[VEL];  // new body axes velocity
	m_AVelB = m_state[AVEL]; // new angular velocity



	// update the guide orientation
	// compute direction from nonzero velocity vector
	vec3 dir;
	if (m_Vel0.Length() < 1.0)
	{
		dir = m_lastVel0;
		dir.Normalize();;
		m_state[ORI] = atan2(dir[_Z], dir[_X]);
	}
	else
	{
		dir = m_Vel0;
		m_lastVel0 = m_Vel0;
	}

	dir.Normalize();
	vec3 up(0.0, 1.0, 0.0);
	vec3 right = up.Cross(dir);
	right.Normalize();
	mat3 rot(right, up, dir);
	m_Guide.setLocalRotation(rot.Transpose());
	m_Guide.setLocalTranslation(m_Guide.getLocalTranslation() + m_Vel0*deltaT);

}


void BehaviorController::setTarget(AJoint& target)
{
	m_Target = target;
}

void BehaviorController::setBehavior(Behavior* pBehavior)
{
	mpActiveBehavior = pBehavior;
}

void BehaviorController::setActiveBehavior(BehaviorType type)
{
	m_BehaviorType = type;
	Behavior* pActiveBehavior = m_BehaviorList[type];
	setBehavior(pActiveBehavior);

}

void BehaviorController::display()
{ // helps with debugging behaviors.  red line is actual velocity vector, green line is desired velocity vector
	
	vec3 pos = getPosition();
	double scale = 1.0;
	vec3 vel = scale* getVelocity();
	double velMag = vel.Length();
	vec3 dvel = scale* getDesiredVelocity();
	vec3 angle = getOrientation() * (180.0 / 3.14159);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(pos[0], pos[1], pos[2]);
	glVertex3f(pos[0] + vel[0], pos[1] + vel[1], pos[2] + vel[2]);
	glColor3f(0, 1, 0);
	glVertex3f(pos[0], pos[1], pos[2]);
	glVertex3f(pos[0] + dvel[0], pos[1] + dvel[1], pos[2] + dvel[2]);
	glEnd();

	if (this->isLeader())
		glColor3f(0, 0, 1);
	else glColor3f(0.5, 0, 0);

	glPushMatrix();
	glTranslatef(pos[0], pos[1], pos[2]);
	glRotatef(90 - angle[1], 0, 1, 0);
	glutSolidCone(40, 80, 10, 10);
	glutSolidSphere(35, 10, 10);
	glPopMatrix();

	BehaviorType active = getActiveBehavior();
	Behavior* pBehavior = m_BehaviorList[active];
	pBehavior->display(*this);

}

