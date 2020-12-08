#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
	: m_vectMassPoints{ MAX_MASS_POINTS_INDEX },
	m_vectSpringObjects{ MAX_SPRING_OBJECTS_INDEX },
	m_iIntegrator{0}
{
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "DEMO1,DEMO2,DEMO3,DEMO4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "step=0.1");
	TwAddVarRW(DUC->g_pTweakBar, "SphereScale", TW_TYPE_FLOAT, &m_fSphereScale, "step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "");
	TwAddVarRW(DUC->g_pTweakBar, "StringStiffnes", TW_TYPE_FLOAT, &m_fStiffness, "");
	TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.01 step=0.01 max=1.0");
	
	TwAddVarRW(DUC->g_pTweakBar, "ExternalForce X", TW_TYPE_FLOAT, &m_externalForce.x, "");
	TwAddVarRW(DUC->g_pTweakBar, "ExternalForce Y", TW_TYPE_FLOAT, &m_externalForce.y, "");
	TwAddVarRW(DUC->g_pTweakBar, "ExternalForce Z", TW_TYPE_FLOAT, &m_externalForce.z, "");
	TwAddVarRW(DUC->g_pTweakBar, "ExplosionForceScalar", TW_TYPE_FLOAT, &m_ExplosionForceScalar, "");
	TwAddButton(DUC->g_pTweakBar, "EXPLODE", [](void* s) {static_cast<MassSpringSystemSimulator*>(s)->m_bExplode = true; }, this, "");
	
	switch (m_iTestCase)
	{
		//Demo1
	case 0:
		
		break;

	case 1:
		break;

	case 2:break;

	case 3:
	{
		TwType TW_TYPE_INTEGRATOR_TYPE = TwDefineEnumFromString("Integrator", "EULER,LEAPFROG,MIDPOINT");
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR_TYPE, &m_iIntegrator, "");
		break;
	}

	default:break;
	}
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0:  break;



	default:
		{
			Vec3 l_v3Green = Vec3(0.0f, 1.0f, 0.0f);
			Vec3 l_v3Red = Vec3(1.0f, 0.0f, 0.0f);

			//Draw all mass objects
			for (int l_iMassObjIndex = 0; l_iMassObjIndex <= m_iCurrentActiveLastMassPointIndex; l_iMassObjIndex++)
			{
				m_vectMassPoints[l_iMassObjIndex].drawMassObject(DUC, m_vectMassPoints[l_iMassObjIndex].m_bIsFixed ? l_v3Green : l_v3Red, Vec3(m_fSphereScale, m_fSphereScale, m_fSphereScale));
			}

			//Draw all springs
			for (int l_iSpringObjIndex = 0; l_iSpringObjIndex <= m_iCurrentActiveLastSpringIndex; l_iSpringObjIndex++)
			{
				m_vectSpringObjects[l_iSpringObjIndex].drawSpring(DUC, m_vectMassPoints);
			}

			//Draw explosion sphere
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(1.0f, 1.0f, 0.0f));
			DUC->drawSphere(m_v3ExplosionPoint, Vec3(m_fSphereScale, m_fSphereScale, m_fSphereScale));

			break;
		}
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	resetAllEntities();

	std::cout << "\n\nDEMO INSTRUCTIONS\n";
	std::cout << "1) Click the left mouse button to position the Yellow Sphere and set the Explosion force scalar. Press the EXPLODE button on the UI. \n";
	std::cout << "2) Set external or gravitational force\n";

	switch (m_iTestCase)
	{
	case 0:
		//DEMO1
		initializeDemo1();
		break;
	case 1:
		//DEMO2
		initializeDemo2();
		break;

	case 2:
		//DEMO3
		initializeDemo3();
		break;
	case 3:
		//DEMO4
		initializeDemo4();
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.000001f;
		inputWorld = inputWorld * inputScale;
		m_v3ExplosionPoint += inputWorld;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	for (int l_iMassObjectIndex = 0; l_iMassObjectIndex <= m_iCurrentActiveLastMassPointIndex; l_iMassObjectIndex++)
	{
		m_vectMassPoints[l_iMassObjectIndex].resetForce();
	}

	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases

		///DEMO 1
		case 0:
			break;

			///DEMO 2
		case 1:
		{
			calculateForce(m_vectSpringObjects[0]);
			calculateEuler(m_vectSpringObjects[0], 0.005f);

			MassPoint& l_refMassPointID1 = m_vectMassPoints[m_vectSpringObjects[0].m_iMassPointID1];
			MassPoint& l_refMassPointID2 = m_vectMassPoints[m_vectSpringObjects[0].m_iMassPointID2];

			std::cout << "EULER : \n\nP0: \nPosition: " << l_refMassPointID1.m_v3Position << "\nVelocity: " << l_refMassPointID1.m_v3Velocity << "\n\n\nP1:\nPosition: " << l_refMassPointID2.m_v3Position << "\nVelocity: " << l_refMassPointID2.m_v3Velocity << "\nSpringLength: " << m_vectSpringObjects[0].m_fCurrentLength << "\n------------\n\n";
		}
		break;

		///DEMO 3
		case 2:
		{
			calculateForce(m_vectSpringObjects[0]);
			calculateMidpoint(0.005f);

			MassPoint& l_refMassPointID1 = m_vectMassPoints[m_vectSpringObjects[0].m_iMassPointID1];
			MassPoint& l_refMassPointID2 = m_vectMassPoints[m_vectSpringObjects[0].m_iMassPointID2];

			std::cout << "MIDPOINT : \n\nP0: \nPosition: " << l_refMassPointID1.m_v3Position << "\nVelocity: " << l_refMassPointID1.m_v3Velocity << "\n\n\nP1:\nPosition: " << l_refMassPointID2.m_v3Position << "\nVelocity: " << l_refMassPointID2.m_v3Velocity << "\nSpringLength: " << m_vectSpringObjects[0].m_fCurrentLength << "\n------------\n\n";
			break;
		}
		///DEMO 4
		case 3:
		{
			for (int l_iSpringIndex = 0; l_iSpringIndex <= m_iCurrentActiveLastSpringIndex; l_iSpringIndex++)
			{
				calculateForce(m_vectSpringObjects[l_iSpringIndex]);
			}

			if (EULER == m_iIntegrator)
			{
				for (int l_iMassIndex = 0; l_iMassIndex <= m_iCurrentActiveLastMassPointIndex; l_iMassIndex++)
				{
					calculateEuler(m_vectMassPoints[l_iMassIndex], timeStep);
				}
			}
			else if (LEAPFROG == m_iIntegrator)
			{
				for (int l_iMassIndex = 0; l_iMassIndex <= m_iCurrentActiveLastMassPointIndex; l_iMassIndex++)
				{
					calculateLeapFrog(m_vectMassPoints[l_iMassIndex], timeStep);
				}
			}
			else if (MIDPOINT == m_iIntegrator)
			{
				calculateMidpoint(timeStep);
			}
			break;
		}
	}

	for (int l_iMassIndex = 0; l_iMassIndex <= m_iCurrentActiveLastMassPointIndex; l_iMassIndex++)
	{
		bool l_bOldValIsColliding = m_vectMassPoints[l_iMassIndex].m_bIsColliding;
		if (l_bOldValIsColliding != isSphereCollided(m_vectMassPoints[l_iMassIndex]))
		{
			if (!l_bOldValIsColliding)
			{
				m_vectMassPoints[l_iMassIndex].m_v3Velocity = -m_vectMassPoints[l_iMassIndex].m_v3Velocity;;
			}

			m_vectMassPoints[l_iMassIndex].m_bIsColliding = !l_bOldValIsColliding;
		}
	}


	m_bExplode = false;
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

#pragma region SPECIFIC FUNCTIONS

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_iCurrentActiveLastMassPointIndex + 1;
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_iCurrentActiveLastSpringIndex + 1;
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	if (index < 0 || index > m_iCurrentActiveLastMassPointIndex)
	{
		std::cout << "MassSpringSystemSimulator::getPositionOfMassPoint:: Index " << index << " out of bounds\n";
		return Vec3();
	}

	return m_vectMassPoints[index].m_v3Position;
}


Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	if (index < 0 || index > m_iCurrentActiveLastMassPointIndex)
	{
		std::cout << "MassSpringSystemSimulator::getPositionOfMassPoint:: Index " << index << " out of bounds\n";
		return Vec3();
	}

	return m_vectMassPoints[index].m_v3Velocity;
}


void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce += force;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 a_v3Position, Vec3 a_v3Velocity, bool a_bIsFixed)
{
	if (m_iCurrentActiveLastMassPointIndex >= (MAX_MASS_POINTS_INDEX - 1))
	{
		std::cout << "MassSpringSystemSimulator::addMassPoint:: Cannot add more mass points\n";
		return MAX_MASS_POINTS_INDEX;
	}

	++m_iCurrentActiveLastMassPointIndex;
	MassPoint& l_CurrentMassPoint =  m_vectMassPoints[m_iCurrentActiveLastMassPointIndex];
	l_CurrentMassPoint.m_iID = m_iCurrentActiveLastMassPointIndex;
	l_CurrentMassPoint.m_bIsFixed = a_bIsFixed;
	l_CurrentMassPoint.m_v3Position = a_v3Position;
	l_CurrentMassPoint.m_v3Velocity = a_v3Velocity;
	l_CurrentMassPoint.m_v3Force = { 0.0f, 0.0f, 0.0f };
	return m_iCurrentActiveLastMassPointIndex;
}

void MassSpringSystemSimulator::addSpring(int a_iMassPoint1, int a_iMassPoint2, float a_fInitialLength)
{
	if (m_iCurrentActiveLastSpringIndex >= (MAX_SPRING_OBJECTS_INDEX - 1))
	{
		std::cout << "MassSpringSystemSimulator::addSpring:: Cannot add more springs\n";
		return;
	}

	++m_iCurrentActiveLastSpringIndex;
	SpringObject& l_CurrentSpring = m_vectSpringObjects[m_iCurrentActiveLastSpringIndex];
	l_CurrentSpring.m_ID = m_iCurrentActiveLastSpringIndex;
	l_CurrentSpring.m_fRestLength = a_fInitialLength;
	l_CurrentSpring.m_iMassPointID1 = a_iMassPoint1;
	l_CurrentSpring.m_iMassPointID2 = a_iMassPoint2;
}

void MassSpringSystemSimulator::calculateForce(SpringObject& a_SpringObject, bool l_bIsTempForce)
{
	MassPoint& l_refMassPoint1 = m_vectMassPoints[a_SpringObject.m_iMassPointID1];
	MassPoint& l_refMassPoint2 = m_vectMassPoints[a_SpringObject.m_iMassPointID2];

	if (l_bIsTempForce)
	{
		float l_fLength = GetV3Length(l_refMassPoint1.m_v3PosTemp - l_refMassPoint2.m_v3PosTemp);
		Vec3 l_v3ForceP2ToP1 = -m_fStiffness * (l_fLength - a_SpringObject.m_fRestLength) * GetV3Direction(l_refMassPoint1.m_v3PosTemp - l_refMassPoint2.m_v3PosTemp);

		l_refMassPoint1.m_v3ForceTemp += l_v3ForceP2ToP1;
		l_refMassPoint2.m_v3ForceTemp += -l_v3ForceP2ToP1;
	}
	else
	{
		a_SpringObject.m_fCurrentLength = GetV3Length(l_refMassPoint1.m_v3Position - l_refMassPoint2.m_v3Position);
		Vec3 l_v3ForceP2ToP1 = -m_fStiffness * (a_SpringObject.m_fCurrentLength - a_SpringObject.m_fRestLength) * GetV3Direction(l_refMassPoint1.m_v3Position - l_refMassPoint2.m_v3Position);

		l_refMassPoint1.m_v3Force += l_v3ForceP2ToP1;
		l_refMassPoint2.m_v3Force += -l_v3ForceP2ToP1;
	}
}


void MassSpringSystemSimulator::calculateEuler(SpringObject& a_SpringObject, const float& a_fTimeStep)
{
	calculateEuler(m_vectMassPoints[a_SpringObject.m_iMassPointID1], a_fTimeStep);
	calculateEuler(m_vectMassPoints[a_SpringObject.m_iMassPointID2], a_fTimeStep);
}


void MassSpringSystemSimulator::calculateEuler(MassPoint& a_MassPoint, const float& a_fTimeStep)
{
	if (a_MassPoint.m_bIsFixed) { return; }

	Vec3 l_v3ExplosionForce =  getExplosionForceExertedOnMass(a_MassPoint);

	//Acceleration = Spring Internal + External - Damping + EXPLOSION + Gravity
	Vec3 l_v3Accelaration = (a_MassPoint.m_v3Force + m_externalForce + (-m_fDamping * a_MassPoint.m_v3Velocity) + l_v3ExplosionForce) / m_fMass;
	l_v3Accelaration += Vec3(0.0f, m_fGravity, 0.0f);

	//Calculate position at h
	a_MassPoint.m_v3Position = a_MassPoint.m_v3Position + (a_MassPoint.m_v3Velocity * a_fTimeStep);

	//Calculate velocity at h
	a_MassPoint.m_v3Velocity = a_MassPoint.m_v3Velocity + a_fTimeStep * l_v3Accelaration;
}

void MassSpringSystemSimulator::calculateMidpoint(float a_fTimeStep)
{
	for (int l_iMassIndex = 0; l_iMassIndex <= m_iCurrentActiveLastMassPointIndex; l_iMassIndex++)
	{
		calculateMidpointAtHalfstep(m_vectMassPoints[l_iMassIndex], a_fTimeStep);
	}

	// Calculate the temporary force i.e. at h/2
	for (int l_iSpringIndex = 0; l_iSpringIndex <= m_iCurrentActiveLastSpringIndex; l_iSpringIndex++)
	{
		calculateForce(m_vectSpringObjects[l_iSpringIndex], true);
	}

	for (int l_iMassIndex = 0; l_iMassIndex <= m_iCurrentActiveLastMassPointIndex; l_iMassIndex++)
	{
		calculateMidpointVelocityAtT(m_vectMassPoints[l_iMassIndex], a_fTimeStep);
	}
}

void MassSpringSystemSimulator::calculateMidpointAtHalfstep(MassPoint& a_MassPoint, const float& a_fTimeStep)
{
	if (a_MassPoint.m_bIsFixed) { return; }

	Vec3 l_v3ExplosionForce = getExplosionForceExertedOnMass(a_MassPoint);

	//Acceleration = Spring Internal + External - Damping + Gravity + Explosion
	Vec3 l_v3P1Accelaration = (a_MassPoint.m_v3Force + m_externalForce + (-m_fDamping * a_MassPoint.m_v3Velocity) + l_v3ExplosionForce) / m_fMass;
	l_v3P1Accelaration += Vec3(0.0f, m_fGravity, 0.0f);

	//calculate pos at h/2
	Vec3 l_v3Pos1_Tild = a_MassPoint.m_v3Position + (0.5f * a_fTimeStep * a_MassPoint.m_v3Velocity);

	//calculate velocity at h/2
	Vec3 l_v3Vel1_Tild = a_MassPoint.m_v3Velocity + (0.5f * a_fTimeStep * l_v3P1Accelaration);

	//calculate pos at h
	a_MassPoint.m_v3Position = a_MassPoint.m_v3Position + a_fTimeStep * l_v3Vel1_Tild;

	a_MassPoint.m_v3PosTemp = l_v3Pos1_Tild;
	a_MassPoint.m_v3VelTemp = l_v3Vel1_Tild;
}

void MassSpringSystemSimulator::calculateMidpointVelocityAtT(MassPoint& a_MassPoint, const float& a_fTimeStep)
{
	if (a_MassPoint.m_bIsFixed) { return; }

	Vec3 l_v3ExplosionForce = getExplosionForceExertedOnMass(a_MassPoint);

	//calculate acceleration at h/2
	//Acceleration = Spring Internal + External - Damping + EXPLOSION + Gravity
	Vec3 l_v3P1Accelaration = (a_MassPoint.m_v3ForceTemp + m_externalForce + (-m_fDamping * a_MassPoint.m_v3Velocity) + l_v3ExplosionForce) / m_fMass;
	l_v3P1Accelaration += Vec3(0.0f, m_fGravity, 0.0f);

	//calculate velocty at h
	a_MassPoint.m_v3Velocity = a_MassPoint.m_v3Velocity + a_fTimeStep * l_v3P1Accelaration;
}

void MassSpringSystemSimulator::calculateLeapFrog(MassPoint& a_MassPoint, float a_fTimeStep)
{
	if (a_MassPoint.m_bIsFixed) { return; }

	Vec3 l_v3ExplosionForce = getExplosionForceExertedOnMass(a_MassPoint);

	//Acceleration = Spring Internal + External - Damping + EXPLOSION + Gravity
	Vec3 l_v3Acceleration = ((a_MassPoint.m_v3Force) + (m_externalForce) + (-m_fDamping * a_MassPoint.m_v3Velocity) + l_v3ExplosionForce) / m_fMass;
	l_v3Acceleration += Vec3(0.0f, m_fGravity, 0.0f);

	a_MassPoint.m_v3Velocity = a_MassPoint.m_v3Velocity + a_fTimeStep * l_v3Acceleration;
	a_MassPoint.m_v3Position = a_MassPoint.m_v3Position + a_fTimeStep * a_MassPoint.m_v3Velocity;
}


#pragma endregion SPECIFIC FUNCTIONS


#pragma region DEMO FUNCTIONS

/// <summary>
/// Resets all spring, mass values
/// </summary>
void MassSpringSystemSimulator::resetAllEntities()
{
	//Reset stiffness and mass
	m_fStiffness = 40.0f;
	m_fMass = 10.0f;
	m_externalForce = Vec3(0.0f, 0.0f, 0.0f);
	m_fDamping = 0.0f;
	m_ExplosionForceScalar = 0.0f;
	m_v3ExplosionPoint = Vec3(0.0f, 0.0f, 0.0f);
	m_bExplode = false;

	//Reset total number of active springs and mass points
	m_iCurrentActiveLastMassPointIndex = -1;
	m_iCurrentActiveLastSpringIndex = -1;

	//Reset all spring points
	for (int l_iSpringIndex = 0; l_iSpringIndex < MAX_SPRING_OBJECTS_INDEX; l_iSpringIndex++)
	{
		m_vectSpringObjects[l_iSpringIndex].reset();
	}

	//Reset all  mass points
	for (int l_iMassPointIndex = 0; l_iMassPointIndex < MAX_MASS_POINTS_INDEX; l_iMassPointIndex++)
	{
		m_vectMassPoints[l_iMassPointIndex].reset();
	}
}


void MassSpringSystemSimulator::initializeDemo1()
{
	std::cout << "============ START DEMO1 ============\n";

	float l_fDemo1TimeStep = 0.1f;

	///Calculate Euler
	MassPoint& l_refMassPoint1_Euler = m_vectMassPoints[addMassPoint({0.0f,0.0f,0.0f}, { -1.0f,0.0f,0.0f }, false)];
	MassPoint& l_refMassPoint2_Euler = m_vectMassPoints[addMassPoint({ 0.0f,2.0f,0.0f }, { 1.0f,0.0f,0.0f }, false)];

	addSpring(l_refMassPoint1_Euler.m_iID, l_refMassPoint2_Euler.m_iID, 1.0f);
	SpringObject& l_CurrentSpringObject = m_vectSpringObjects[m_iCurrentActiveLastSpringIndex];

	calculateEuler(l_CurrentSpringObject, l_fDemo1TimeStep);
	std::cout << "EULER : \n\n At (h=0.1)	\nP0: \nPosition: "<< l_refMassPoint1_Euler.m_v3Position <<"\nVelocity: "<< l_refMassPoint1_Euler.m_v3Velocity<<"\n\n\nP1:\nPosition: " << l_refMassPoint2_Euler.m_v3Position << "\nVelocity: " << l_refMassPoint2_Euler.m_v3Velocity << "\n";

	std::cout << "\n\n\n";

	///Reset all values of all springs and mass points
	resetAllEntities();

	/// Calculate Midpoint
	MassPoint& l_refMassPoint1_Midpoint = m_vectMassPoints[addMassPoint({ 0.0f,0.0f,0.0f }, { -1.0f,0.0f,0.0f }, false)];

	MassPoint& l_refMassPoint2_Midpoint = m_vectMassPoints[addMassPoint({ 0.0f,2.0f,0.0f }, { 1.0f,0.0f,0.0f }, false)];

	addSpring(l_refMassPoint1_Midpoint.m_iID, l_refMassPoint2_Midpoint.m_iID, 1.0f);
	SpringObject& l_CurrentSpringObject_Midpoint = m_vectSpringObjects[m_iCurrentActiveLastSpringIndex];

	calculateMidpoint(l_fDemo1TimeStep);
	std::cout << "MIDPOINT : \n\n At (h=0.1)	\nP0: \nPosition: " << l_refMassPoint1_Midpoint.m_v3Position << "\nVelocity: " << l_refMassPoint1_Midpoint.m_v3Velocity << "\n\n\nP1:\nPosition: " << l_refMassPoint2_Midpoint.m_v3Position << "\nVelocity: " << l_refMassPoint2_Midpoint.m_v3Velocity << "\n\n\n\n";

	std::cout << "============ END DEMO1 ============\n";
}

void MassSpringSystemSimulator::initializeDemo2()
{
	//std::cout << "============ START DEMO2 ============\n";

	float l_fDemo2TimeStep = 0.005f;

	///Calculate Euler
	MassPoint& l_refMassPoint1_Euler = m_vectMassPoints[addMassPoint({ 0.0f,0.0f,0.0f }, { -1.0f,0.0f,0.0f }, false)];

	MassPoint& l_refMassPoint2_Euler = m_vectMassPoints[addMassPoint({ 0.0f,2.0f,0.0f }, { 1.0f,0.0f,0.0f }, false)];

	addSpring(l_refMassPoint1_Euler.m_iID, l_refMassPoint2_Euler.m_iID, 1.0f);

	//std::cout << "============ END DEMO2 ============\n";
}

void MassSpringSystemSimulator::initializeDemo3()
{
	//std::cout << "============ START DEMO3 ============\n";
	float l_fDemo3TimeStep = 0.005f;

	///Calculate Euler
	MassPoint& l_refMassPoint1_Euler = m_vectMassPoints[addMassPoint({ 0.0f,0.0f,0.0f }, { -1.0f,0.0f,0.0f }, false)];

	MassPoint& l_refMassPoint2_Euler = m_vectMassPoints[addMassPoint({ 0.0f,2.0f,0.0f }, { 1.0f,0.0f,0.0f }, false)];

	addSpring(l_refMassPoint1_Euler.m_iID, l_refMassPoint2_Euler.m_iID, 1.0f);
	
	//std::cout << "============ END DEMO3 ============\n";
}

void MassSpringSystemSimulator::initializeDemo4()
{
	//Setup mass points
	MassPoint& l_refMassPoint1 = m_vectMassPoints[addMassPoint({ -0.5f,-0.5f,-0.5f }, { 0.0f,0.0f,0.0f }, true)];

	MassPoint& l_refMassPoint2 = m_vectMassPoints[addMassPoint({ -0.5f,-0.5f,0.5f }, { 0.0f,0.0f,0.0f }, false)];

	MassPoint& l_refMassPoint3 = m_vectMassPoints[addMassPoint({ 0.5f, -0.5f, 0.5f }, { 0.0f,0.0f,0.0f }, true)];

	MassPoint& l_refMassPoint4 = m_vectMassPoints[addMassPoint({ 0.5f, -0.5f,-0.5f }, { 0.0f,0.0f,0.0f }, false)];

	MassPoint& l_refMassPoint5 = m_vectMassPoints[addMassPoint({ -0.5f,0.5f,-0.5f }, { 0.0f,0.0f,0.0f }, false)];

	MassPoint& l_refMassPoint6 = m_vectMassPoints[addMassPoint({ -0.5f,0.5f,0.5f } , { 0.0f,0.0f,0.0f }, true)];

	MassPoint& l_refMassPoint7 = m_vectMassPoints[addMassPoint({ 0.5f, 0.5f, 0.5f }, { 0.0f,0.0f,0.0f }, false)];
	
	MassPoint& l_refMassPoint8 = m_vectMassPoints[addMassPoint({ 0.5f, 0.5f, -0.5f }, { 0.0f,0.0f,0.0f }, true)];

	MassPoint& l_refMassPoint9 = m_vectMassPoints[addMassPoint({ 0.0f, 0.5f, 0.0f }, { 0.0f,0.0f,0.0f }, false)];

	MassPoint& l_refMassPoint10 = m_vectMassPoints[addMassPoint({ 0.0f, -0.5f, -0.0f }, { 0.0f,0.0f,0.0f }, false)];

	//Setup spring objects
	addSpring(l_refMassPoint1.m_iID, l_refMassPoint2.m_iID, 1.2f);
	addSpring(l_refMassPoint2.m_iID, l_refMassPoint3.m_iID, 1.2f);
	addSpring(l_refMassPoint3.m_iID, l_refMassPoint4.m_iID, 1.2f);
	addSpring(l_refMassPoint4.m_iID, l_refMassPoint1.m_iID, 1.2f);
															  
	addSpring(l_refMassPoint5.m_iID, l_refMassPoint6.m_iID, 1.2f);
	addSpring(l_refMassPoint6.m_iID, l_refMassPoint7.m_iID, 1.2f);
	addSpring(l_refMassPoint7.m_iID, l_refMassPoint8.m_iID, 1.2f);
	addSpring(l_refMassPoint8.m_iID, l_refMassPoint5.m_iID, 1.2f);
															  
	addSpring(l_refMassPoint1.m_iID, l_refMassPoint5.m_iID, 1.2f);
	addSpring(l_refMassPoint2.m_iID, l_refMassPoint6.m_iID, 1.2f);
	addSpring(l_refMassPoint3.m_iID, l_refMassPoint7.m_iID, 1.2f);
	addSpring(l_refMassPoint4.m_iID, l_refMassPoint8.m_iID, 1.2f);
															  
	addSpring(l_refMassPoint2.m_iID, l_refMassPoint9.m_iID, 1.2f);
	addSpring(l_refMassPoint9.m_iID, l_refMassPoint4.m_iID, 1.2f);

	addSpring(l_refMassPoint7.m_iID, l_refMassPoint10.m_iID, 1.2f);
	addSpring(l_refMassPoint10.m_iID, l_refMassPoint5.m_iID, 1.2f);
}

float MassSpringSystemSimulator::GetV3Length(const Vec3& a_Vec3)
{
	float l_fResult =  (a_Vec3.x * a_Vec3.x) + (a_Vec3.y * a_Vec3.y) + (a_Vec3.z * a_Vec3.z);
	return sqrt(l_fResult);
}

Vec3 MassSpringSystemSimulator::GetV3Direction(const Vec3& a_Vec3)
{
	return a_Vec3 / GetV3Length(a_Vec3);
}

bool MassSpringSystemSimulator::isSphereCollided(const MassPoint& a_MassPoint)
{
	if (a_MassPoint.m_bIsFixed) { return false; }

	for (int l_iMassPointIndex = 0; l_iMassPointIndex <= m_iCurrentActiveLastMassPointIndex; l_iMassPointIndex++)
	{
		if (a_MassPoint.m_iID == l_iMassPointIndex)
		{
			continue;
		}

		if (GetV3Length(m_vectMassPoints[l_iMassPointIndex].m_v3Position - a_MassPoint.m_v3Position) < m_fSphereScale)
		{
			return true;
		}
		else if ((a_MassPoint.m_v3Position.y - m_fSphereScale) < m_fGroundY)
		{
			return true;
		}
	}
	return false;
}

Vec3 MassSpringSystemSimulator::getExplosionForceExertedOnMass(const MassPoint& a_MassPoint)
{
	if (!m_bExplode) { return Vec3(0.0f, 0.0f, 0.0f); }

	return m_ExplosionForceScalar * GetV3Direction(a_MassPoint.m_v3Position - m_v3ExplosionPoint) * 1000.0f;

}

#pragma endregion DEMO FUNCTIONS