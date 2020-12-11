#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
	: m_vectRigidBodies(MAX_RIGIDBODIES)
{
	m_iTestCase = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr() {
	return "DEMO1,DEMO2,DEMO3,DEMO4";
}

void RigidBodySystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_bIsFirstRunOnNewCase = true;
	for (int l_iRBIndex = 0; l_iRBIndex < MAX_RIGIDBODIES; l_iRBIndex++)
	{
		m_vectRigidBodies[l_iRBIndex].reset();
	}

	m_iTestCase = testCase;

	//Reset active rigid bodies to 0 to disable all rigid bodies
	m_iActiveRigidBodies = 0;

	std::cout << "\n\n\n\n\n===========================================================================================================\n";
	switch (m_iTestCase)
	{
	case 0:
	{
		//DEMO 1
		std::cout << "START DEMO1\n";
		addRigidBody({ 0.0f, 0.0f, 0.0f }, { 1.0f, 0.6f, 0.5f }, 2.0f);
		setOrientationOf(0, Quat(0.0f, 0.0f, (90.0f / 180.0f) * M_PI));
		applyForceOnBody(0, { 0.3f, 0.5f, 0.25f }, { 1.0f,1.0f,0.0f });
		break;
	}

	case 1:
	{
		//DEMO 2
		std::cout << "START DEMO2\n";
		addRigidBody({ 0.0f, 0.0f, 0.0f }, { 1.0f, 0.6f, 0.5f }, 2.0f);
		setOrientationOf(0, Quat(0.0f, 0.0f, (90.0f / 180.0f) * M_PI));
		applyForceOnBody(0, { 0.3f, 0.5f, 0.25f }, { 1.0f,1.0f,0.0f });
		applyForceOnBody(0, { 0.1f, -0.2f, -0.25f }, { -1.3f,-0.4f,0.0f });
		//applyForceOnBody(0, { -0.3f, -0.5f, -0.25f }, { 1.0f,1.0f,0.0f });
		break;
	}

	case 2:
	{
		//DEMO 3
		std::cout << "START DEMO3\n";
		break;
	}

	case 3:
	{
		//DEMO 4
		std::cout << "START DEMO4\n";
		break;
	}

	default:
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	//// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	//Point2D mouseDiff;
	//mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	//mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	//if (mouseDiff.x != 0 || mouseDiff.y != 0)
	//{
	//	Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
	//	worldViewInv = worldViewInv.inverse();
	//	Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
	//	Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
	//	// find a proper scale!
	//	float inputScale = 0.001f;
	//	inputWorld = inputWorld * inputScale;
	//	m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	//}
	//else {
	//	m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	//}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0:
		{
			if (m_bIsFirstRunOnNewCase)
			{
				// simulate movement of all active rigid bodies
				simulateRigidBodies(2.0f);
				Vec3 l_v3WorldPosOfPoint = m_vectRigidBodies[0].m_v3CenterPosition + (m_vectRigidBodies[0].m_quatRotation.getRotMat() * Vec3 { 0.3f, 0.5f, 0.25f });
				Vec3 l_v3WorldVelocityOfPoint = m_vectRigidBodies[0].m_v3LinearVelocity + cross(m_vectRigidBodies[0].m_v3AngularVelocity, { 0.3f, 0.5f, 0.25f });
				std::cout << "After single timestep h = 2.0f of point (0.3, 0.5, 0.25) \nWorld space position : " << l_v3WorldPosOfPoint << "\nWorld space velocity : " << l_v3WorldVelocityOfPoint << "\n";
			}
			break;
		}
		default:
		{
			// simulate movement of all active rigid bodies
			simulateRigidBodies(timeStep);
		}
	}

	//Reset external forces
	m_externalForce = { 0.0f, 0.0f, 0.0f };
	m_bIsFirstRunOnNewCase = false;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	// draw all rigid bodies
	drawRigidBodies();
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

#pragma region User defined
RigidBodySystemSimulator::RBCube* RigidBodySystemSimulator::getRigidBody(unsigned int a_iIndex)
{
	if (a_iIndex >= 0 && a_iIndex < m_iActiveRigidBodies)
	{
		return &m_vectRigidBodies[a_iIndex];
	}
	else
	{
		std::cout << "Rigid body at intex " << a_iIndex << " does not exist. Min Index: " << 0 << " Max Index : " << m_iActiveRigidBodies - 1 << " Count : " << m_vectRigidBodies.size() << "\n";
		return nullptr;
	}
}


void RigidBodySystemSimulator::simulateRigidBodies(float a_fTimeStep)
{
	int l_iRigidBodyCount = getNumberOfRigidBodies();
	for (int l_iRBIndex = 0; l_iRBIndex < l_iRigidBodyCount; l_iRBIndex++)
	{
		RBCube& l_RB = m_vectRigidBodies[l_iRBIndex];

		//New position with linear velocity from last frame
		l_RB.m_v3CenterPosition = l_RB.m_v3CenterPosition + a_fTimeStep * l_RB.m_v3LinearVelocity;

		//New Linear velocity to be used next frame with the external force from this frame
		l_RB.m_v3LinearVelocity = l_RB.m_v3LinearVelocity + a_fTimeStep * (m_externalForce / l_RB.m_iMass);

		//Set new rotation based on angular velocity calculated from last frame
		l_RB.m_quatRotation = l_RB.m_quatRotation + 0.5f * a_fTimeStep * Quat(l_RB.m_v3AngularVelocity.x, l_RB.m_v3AngularVelocity.y, l_RB.m_v3AngularVelocity.z, 0.0f) * l_RB.m_quatRotation;

		//New angular momentum
		l_RB.m_v3AngularMomentum = l_RB.m_v3AngularMomentum + a_fTimeStep * l_RB.m_v3Torque;

		// New inverse inertia tensor
		Mat4 l_m4TransposeRot = l_RB.m_quatRotation.getRotMat();
		l_m4TransposeRot.transpose();
		Mat4 l_CurrentInvInertiaTensor = l_RB.m_quatRotation.getRotMat() * l_RB.m_m4InvInertiaTensor * l_m4TransposeRot;

		// New angular velocity
		l_RB.m_v3AngularVelocity = l_CurrentInvInertiaTensor * l_RB.m_v3AngularMomentum;

		//re normalizing quat
		l_RB.m_quatRotation = l_RB.m_quatRotation.unit();

		//Set torque to zero for next frame
		l_RB.m_v3Torque = { 0.0f, 0.0f, 0.0f };
	}
}

void RigidBodySystemSimulator::drawRigidBodies()
{
	int l_iRigidBodyCount = getNumberOfRigidBodies();
	for (int l_iRBIndex = 0; l_iRBIndex < l_iRigidBodyCount; l_iRBIndex++)
	{
		RBCube& l_RB = m_vectRigidBodies[l_iRBIndex];

		DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		DUC->drawRigidBody(l_RB.getTransformation());
	}
}

#pragma endregion User defined

#pragma region Extra functions

// ExtraFunctions
int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return m_iActiveRigidBodies;
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	RBCube* l_pRB = getRigidBody(i);
	return (l_pRB != nullptr) ? l_pRB->m_v3CenterPosition : Vec3(0.0f, 0.0f, 0.0f);
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	RBCube* l_pRB = getRigidBody(i);
	return (l_pRB != nullptr) ? l_pRB->m_v3LinearVelocity : Vec3(0.0f, 0.0f, 0.0f);
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	RBCube* l_pRB = getRigidBody(i);
	return (l_pRB != nullptr) ? l_pRB->m_v3AngularVelocity : Vec3(0.0f, 0.0f, 0.0f);
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	RBCube* l_pRB = getRigidBody(i);

	if (l_pRB != nullptr)
	{
		l_pRB->m_v3Torque += cross(loc, force);
	}
	m_externalForce += force;
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	if (m_iActiveRigidBodies < MAX_RIGIDBODIES)
	{
		++m_iActiveRigidBodies;
		RBCube* l_pRB = getRigidBody(m_iActiveRigidBodies - 1);
		l_pRB->reset();
		l_pRB->m_v3CenterPosition = position;
		l_pRB->m_v3Dimensions = size;
		l_pRB->m_iMass = mass;

		//Calculate initial inverse inertia tensor
		Mat4 l_m4Covariance;
		l_m4Covariance.initId();
		float l_f1By12 = (1.0f / 12.0f);
		l_m4Covariance.value[0][0] = l_f1By12 * l_pRB->m_iMass * ((l_pRB->m_v3Dimensions.y * l_pRB->m_v3Dimensions.y) + (l_pRB->m_v3Dimensions.z * l_pRB->m_v3Dimensions.z));
		l_m4Covariance.value[1][1] = l_f1By12 * l_pRB->m_iMass * ((l_pRB->m_v3Dimensions.x * l_pRB->m_v3Dimensions.x) + (l_pRB->m_v3Dimensions.z * l_pRB->m_v3Dimensions.z));
		l_m4Covariance.value[2][2] = l_f1By12 * l_pRB->m_iMass * ((l_pRB->m_v3Dimensions.x * l_pRB->m_v3Dimensions.x) + (l_pRB->m_v3Dimensions.y * l_pRB->m_v3Dimensions.y));

		float l_fCovarianceTrace = l_m4Covariance.value[0][0] + l_m4Covariance.value[1][1] + l_m4Covariance.value[2][2];

		l_pRB->m_m4InvInertiaTensor.initId();
		l_pRB->m_m4InvInertiaTensor = l_pRB->m_m4InvInertiaTensor * l_fCovarianceTrace - l_m4Covariance;
		l_pRB->m_m4InvInertiaTensor.inverse();
	}
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	RBCube* l_pRB = getRigidBody(i);

	if (l_pRB != nullptr)
	{
		l_pRB->m_quatRotation = orientation;
	}
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	RBCube* l_pRB = getRigidBody(i);

	if (l_pRB != nullptr)
	{
		l_pRB->m_v3AngularVelocity = velocity;
	}
}


#pragma endregion Extra functions



