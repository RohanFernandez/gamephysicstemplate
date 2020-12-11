#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	/// <summary>
	/// Is the first update on a new case
	/// </summary>
	bool m_bIsFirstRunOnNewCase = true;

	//USER DEFINED


	class RBCube
	{
		public:

			Vec3 m_v3CenterPosition				= { 0.0f, 0.0f ,0.0f };
			Vec3 m_v3Dimensions					= { 0.0f, 0.0f ,0.0f };
			Quat m_quatRotation					= { 0.0f, 0.0f, 0.0f, 0.0f };;
			Vec3 m_v3AngularVelocity			= { 0.0f, 0.0f ,0.0f };
			int m_iMass = 0;
			Vec3 m_v3LinearVelocity				= { 0.0f, 0.0f ,0.0f };
			Vec3 m_v3Torque						= {0.0f, 0.0f ,0.0f};
			Vec3 m_v3AngularMomentum			= { 0.0f, 0.0f ,0.0f };
			Mat4 m_m4InvInertiaTensor = Mat4();

		private:
			Mat4 m_m4Translation = Mat4();
			Mat4 m_m4Scale = Mat4();
			Mat4 m_m4Rotation = Mat4();

		public:
			RBCube() {};
			~RBCube() {};

			Mat4 getTransformation()
			{
				m_m4Translation.initTranslation(m_v3CenterPosition.x, m_v3CenterPosition.y, m_v3CenterPosition.z);
				m_m4Scale.initScaling(m_v3Dimensions.x, m_v3Dimensions.y, m_v3Dimensions.z);
				m_m4Rotation = m_quatRotation.getRotMat();
				return m_m4Scale * m_m4Rotation * m_m4Translation;
			}

			void reset()
			{
				m_v3CenterPosition		= { 0.0f, 0.0f, 0.0f };
				m_v3Dimensions			= { 1.0f, 1.0f, 1.0f };
				m_quatRotation			= {0.0f, 0.0f, 0.0f, 0.0f};
				m_iMass = 0;
				m_v3LinearVelocity		= { 0.0f, 0.0f, 0.0f };
				m_v3AngularVelocity		= { 0.0f, 0.0f, 0.0f };
				m_v3Torque				= { 0.0f, 0.0f, 0.0f };
				m_v3AngularMomentum     = { 0.0f, 0.0f, 0.0f };
				m_m4InvInertiaTensor.initId();
				m_m4Rotation.initId();
			}
	};

	//The max rigidbodies that are pooled on start
	static constexpr unsigned int MAX_RIGIDBODIES = 50;

	//the current rigid bodies that are active in the scene
	unsigned int m_iActiveRigidBodies = 0;

	Vec3 m_v3ExternalForce = { 0.0f, 0.0f, 0.0f };

	//The vector that holds all the rigid body objects
	std::vector<RBCube> m_vectRigidBodies;

	// Returns  the rigid body at index if it exists in the vector else returns nullptr
	RBCube* getRigidBody(unsigned int a_iIndex);

	//Simulate rigid bodies with the set values
	void simulateRigidBodies(float a_fTimeStep);

	// Draws all rigidbodies
	void drawRigidBodies();



	};
#endif