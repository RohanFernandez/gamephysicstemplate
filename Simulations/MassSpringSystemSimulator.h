#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

#include <vector>

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

private:
	static constexpr int MAX_MASS_POINTS_INDEX = 20;
	static constexpr int MAX_SPRING_OBJECTS_INDEX = 20;
	
	/// <summary>
	/// The index of the last mass object in the mass object vector container
	/// -1 means no mass object is active
	/// </summary>
	int m_iCurrentActiveLastMassPointIndex = -1;

	/// <summary>
	/// The index of the last spring in the spring vector container
	/// -1 means no spring object is active
	/// </summary>
	int m_iCurrentActiveLastSpringIndex = -1;

	/// <summary>
	/// The scale of the sphere
	/// </summary>
	float m_fSphereScale = 0.05f;

	/// <summary>
	/// The Y position of the ground
	/// </summary>
	float m_fGroundY = -1.0f;

	/// <summary>
	/// The point in the environment that exerts the explosion onto all the objects
	/// The direction of the wind is from the this point to the center
	/// </summary>
	Vec3 m_v3ExplosionPoint;
	float m_ExplosionForceScalar = 0.0f;
	bool m_bExplode = false;

	class MassPoint
	{
	public:
		MassPoint() {};
		~MassPoint() {};

		int m_iID = 0;
		Vec3 m_v3Position;
		Vec3 m_v3Velocity;
		Vec3 m_v3Force;

		Vec3 m_v3PosTemp;
		Vec3 m_v3VelTemp;
		Vec3 m_v3ForceTemp;

		/// <summary>
		/// The mass does not move if true
		/// </summary>
		bool m_bIsFixed = false;

		bool m_bIsColliding = false;

		void reset()
		{
			m_iID = 0;
			m_v3Position = {0.0f, 0.0f, 0.0f};
			m_v3Velocity = { 0.0f, 0.0f, 0.0f };
			m_v3Force = { 0.0f, 0.0f, 0.0f };
			m_bIsFixed = false;

			m_v3PosTemp = {0.0f, 0.0f, 0.0f};
			m_v3VelTemp = {0.0f, 0.0f, 0.0f};
			m_v3ForceTemp = { 0.0f, 0.0f, 0.0f };

			m_bIsColliding = false;
		}

		void drawMassObject(DrawingUtilitiesClass* a_pDUC, Vec3 a_v3Color, Vec3 a_v3Scale)
		{
			a_pDUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, a_v3Color);
			a_pDUC->drawSphere(m_v3Position, a_v3Scale);
		}

		void resetForce()
		{
			m_v3Force = { 0.0f, 0.0f, 0.0f };
			m_v3ForceTemp = { 0.0f, 0.0f, 0.0f };
		}
	};

	class SpringObject
	{
	public:
		SpringObject() {}
		~SpringObject() {}

		int m_ID = 0;
		int m_iMassPointID1 = MassSpringSystemSimulator::MAX_MASS_POINTS_INDEX;
		int m_iMassPointID2 = MassSpringSystemSimulator::MAX_MASS_POINTS_INDEX;
		float m_fRestLength = 0;
		float m_fCurrentLength = 0;


		void reset()
		{
			m_ID = 0;
			m_iMassPointID1 = MassSpringSystemSimulator::MAX_MASS_POINTS_INDEX;
			m_iMassPointID2 = MassSpringSystemSimulator::MAX_MASS_POINTS_INDEX;
			m_fRestLength = 0;
			m_fCurrentLength = 0;
		}

		void drawSpring(DrawingUtilitiesClass* a_DUC, std::vector<MassPoint>& a_vectMassPoint)
		{
			a_DUC->beginLine();
			a_DUC->drawLine(a_vectMassPoint[m_iMassPointID1].m_v3Position, { 0.0f, 0.0f, 1.0f }, a_vectMassPoint[m_iMassPointID2].m_v3Position, { 1.0f, 1.0f, 0.0f });
			a_DUC->endLine();
		}
	};

	/// <summary>
	/// The container that stores all the Mass points
	/// </summary>
	std::vector<MassPoint> m_vectMassPoints;

	/// <summary>
	/// The container that stores all the spring objects
	/// </summary>
	std::vector<SpringObject> m_vectSpringObjects;

	/// <summary>
	/// removes all the active springs and mass objects
	/// </summary>
	void resetAllEntities();

	/// <summary>
	/// Calculates the euler integrator values of force, velocity and position of the 2 objects of the spring input
	/// </summary>
	/// <param name="a_SpringObject"></param>
	/// <param name="a_fTimeStep"></param>
	void calculateEuler(SpringObject& a_SpringObject, const float& a_fTimeStep);

	/// <summary>
	/// Calculates the euler integrator values of force, velocity and position of the input mass object
	/// </summary>
	/// <param name="a_MassPoint"></param>
	/// <param name="a_fTimeStep"></param>
	void calculateEuler(MassPoint& a_MassPoint, const float& a_fTimeStep);

	/// <summary>
	/// Calculates the midpoint integrator values of all the springs
	/// </summary>
	/// <param name="a_fTimeStep"></param>
	void calculateMidpoint(float a_fTimeStep);

	/// <summary>
	/// Calculates the midpoint integrator values at t/2
	/// </summary>
	/// <param name="a_MassPoint"></param>
	/// <param name="a_fTimeStep"></param>
	void calculateMidpointAtHalfstep(MassPoint& a_MassPoint, const float& a_fTimeStep);

	/// <summary>
	/// Calculates the midpoint integrator values at t
	/// uses the accumulated force of a mass object at half step
	/// </summary>
	/// <param name="a_MassPoint"></param>
	/// <param name="a_fTimeStep"></param>
	void calculateMidpointVelocityAtT(MassPoint& a_MassPoint, const float& a_fTimeStep);

	/// <summary>
	/// Calculates leap -frog accumulator values of all the mass objects
	/// </summary>
	/// <param name="a_MassPoint"></param>
	/// <param name="a_fTimeStep"></param>
	void calculateLeapFrog(MassPoint& a_MassPoint, float a_fTimeStep);

	void initializeDemo1();
	void initializeDemo2();
	void initializeDemo3();
	void initializeDemo4();

	/// <summary>
	/// Is the sphere colliding with the ground or any other sphere
	/// </summary>
	/// <param name="a_MassPoint"></param>
	/// <returns></returns>
	bool isSphereCollided(const MassPoint& a_MassPoint);


	/// <summary>
	/// Get length of the vector
	/// </summary>
	/// <param name="a_Vec3"></param>
	/// <returns></returns>
	static float GetV3Length(const Vec3& a_Vec3);

	/// <summary>
	/// Get vector3 direction
	/// </summary>
	/// <param name="a_Vec3"></param>
	/// <returns></returns>
	static Vec3 GetV3Direction(const Vec3& a_Vec3);

	/// <summary>
	/// Sets the force on a mass object by accumulating the force by all the connected springs
	/// </summary>
	/// <param name="a_SpringObject"></param>
	/// <param name="l_bIsTempForce"></param>
	void calculateForce(SpringObject& a_SpringObject, bool l_bIsTempForce = false);

	/// <summary>
	/// returns the force exerted from the explosion on the mass object
	/// m_bIsExplode should be true in the frame i.e. Explode scalar should not be 0 and mouse should be clicked in that frame
	/// </summary>
	/// <param name="a_MassPoint"></param>
	/// <returns></returns>
	Vec3 getExplosionForceExertedOnMass(const MassPoint& a_MassPoint);

	/// <summary>
	/// The gravity in the scene
	/// </summary>
	float m_fGravity = 0;

};

#endif