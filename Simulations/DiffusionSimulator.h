#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

struct GridValues
{
	float m_fOldVal = 0.0f;
	float m_fNewVal = 0.0f;
};

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid();
	Grid(unsigned int a_iWidth, unsigned int a_iHeight, unsigned int a_iDepth);
	~Grid();

	GridValues* getVal(unsigned int a_iX, unsigned int a_iY, unsigned int a_iZ);

	inline unsigned int getWidth() { return m_iWidth; }
	inline unsigned int getHeight() { return m_iHeight; }
	inline unsigned int getDepth() { return m_iDepth; }

private:
	// Attributes
	unsigned int m_iWidth	= 0;
	unsigned int m_iHeight	= 0;
	unsigned int m_iDepth	= 0;
	const unsigned int DATA_CAPACITY = 0;
	const unsigned int GRID_MAX_INDEX = 0;
	GridValues* m_pArrData = nullptr;
};

class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	//Destructors
	~DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	void diffuseTemperatureExplicit(const float& a_fTimeStep);
	void diffuseTemperatureImplicit();

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	unsigned int m_iGridX = 16;
	unsigned int m_iGridY = 16;
	unsigned int m_iGridZ = 1;
	float m_fSphereRadius = 0.04f;
	float m_fCubeDimension = 1.0f;
	Grid* m_pGrid = nullptr; //save results of every time step

};

#endif