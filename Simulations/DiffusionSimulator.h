#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid();
	Grid(unsigned int a_iWidth, unsigned int a_iHeight, unsigned int a_iDepth);
	~Grid();

	int getVal(unsigned int a_iX, unsigned int a_iY, unsigned int a_iZ);
	void setVal(unsigned int a_iX, unsigned int a_iY, unsigned int a_iZ, int a_iValue);

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
	int* m_pArrData = nullptr;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

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
	Grid* diffuseTemperatureExplicit();
	void diffuseTemperatureImplicit();

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid *T; //save results of every time step

};

#endif