#include "DiffusionSimulator.h"
using namespace std;

Grid::Grid() {
}

Grid::Grid(unsigned int a_iWidth, unsigned int a_iHeight, unsigned int a_iDepth)
	: 
	m_iWidth{ a_iWidth },
	m_iHeight{ a_iHeight },
	m_iDepth{ a_iDepth },
	DATA_CAPACITY{ m_iWidth * m_iHeight * m_iDepth },
	GRID_MAX_INDEX{ DATA_CAPACITY - 1},
	m_pArrData{ new float[DATA_CAPACITY]{} }
{

}

Grid::~Grid()
{
	delete[] m_pArrData;
	m_pArrData = nullptr;
}

float Grid::getVal(unsigned int a_iX, unsigned int a_iY, unsigned int a_iZ)
{
	if ((a_iX >= m_iWidth) || (a_iY >= m_iHeight) || (a_iZ >= m_iDepth))
	{
		std::cout << "ERROR:: Dimensions out of bounds \n";
	}

	unsigned int l_iIndex = (m_iWidth * m_iHeight * a_iZ) + (a_iY * m_iWidth) + a_iX;
	if (l_iIndex < 0 || l_iIndex > GRID_MAX_INDEX) 
	{ 
		std::cout << "ERROR:: Grid index out of bounds\n";
		return 0.0f; 
	}
	return m_pArrData[l_iIndex];
}

void Grid::setVal(unsigned int a_iX, unsigned int a_iY, unsigned int a_iZ, float a_fVal)
{
	if ((a_iX >= m_iWidth) || (a_iY >= m_iHeight) || (a_iZ >= m_iDepth))
	{
		std::cout << "ERROR:: Dimensions out of bounds \n";
	}

	unsigned int l_iIndex = (m_iWidth * m_iHeight * a_iZ) + (a_iY * m_iWidth) + a_iX;
	if (l_iIndex < 0 || l_iIndex > GRID_MAX_INDEX)
	{
		std::cout << "ERROR:: Grid index out of bounds\n";
		return;
	}
	m_pArrData[l_iIndex] = a_fVal;
}

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
}

DiffusionSimulator::~DiffusionSimulator()
{
	if (m_pGrid1 != nullptr)
	{
		delete m_pGrid1;
		m_pGrid1 = nullptr;
	}

	if (m_pGrid2 != nullptr)
	{
		delete m_pGrid2;
		m_pGrid2 = nullptr;
	}
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	TwAddVarCB(DUC->g_pTweakBar, "GRID", TW_TYPE_DIR3F, 
		[](const void* value, void* clientData) { 
			DiffusionSimulator* l_pDiffusionSim = static_cast<DiffusionSimulator*>(clientData);
			const float* l_pValues = static_cast<const float*>(value);
			l_pDiffusionSim->m_iGridX = l_pValues[0];
			l_pDiffusionSim->m_iGridY = l_pValues[1];
			l_pDiffusionSim->m_iGridZ = l_pValues[2];
			l_pDiffusionSim->notifyCaseChanged(l_pDiffusionSim->m_iTestCase);
		}, 
		[](void* value, void* clientData) {
			DiffusionSimulator* l_pDiffusionSim = static_cast<DiffusionSimulator*>(clientData);
			float* l_pValues = static_cast<float*>(value);
			l_pValues[0] = l_pDiffusionSim->m_iGridX;
			l_pValues[1] = l_pDiffusionSim->m_iGridY;
			l_pValues[2] = l_pDiffusionSim->m_iGridZ;
		}, this, "step=1.0"
			);

	TwAddVarRW(DUC->g_pTweakBar, "SphereRadius", TW_TYPE_FLOAT, &m_fSphereRadius, "step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "CubeDimension", TW_TYPE_FLOAT, &m_fCubeDimension, "step=0.01");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//

	if (m_pGrid1 != nullptr)
	{
		delete m_pGrid1;
		m_pGrid1 = nullptr;
	}

	if (m_pGrid2 != nullptr)
	{
		delete m_pGrid2;
		m_pGrid2 = nullptr;
	}

	m_fMaxTemperatureReached = 0.0f;

	//Grid Setup
	m_pGrid1 = new Grid(m_iGridX, m_iGridY, m_iGridZ);
	m_pGrid2 = new Grid(m_iGridX, m_iGridY, m_iGridZ);

	m_pNewGrid = m_pGrid1; //set with values on start or default on start
	m_pOldGrid = m_pGrid2; // Is set to 0 on start

	int l_iGridYLimiter = m_iGridY - 1;
	int l_iGridXLimiter = m_iGridX - 1;

	for (int l_iIndexZ = 0; l_iIndexZ < m_iGridZ; l_iIndexZ++)
	{
		for (int l_iIndexY = 1; l_iIndexY < l_iGridYLimiter; l_iIndexY++)
		{
			for (int l_iIndexX = 1; l_iIndexX < l_iGridXLimiter; l_iIndexX++)
			{
				if (!(l_iIndexY == 1 || l_iIndexX == 1))
				{
					m_pNewGrid->setVal(l_iIndexX, l_iIndexY, l_iIndexZ, 10);
				}
			}
		}
	}

	switch (m_iTestCase)
	{
	case 0:
	{
		cout << "Explicit solver!\n";
		break;
	}
	case 1:
		cout << "Implicit solver!\n";

		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(const float& a_fTimeStep) 
{
	float l_fDeltaSqX = (float)m_fCubeDimension / (float)m_iGridX;
	l_fDeltaSqX *= l_fDeltaSqX;

	float l_fDeltaSqY = (float)m_fCubeDimension / (float)m_iGridY;
	l_fDeltaSqY *= l_fDeltaSqY;

	float l_fDeltaSqZ = (float)m_fCubeDimension / (float)m_iGridZ;
	l_fDeltaSqZ *= l_fDeltaSqZ;

	// to avoid changing the border values from 0
	int l_iXLimiter = m_iGridX - 1;
	int l_iYLimiter = m_iGridY - 1;
	int l_iZLimiter = m_iGridZ - 1;

	for (int l_iIndexZ = 0; l_iIndexZ < m_iGridZ; l_iIndexZ++)
	{
		for (int l_iIndexY = 1; l_iIndexY < l_iYLimiter; l_iIndexY++)
		{
			for (int l_iIndexX = 1; l_iIndexX < l_iXLimiter; l_iIndexX++)
			{
				float l_fCurrentIndexTemperature = m_pOldGrid->getVal(l_iIndexX, l_iIndexY, l_iIndexZ);

				float l_fValX = m_pOldGrid->getVal(l_iIndexX + 1, l_iIndexY, l_iIndexZ) - 2 * l_fCurrentIndexTemperature + m_pOldGrid->getVal(l_iIndexX - 1, l_iIndexY, l_iIndexZ) * l_fDeltaSqX;
				float l_fValY = m_pOldGrid->getVal(l_iIndexX, l_iIndexY + 1, l_iIndexZ) - 2 * l_fCurrentIndexTemperature + m_pOldGrid->getVal(l_iIndexX, l_iIndexY - 1, l_iIndexZ) * l_fDeltaSqY;
				float l_fValZ = 0;

				if (l_iZLimiter > l_iIndexZ && l_iIndexZ > 0)
				{
					l_fValZ = m_pOldGrid->getVal(l_iIndexX, l_iIndexY, l_iIndexZ + 1) - 2 * l_fCurrentIndexTemperature + m_pOldGrid->getVal(l_iIndexX, l_iIndexY, l_iIndexZ - 1);
				}

				float l_fTemperature = m_fDiffusionAlpa* a_fTimeStep* (l_fValX + l_fValY + l_fValZ) + l_fCurrentIndexTemperature;
				m_pNewGrid->setVal(l_iIndexX, l_iIndexY, l_iIndexZ, l_fTemperature);
				if (l_fTemperature > m_fMaxTemperatureReached)
				{
					m_fMaxTemperatureReached = l_fTemperature;
				}
			}
		}
	}
}

void DiffusionSimulator::setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	int l_iGridSize = m_iGridX * m_iGridY * m_iGridZ;
	for (int l_iIndexZ = 0; l_iIndexZ < m_iGridZ; l_iIndexZ++)
	{
		for (int l_iIndexY = 0; l_iIndexY < m_iGridY; l_iIndexY++)
		{
			int l_iCurrentRowIndex = (l_iIndexZ * m_iGridY * m_iGridX) + (l_iIndexY * m_iGridX);
			for (int l_iIndexX = 0; l_iIndexX < m_iGridX; l_iIndexX++)
			{
				b.at(l_iCurrentRowIndex + l_iIndexX) = m_pOldGrid->getVal(l_iIndexX, l_iIndexY, l_iIndexZ);
			}
		}
	}
}

void DiffusionSimulator::fillT(std::vector<Real>& x) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero

	int l_iXLimiter = m_iGridX - 1;
	int l_iYLimiter = m_iGridY - 1;
	for (int l_iIndexZ = 0; l_iIndexZ < m_iGridZ; l_iIndexZ++)
	{
		for (int l_iIndexY = 1; l_iIndexY < l_iYLimiter; l_iIndexY++)
		{
			for (int l_iIndexX = 1; l_iIndexX < l_iXLimiter; l_iIndexX++)
			{
				float l_fTemperature = x.at((l_iIndexZ * m_iGridX * m_iGridY) + (l_iIndexY * m_iGridX) + l_iIndexX);
				m_pNewGrid->setVal(l_iIndexX, l_iIndexY, l_iIndexZ, l_fTemperature);
				if (l_fTemperature > m_fMaxTemperatureReached)
				{
					m_fMaxTemperatureReached = l_fTemperature;
				}
			}
		}
	}
}

void DiffusionSimulator::setupA(SparseMatrix<Real>& A, const float& a_fTimeStep) 
{	//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0

	float l_fCoefficient = m_fDiffusionAlpa * a_fTimeStep;
	int l_iGridXLimiter = m_iGridX - 1;
	int l_iGridYLimiter = m_iGridY - 1;
	
	int l_iGridXY = m_iGridX * m_iGridY;
	int l_iGridZLimiterIndex = (m_iGridZ - 1) * l_iGridXY;

	float l_fDeltaSqX = (float)m_fCubeDimension / (float)m_iGridX;
	l_fDeltaSqX *= l_fDeltaSqX;
	float l_fCoefficientX = l_fCoefficient / l_fDeltaSqX;

	float l_fDeltaSqY = (float)m_fCubeDimension / (float)m_iGridY;
	l_fDeltaSqY *= l_fDeltaSqY;
	float l_fCoefficientY = l_fCoefficient / l_fDeltaSqY;

	float l_fDeltaSqZ = (float)m_fCubeDimension / (float)m_iGridZ;
	l_fDeltaSqZ *= l_fDeltaSqZ;
	float l_fCoefficientZ = l_fCoefficient / l_fDeltaSqZ;

	for (int l_iIndexZ = 0; l_iIndexZ < m_iGridZ; l_iIndexZ++)
	{
		for (int l_iIndexY = 0; l_iIndexY < m_iGridY; l_iIndexY++)
		{
			for (int l_iIndexX = 0; l_iIndexX < m_iGridX; l_iIndexX++)
			{
				//Check if border index
				if ((l_iIndexX == 0 ||
					l_iIndexX == l_iGridXLimiter||
					l_iIndexY == 0 ||
					l_iIndexY == l_iGridYLimiter))
				{
					// If border and diagonal
					if (l_iIndexY == l_iIndexX)
					{
						A.set_element(l_iIndexX, l_iIndexY, 1); // set diagonal of boundary values to 1.0
					}
					// If border and not diagonal
					else
					{
						A.set_element(l_iIndexX, l_iIndexY, 0);
					}
				}
				else
				{
					int l_iCurrentIndex = (l_iIndexZ * l_iGridXY) + (l_iIndexY * m_iGridX) + l_iIndexX;

					A.set_element(l_iCurrentIndex, l_iCurrentIndex - 1, -l_fCoefficientX); //x - 1
					A.set_element(l_iCurrentIndex, l_iCurrentIndex + 1, -l_fCoefficientX); //x + 1
					A.set_element(l_iCurrentIndex, l_iCurrentIndex - m_iGridX, -l_fCoefficientY); // y - 1
					A.set_element(l_iCurrentIndex, l_iCurrentIndex + m_iGridX, -l_fCoefficientY); // y + 1

					if (l_iCurrentIndex > l_iGridXY && l_iCurrentIndex < l_iGridZLimiterIndex)
					{
						A.set_element(l_iCurrentIndex, l_iCurrentIndex - l_iGridXY, -l_fCoefficientZ); // z - 1 
						A.set_element(l_iCurrentIndex, l_iCurrentIndex + l_iGridXY, -l_fCoefficientZ); // z + 1
					}
					A.set_element(l_iCurrentIndex, l_iCurrentIndex, 1.0f + 2.0f * (l_fCoefficientX + l_fCoefficientY + l_fCoefficientZ)); // x, y, z
				}
			}
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(const float& a_fTimeStep) {//add your own parameters
	// solve A T = b
	// to be implemented
	int l_iGridElementCount = m_iGridX * m_iGridY * m_iGridZ;
	//const int N = l_iGridElementCount;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> A(l_iGridElementCount);
	std::vector<Real> b(l_iGridElementCount);

	setupA(A, a_fTimeStep);
	setupB(b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(l_iGridElementCount);
	for (int j = 0; j < l_iGridElementCount; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(x);//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	Grid* l_pTemp = m_pOldGrid;
	m_pOldGrid = m_pNewGrid;
	m_pNewGrid = l_pTemp;

	switch (m_iTestCase)
	{
	case 0:
		{
			diffuseTemperatureExplicit(timeStep);
			break;
		}
	case 1:
		{
			diffuseTemperatureImplicit(timeStep);
			break;
		}
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization

	Vec3 l_v3SphereScale = { m_fSphereRadius ,m_fSphereRadius ,m_fSphereRadius };

	float l_fDeltaX = (float)m_fCubeDimension / (float)m_iGridX;
	float l_fDeltaY = (float)m_fCubeDimension / (float)m_iGridY;
	float l_fDeltaZ = (float)m_fCubeDimension / (float)m_iGridZ;
	for (int l_iIndexZ = 0; l_iIndexZ < m_iGridZ; l_iIndexZ++)
	{
		float l_fXPoZ = (m_fCubeDimension * -0.5f) + l_fDeltaZ * l_iIndexZ;
		for (int l_iIndexY = 0; l_iIndexY < m_iGridY; l_iIndexY++)
		{
			float l_fYPos = (m_fCubeDimension * -0.5f) + l_fDeltaY * l_iIndexY;
			for (int l_iIndexX = 0; l_iIndexX < m_iGridX; l_iIndexX++)
			{
			 	float l_fNewValue = m_pNewGrid->getVal(l_iIndexX, l_iIndexY, l_iIndexZ);

				float l_fNormalizedTemperature = l_fNewValue / m_fMaxTemperatureReached;

				DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100,  Vec3(1.0f, l_fNormalizedTemperature, l_fNormalizedTemperature));
				DUC->drawSphere({ (m_fCubeDimension * -0.5f) + l_fDeltaX * l_iIndexX, l_fYPos, l_fXPoZ }, l_v3SphereScale);
			}
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
