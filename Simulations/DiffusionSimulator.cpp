#include "DiffusionSimulator.h"
#include "pcgsolver.h"
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
	m_pArrData{ new int[DATA_CAPACITY]{0} }
{

}

Grid::~Grid()
{
	delete[] m_pArrData;
	m_pArrData = nullptr;
}

int Grid::getVal(unsigned int a_iX, unsigned int a_iY, unsigned int a_iZ)
{
	if ((a_iX >= m_iWidth) || (a_iY >= m_iHeight) || (a_iZ >= m_iDepth))
	{
		std::cout << "ERROR:: Dimensions out of bounds \n";
	}

	unsigned int l_iIndex = (m_iWidth * m_iHeight * a_iZ) + (a_iY * m_iWidth) + a_iX;
	if (l_iIndex < 0 || l_iIndex > GRID_MAX_INDEX) 
	{ 
		std::cout << "ERROR:: Grid index out of bounds\n";
		return 0; 
	}
	return m_pArrData[l_iIndex];
}

void Grid::setVal(unsigned int a_iX, unsigned int a_iY, unsigned int a_iZ, int a_iValue)
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
	m_pArrData[l_iIndex] = a_iValue;
}

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
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
	// to be implemented
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit() {//add your own parameters
	Grid* newT = new Grid();
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	return newT;
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}

void fillT() {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
}

void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
			A.set_element(i, i, 1); // set diagonal
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = 25;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, 0.1);
	setupB(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT();//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit();
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
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
