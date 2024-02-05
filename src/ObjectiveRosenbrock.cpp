#include "ObjectiveRosenbrock.h"
#include <cmath>

using namespace Eigen;

ObjectiveRosenbrock::ObjectiveRosenbrock(double a, double b)
{
	this->a = a;
	this->b = b;
}

ObjectiveRosenbrock::~ObjectiveRosenbrock()
{

}

double ObjectiveRosenbrock::evalObjective(const VectorXd &x) const //x is input point 2D
{
	//f(x) = (a-x0)^2 + b*(x1 - x0^2)^2
	//double a = 1.0, b = 1.0;
	double part1 = pow((a - x[0]), 2);
	double part2 = b * pow(x[1] - pow(x[0], 2), 2);
	double f = part1 + part2;

	return f;
}

double ObjectiveRosenbrock::evalObjective(const VectorXd &x, VectorXd &g) const
{
	double f = evalObjective(x);
	//double a = 1.0, b = 1.0;
	double top = -1 * (a - x[0]) - 2 * b * x[0] * (x[1] - pow(x[0], 2));
	double bottom = b * (x[1] - pow(x[0], 2));
	Vector2d gradient;
	gradient << top, bottom;
	gradient = 2 * gradient;

	g = gradient;
	//g(x) = 2*(-(a-x0) - 2bx0(x1-x0^2))
		// = 2*(b(x1- x0^2)
	return f;
}

double ObjectiveRosenbrock::evalObjective(const VectorXd &x, VectorXd &g, MatrixXd &H) const
{
	//H(x) = 2(2b(3x0^2 - x1) + 1		2*-2bx0
		// = 2(-2bx0)					2*b
	Matrix2d Hessian;
	double f = evalObjective(x, g);
	//double a = 1.0, b = 1.0;
	double a11 = 2 * b * (3 * pow(x[0], 2) - x[1]) + 1;
	double a12 = -2 * b * x[0];
	double a21 = a12;
	double a22 = b;
	Hessian << a11, a12,
			   a21, a22;
	Hessian = 2 * Hessian;
	H = Hessian;
	return f;
}
