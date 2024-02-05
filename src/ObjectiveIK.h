#pragma once
#ifndef OBJECTIVE_IK_H
#define OBJECTIVE_IK_H

#include "Objective.h"

class ObjectiveIK : public Objective
{
public:
	ObjectiveIK();
	virtual ~ObjectiveIK();
	virtual double evalObjective(const Eigen::VectorXd &x) const;
	virtual double evalObjective(const Eigen::VectorXd &x, Eigen::VectorXd &g) const;
	virtual double evalObjective(const Eigen::VectorXd &x, Eigen::VectorXd &g, Eigen::MatrixXd &H) const;

	
	Eigen::MatrixXd ptarget;
	std::vector<double> wreg;

private:

};

#endif
