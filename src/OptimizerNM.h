#pragma once
#ifndef OPTIMIZER_NM_H
#define OPTIMIZER_NM_H

#include "Optimizer.h"

class Objective;

class OptimizerNM : public Optimizer
{
public:
	OptimizerNM();
	virtual ~OptimizerNM();
	virtual Eigen::VectorXd optimize(const std::shared_ptr<Objective> objective, const Eigen::VectorXd &xInit);

	int iterationsUsed;
};

#endif
