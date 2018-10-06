#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <vector>

const double PI = 3.141592654;

using namespace std;

/*
*
* d = d0 * cos(w*t)
* w = sqrt(k/m)
*/
class CSDoF
{
public:
	double m; // mass.
	double k; // spring stiffness.
	double w; // undamped circular frenquency.
	double uu; // damping coefficient.
	double uc; //critical damping coefficient.
	double ur; // damping ratio (the applied damping coefficient to critical damping coefficient).
	double T; // undamped period.
	double f; // undamped frequency.
	double dDispl_0;
	double dVelo_0;
	double dVelo_i;
	double dDisplAccum_i;
	void InputParameter(double mass, double stiffness, double DampingRatio, double VeloInit, double DisplInit);
	void ComputeFrequencyPeriod();
	void InitializationBeforeComputaton();
};

class CNewmark_Implicit
{
public:
	CSDoF *pSDoF;
	double Newmark_B; //collocation parameter b in Newmark approach.
	double Newmark_R; //collocation parameter r in Newmark approach.
	double gg; //viscous damping ratio propotional to the velocity.
	double hh; //time step size.
	double tt; //total time.
	vector<double> v_AccumTime;
	vector<double> v_StepVelo;
	vector<double> v_StepDisplAccum;
	// implicit newmark-b-r analysis.
	void Analysis();
};

class CNewmark_PredictorCorrector
{
public:
	CSDoF *pSDoF; // pointer to the SDoF object.
	double Newmark_B; //collocation parameter b in Newmark approach.
	double Newmark_R; //collocation parameter r in Newmark approach.
	double gg; //viscous damping ratio propotional to the velocity.
	double hh; //time step size.
	double tt; //total time.
	vector<double> v_AccumTime;
	vector<double> v_StepVelo;
	vector<double> v_StepDisplAccum;
	// Input computation parameters.
	void InputParameter(double N_B, double N_R, double KineticDampingRatio, double TimeInterval, double TimeTotal);
	// predictor-solution-corrector approach.
	void Analysis();
};