#include "approach.h"

void CSDoF::InputParameter(double mass, double stiffness, double DampingRatio, double VeloInit, double DisplInit)
{
	m = mass;
	k = stiffness;
	ur = DampingRatio;
	dVelo_0 = VeloInit;
	dDispl_0 = DisplInit;
}

void CSDoF::ComputeFrequencyPeriod()
{
	w = sqrt(k / m);
	f = w / 2.0 / PI;
	T = 1.0 / f;
	uc = 2.0*sqrt(k*m);
}

void CSDoF::InitializationBeforeComputaton()
{
	// [initial] condition.
	dVelo_i = dVelo_0;
	dDisplAccum_i = dDispl_0;
	ComputeFrequencyPeriod();
	uu = uc * ur;
}

// implicit newmark-b-r analysis.
void CNewmark_Implicit::Analysis()
{
	// [initial] condition.
	pSDoF->dVelo_0 = 0;
	pSDoF->dDispl_0 = 0.1;
	pSDoF->InitializationBeforeComputaton();
	// using local variables.
	double w = pSDoF->w;
	double ur = pSDoF->ur;
	// [step-based computation].
	int i_accum = 0;
	double t_accum = 0;
	// [record].
	v_AccumTime.push_back(t_accum);
	v_StepVelo.push_back(pSDoF->dVelo_i);
	v_StepDisplAccum.push_back(pSDoF->dDisplAccum_i);
	while (t_accum < tt)
	{
		// update accumulated time.
		i_accum++;
		t_accum += hh;

		// compute the displacemet and velocity
		// at the end of current time step.


		// [1] di, vi, ai;
		double di = pSDoF->dDisplAccum_i;
		double vi = pSDoF->dVelo_i;
		double ai = -(w*w)*pSDoF->dDisplAccum_i - 2.0*ur*w*pSDoF->dVelo_i;

		double a0 = 1 + w * w*Newmark_B*hh*hh + 2.0*ur*w*Newmark_R*hh;
		double a1 = w * w;
		double a2 = w * w*hh + 2 * ur*w;
		double a3 = w * w*hh*hh*(0.5 - Newmark_B) + 2.0*ur*w*hh*(1 - Newmark_R);

		// [2] ai1, vi1, di1.
		double ai1 = -(a1 * di + a2 * vi + a3 * ai) / a0;
		double vi1 = vi + hh * (1 - Newmark_R)*ai + hh * Newmark_R*ai1;
		double di1 = di + hh * vi + hh * hh*(0.5 - Newmark_B)*ai + hh * hh*Newmark_B*ai1;

		// [3] update.
		pSDoF->dVelo_i = vi1 * gg;
		pSDoF->dDisplAccum_i = di1;

		// record.
		v_AccumTime.push_back(t_accum);
		v_StepVelo.push_back(pSDoF->dVelo_i);
		v_StepDisplAccum.push_back(pSDoF->dDisplAccum_i);
	}

	// output computation results.
	fstream fout;
	fout.open("output_Newmark.txt", fstream::out | fstream::trunc);
	if (fout.is_open())
	{
		fout << "--------------------------------------------------\n";
		fout << "### Mass point displacement & velocity record ####\n";
		fout << "--------------------------------------------------\n";
		fout << " m  = " << pSDoF->m << " kg \n";
		fout << " k  = " << pSDoF->k << " N/m \n";
		fout << " d0 = " << pSDoF->dDispl_0 << " m \n";
		fout << " v0 = " << pSDoF->dVelo_0 << " m/s \n";
		fout << " w  = " << pSDoF->w << " hz \n";
		fout << " f  = " << pSDoF->f << " hz \n";
		fout << " T  = " << pSDoF->T << " s \n";
		fout << " ur = " << pSDoF->ur << "\n";
		fout << " uc = " << pSDoF->uc << " kg/s \n";
		fout << "--------------------------------------------------\n";
		fout << " Newmark_B  = " << Newmark_B << "\n";
		fout << " Newmark_R  = " << Newmark_R << "\n";
		fout << " gg in NPC  = " << gg << "\n";
		fout << " step size  = " << hh << "\n";
		fout << " total time = " << tt << "\n";
		fout << "--------------------------------------------------\n";
		fout << "| StepNo | StepAccumTime | DisplAccum | Velo |    \n";
		fout << "--------------------------------------------------\n";
		for (int is = 0; is <= i_accum; is++)
		{
			fout << is << "\t" << v_AccumTime[is] << "\t" << v_StepDisplAccum[is] << "\t" << v_StepVelo[is] << "\n";
		}
		fout << "#End#\n";
		cout << "[Msg] Finish writing the computation data ...\n";
		fout.close();
	}
	else { cout << "[Error] in openning str_record !\n"; }
}

// Input computation parameters.
void CNewmark_PredictorCorrector::InputParameter(double N_B, double N_R, double KineticDampingRatio, double TimeInterval, double TimeTotal)
{
	Newmark_B = N_B;
	Newmark_R = N_R;
	gg = KineticDampingRatio;
	hh = TimeInterval;
	tt = TimeTotal;
}

// predictor-solution-corrector approach.
void CNewmark_PredictorCorrector::Analysis()
{
	// [initial] condition.
	pSDoF->dVelo_0 = 0;
	pSDoF->dDispl_0 = 0.1;
	pSDoF->InitializationBeforeComputaton();
	// [step-based computation].
	int i_accum = 0;
	double t_accum = 0;
	// [record].
	v_AccumTime.push_back(t_accum);
	v_StepVelo.push_back(pSDoF->dVelo_i);
	v_StepDisplAccum.push_back(pSDoF->dDisplAccum_i);
	while (t_accum < tt)
	{
		// update accumulated time.
		i_accum++;
		t_accum += hh;

		// compute the displacemet and velocity
		// at the end of current time step.
		
		// [1] predictor.
		double an = (-(pSDoF->k * pSDoF->dDisplAccum_i) - pSDoF->uu * pSDoF->dVelo_i) / pSDoF->m;
		double dn1_pdt = pSDoF->dDisplAccum_i + hh * pSDoF->dVelo_i + 0.5*(hh*hh)*(1.0 - 2.0*Newmark_B)*an;
		double vn1_pdt = pSDoF->dVelo_i + hh * (1.0 - Newmark_R) * an;
		// [2] solution.
		double an1_sln = (-pSDoF->k*(dn1_pdt - pSDoF->uu * vn1_pdt)) / pSDoF->m;;
		// [3] corrector.
		double dn1 = dn1_pdt + Newmark_B * hh * hh * an1_sln;
		double vn1 = vn1_pdt + Newmark_R * hh * an1_sln;
		
		// update mass point coordinate.
		// [#] viscous artificial damping proportional to the velocity.
		pSDoF->dVelo_i = vn1 * gg;
		pSDoF->dDisplAccum_i = dn1;
		
		// record.
		v_AccumTime.push_back(t_accum);
		v_StepVelo.push_back(pSDoF->dVelo_i);
		v_StepDisplAccum.push_back(pSDoF->dDisplAccum_i);
	}

	// output computation results.
	fstream fout;
	fout.open("output_NPC.txt", fstream::out | fstream::trunc);
	if (fout.is_open())
	{
		fout << "--------------------------------------------------\n";
		fout << "### Mass point displacement & velocity record ####\n";
		fout << "--------------------------------------------------\n";
		fout << " m  = " << pSDoF->m << " kg \n";
		fout << " k  = " << pSDoF->k << " N/m \n";
		fout << " d0 = " << pSDoF->dDispl_0 << " m \n";
		fout << " v0 = " << pSDoF->dVelo_0 << " m/s \n";
		fout << " w  = " << pSDoF->w << " hz \n";
		fout << " f  = " << pSDoF->f << " hz \n";
		fout << " T  = " << pSDoF->T << " s \n";
		fout << " ur = " << pSDoF->ur << "\n";
		fout << " uc = " << pSDoF->uc << " kg/s \n";
		fout << "--------------------------------------------------\n";
		fout << " Newmark_B  = " << Newmark_B << "\n";
		fout << " Newmark_R  = " << Newmark_R << "\n";
		fout << " gg in NPC  = " << gg << "\n";
		fout << " step size  = " << hh << "\n";
		fout << " total time = " << tt << "\n";
		fout << "--------------------------------------------------\n";
		fout << "| StepNo | StepAccumTime | DisplAccum | Velo |    \n";
		fout << "--------------------------------------------------\n";
		for (int is = 0; is <= i_accum; is++)
		{
			fout << is << "\t" << v_AccumTime[is] << "\t" << v_StepDisplAccum[is] << "\t" << v_StepVelo[is] << "\n";
		}
		fout << "#End#\n";
		cout << "[Msg] Finish writing the computation data ...\n";
		fout.close();
	}
	else { cout << "[Error] in openning str_record !\n"; }
	
}