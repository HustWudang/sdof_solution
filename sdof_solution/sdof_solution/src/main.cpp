/*
*	The main module for test time integration approaches
*	for single-degree-of-freedom problem
*/

#include "approach.h"

using namespace std;

int main()
{
	
	// Initial reminder.
	cout << "------------------------------------------------------\n";
	cout << "# Test time integration approach for SDoF system ... #\n";
	cout << "------------------------------------------------------\n";
	cout << "# [Note] Please prepare the data in 'input.txt'.     #\n";
	cout << "------------------------------------------------------\n";
	cout << "# >> Analysis Mode (please type in & press enter):    \n";
	cout << "# [1] Newmark-B ; [2] Newmark Predictor-Corrector.    \n";
	cout << "------------------------------------------------------\n";
	cout << "# 'output_Newmark.txt' is the result of Newmark_B.   #\n";
	cout << "# 'output_NPC.txt' is the result of Newmark_PC.      #\n";
	cout << "------------------------------------------------------\n";

	int IMODE = 0;

	cin >> IMODE;


	if (IMODE == 1)
	{
		// create an analysis object.
		CNewmark_Implicit mNewmark;
		mNewmark.pSDoF = new CSDoF;

		// open input.txt and load all parameters.
		fstream finput;
		finput.open("input.txt");
		if (finput.is_open())
		{
			finput >> mNewmark.pSDoF->m >> mNewmark.pSDoF->k >> mNewmark.pSDoF->ur >> mNewmark.pSDoF->dDispl_0 >> mNewmark.pSDoF->dVelo_0;
			finput >> mNewmark.Newmark_B >> mNewmark.Newmark_R >> mNewmark.gg >> mNewmark.hh >> mNewmark.tt;
			cout << "[Msg] load all input parameter successfully ...\n";
			finput.close();
		}
		else
		{
			cout << "[Error] in openning input.txt !\n";
		}

		// analyses.
		mNewmark.Analysis();
	}

	else if (IMODE == 2)
	{
		// create an analysis object.
		CNewmark_PredictorCorrector mNPC;
		mNPC.pSDoF = new CSDoF;

		// open input.txt and load all parameters.
		fstream finput;
		finput.open("input.txt");
		if (finput.is_open())
		{
			finput >> mNPC.pSDoF->m >> mNPC.pSDoF->k >> mNPC.pSDoF->ur >> mNPC.pSDoF->dDispl_0 >> mNPC.pSDoF->dVelo_0;
			finput >> mNPC.Newmark_B >> mNPC.Newmark_R >> mNPC.gg >> mNPC.hh >> mNPC.tt;
			cout << "[Msg] load all input parameter successfully ...\n";
			finput.close();
		}
		else
		{
			cout << "[Error] in openning input.txt !\n";
		}

		// analyses.
		mNPC.Analysis();
	}

	else
	{
		cout << "[Error] wrong analysis mode !\n";
	}
	

	// pause.
	system("pause");

	// return.
	return 0;
}