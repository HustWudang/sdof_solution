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
	cout << "# [Note] please prepare the data in 'input.txt'.     #\n";
	cout << "#      'output.txt' is the default output file name. #\n";
	cout << "------------------------------------------------------\n";

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

	// pause.
	system("pause");

	// return.
	return 0;
}