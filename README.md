# sdof_solution
Test different time integration approaches for single degree of freedom (SDOF) system.
-----
To use this program, users should : see the folder "exp".
[1] prepare the "input.txt" file (the instruction is in "input_instruction.txt").
[2] compile and run the program in the same path with the "input.txt" file.
[3] Analysis mode : 1-original Newmark approach; 2-Newmark-Predictor-Corrector approach.
[4] The displacement and velocity of the mass point is recorded in "output_Newmark.txt" and "output_NPC.txt" file within the same folder.
-----
The Newmark-Predictor-Corrector(NPC) time integration approach was added for SDOF problem.
The original Newmark time integration approach was added for SDOF problem.
-----
A journal paper can be founded as a reference for the application of the modified predictor-corrector solution approach for discontinuous computation problem: https://onlinelibrary.wiley.com/doi/full/10.1002/nag.2881
Zheng, F., Leung, Y. F., Zhu, J. B., & Jiao, Y. Y. (2018). Modified predictor‐corrector solution approach for efficient discontinuous deformation analysis of jointed rock masses. International Journal for Numerical and Analytical Methods in Geomechanics.
-----
To get more information, you can contact Dr. Fei Zheng by e-mail: feizhengprchina@hotmail.com
