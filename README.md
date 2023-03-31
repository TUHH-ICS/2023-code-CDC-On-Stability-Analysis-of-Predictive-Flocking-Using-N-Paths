# 2023-code-CDC-On Stability Analysis of Predictive Flocking using N-Paths

## General

This repository the code for the paper

> P.Hastedt and H. Werner, "On Stability Analysis of Predictive Flocking using N-Paths"

submitted to IEEE CDC, 2023.

It may be used to recreate and validate the simulation results and figures from the paper. 

## N-Path Generation

The directory `N-Paths` contains code for generating arbitrary N-Paths and checking whether there exists a path satisfying Lemma 1. In order to execute the simulation, run `simulation.m`. Some example paths of different lengths and dimensions for which it is not possible to find a path satisfying Lemma 1 can be found in  `N-Paths/examples`.

## Simulation Scenarios
#### Setup
When downloading the code from Zenodo, the MAS-simulation submodule directory `simulation/MAS-simulation` will be empty. This can be resolved by either directly downloading the code for the paper from GitHub or by copying the source code of the [MAS library](https://github.com/TUHH-ICS/MAS-Simulation) to the corresponding directory.

#### Simulation

Code for reproducing the simulation scenarios is provided in the directory `MPF Simulation`

For the simulations, an open source MAS library which can be found [on GitHub](https://github.com/TUHH-ICS/MAS-Simulation) is utilized.

At the top of `simulation.m`, the algorithms, scenarios, and configuration files to be simulated can be selected by changing the `algorithmIndex` variable. The simulation results will be saved in the `simulation/out` directory and can then be used for evaluation. Two algorithms are provided:
- the centralized model predictive flocking algorithm from the paper 
	J. Zhan and X. Li, "Flocking of Multi-Agent Systems Via Model Predictive Control Based on Position-Only Measurements," in IEEE Transactions on Industrial Informatics, vol. 9, no. 1, pp. 377-385, Feb. 2013, doi: 10.1109/TII.2012.2216536
	
- the distributed model predictive flocking algorithm from the paper
	H. -T. Zhang, Z. Cheng, G. Chen and C. Li, "Model predictive flocking control for second-order multi-agent systems with input constraints," in IEEE Transactions on Circuits and Systems I: Regular Papers, vol. 62, no. 6, pp. 1599-1606, June 2015, doi: 10.1109/TCSI.2015.2418871.

Simulation results using the second algorithm are only presented in the extended version of the paper.

#### Evaluation

At the top of `evaluation.m`, the results of the paper can be reproduced by setting the `evaluationIndex`. 

The code in this repository was tested in the following environment:

* *Windows 10* Version 21H2
* *Matlab* 2021a
