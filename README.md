# 2023-code-CDC-On Stability Analysis of Predictive Flocking using N-Paths

## General

This repository the code for the paper

> P.Hastedt and H. Werner, "On Stability Analysis of Predictive Flocking Using N-Paths"

presented at IEEE CDC, 2023.

It may be used to recreate and validate the simulation results and figures from the paper. 

## N-Path Generation

The directory `N-Paths` contains code for generating arbitrary N-Paths and checking whether there exists a path satisfying Lemma 1. In order to execute the simulation, run `NPaths.m`. Parameters such as dimension and length of the path can be set in the setup section of the script. Some example paths of different lengths and dimensions for which it is not possible to find a path satisfying Lemma 1 can be found in  `N-Paths/examples`.

## Simulation Scenarios
#### Setup
When downloading the code from Zenodo, the MAS-simulation submodule directory `Simulation/mas-simulation` will be empty. This can be resolved by either directly downloading the code for the paper from GitHub or by copying the source code of the [WiMAS library](https://github.com/TUHH-ICS/MAS-Simulation) to the corresponding directory.

#### Simulation

Code for reproducing the simulation scenarios is provided in the directory `CMPF Simulation`

For the simulations, an open source MAS library which can be found [on GitHub](https://github.com/TUHH-ICS/MAS-Simulation) is utilized.

The simulation results will be saved in the `simulation/out` directory and can then be used for evaluation. The centralized predictive flocking algorithm is an implementaiton of the algorithm presented in 
	J. Zhan and X. Li, "Flocking of Multi-Agent Systems Via Model Predictive Control Based on Position-Only Measurements," in IEEE Transactions on Industrial Informatics, vol. 9, no. 1, pp. 377-385, Feb. 2013, doi: 10.1109/TII.2012.2216536


#### Evaluation

The figures in the paper can be reproduced with the `evaluation.m`script

The code in this repository was tested in the following environment:

* *Windows 10* Version 22H2
* *Matlab* 2023a
