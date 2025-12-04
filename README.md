# UAV-Aided Vehicular Networks: A Trajectory Optimization Approach for 5G Communication on Highways

IEEE Latin America Transactions

Manuscript ID: 

**Authors**
* Ignacio Vidal
* Sandy Bolufé
* Karel Toledo

## Scripts
This repository contains all scripts required to reproduce the simulation and numerical results presented in the article.

| Script | Description | Related figures or tables |
| --- | --- | --- | --- |
| `NR.m` | Computes error using Newton-Raphson algorithm to try and solve inverse kinematics | `Test.mat` |
| `Error_NN.m` | Calculates error using a Neural network to solve inverse kinematics   |  |
| `Simulacion_cinematica.m` | Simulates the movement of the mechanism following a trajectory and plotting _x_ and _y_ coordinates| `Fcn_Red_Qp_1.m`, `Fcn_Red_Qr_1.m`, `Walk_50.mat` |
| `Simulacion_cinematica_1.m` | Simulates the movement of the mechanism following a trajectory and plotting _x_ and _y_ coordinates| `Fcn_Red_Qp_1.m`, `Fcn_Red_Qr_1.m`, `Walk_50.mat` |
| `Simulacion_cinematica_2.m` | Simulates the movement of the mechanism following a trajectory and plotting _x_ and _y_ coordinates| `Fcn_Red_Qp_1.m`, `Fcn_Red_Qr_1.m`, `Walk_50.mat` |
| `M_perf.py` | Measures average execution time of neural networks |`Funcion_Qp_1.py`,`Funcion_Qr_1.py`, `Fcn_Qp.mat`,`Fcn_Qr.mat`,`Walk_50.mat` |

## Requirements for matlab scripts
* Matlab 2018r or  later.
* No additional toolboxes are required.

## Requirements for python scripts
* Raspberry pi 4B (Only to measure NN performance)
* Python 3
*  numpy
*  scipy.io
*  time
*  statistics
