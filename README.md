# UAV-Aided Vehicular Networks: A Trajectory Optimization Approach for 5G Communication on Highways

IEEE Latin America Transactions  

Manuscript ID: [to be assigned]

**Authors**
- Ignacio Vidal  
- Sandy Bolufé  
- Karel Toledo  

## Scripts

This repository contains all the scripts required to reproduce the simulations and numerical results presented in the article.

| Script / Files | Description | Related figures, tables or algorithm |
| --- | --- | --- |
| `UAVs_VANET.m` | Simulates the selected UAV-aided vehicular network scenario, different from the first tracking case | Algorithm 1 |
| `highway_bremen.view.xml`, `osm.net.xml.gz`, `osm.poly.xml.gz` | SUMO files that define the physical layout of the highway scenario | Fig. 2 |
| `Tracking.m` | Simulates the first tracking case with a single vehicle | Figs. 3, 4, 5 |
| `highway_bremen.rou_tracking.xml`, `highway_bremen_tracking.sumocfg` | Route and scenario files for the tracking case with a single vehicle | Figs. 3, 4, 5 |
| `highway_bremen.rou_1dir_low.xml`, `highway_bremen_1dir_low.sumocfg` | Route and scenario files for the one-way traffic case with low vehicular density | Fig. 6 |
| `highway_bremen.rou_1dir_high.xml`, `highway_bremen_1dir_high.sumocfg` | Route and scenario files for the one-way traffic case with high vehicular density | Fig. 7 |
| `highway_bremen.rou_2dir_low.xml`, `highway_bremen_2dir_low.sumocfg` | Route and scenario files for the two-way traffic case with low vehicular density | Fig. 8 |
| `highway_bremen.rou_2dir_high.xml`, `highway_bremen_2dir_high.sumocfg` | Route and scenario files for the two-way traffic case with high vehicular density | Fig. 9 |
| `HARQEntity.m` | Manages the hybrid ARQ processes for a single UE | N/A |
| `hArrayGeometry.m` | Defines the base station and UE antenna array geometry used in the CDL channel model | N/A |
| `hSVDPrecoders.m` | Computes SVD-based precoding matrices for the MIMO channel | N/A |
| `hSkipWeakTimingOffset.m` | Skips weak timing offset estimates based on the correlation magnitude | N/A |
| `hSubbandChannelEstimate.m` | Performs subband channel estimation using reference signals for the OFDM-based link | N/A |

## Requirements for MATLAB scripts

- MATLAB R2024a or later  
- 5G Toolbox  
- Communications Toolbox  
- Statistics and Machine Learning Toolbox  
- Parallel Computing Toolbox  

## Additional software

- SUMO (Simulation of Urban MObility)  
- TraCI interface to connect SUMO and MATLAB  
