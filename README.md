# System.md

Allows easy calculation of characteristics, transformations, and Open-Loop to Closed-Loop additions of State Space Systems. Also allows to see Transer Functions of system (as of now, only SISO systems). MIMO systems can be used, but are currently not compatible with any transfer function methods, nor are they convertable to Observer Canonical Form or Contrller Canonical Form due to the fact that the current inv(Pccf) algorithm is only for SISO systems. See system_examples.m for specific examples of how to use the class System.
