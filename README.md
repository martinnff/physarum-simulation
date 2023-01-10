# physarum-simulation
A two-dimensional particle flow simulation based on the exploratory growth of genus physarium 


This implementation is based on the paper Characteristics of pattern formation and evolution in approximations of physarum transport networks.  https://doi.org/10.1162/artl.2010.16.2.16202 by Jeff Jones. The simulation uses two layers, one that stores the position and angle of movement of the particles and the other that contains the evolution of the trail left by these particles. The simulation proceeds in stages, each particle has three sensors (lateral and frontal) placed at a given distance and angle. It uses these sensors to evaluate the concentration of the trace in its environment and takes the direction with the highest concentration of attractant. As the particles move, they leave a trace behind them that eventually dissipates with each step.

This implementation has two variants of particles, each attracted to traces of the same type and repelled by those of the opposite type. Con estas reglas sencillas es posible obserbas la aparicion de patrones complejos como los que se muestran en las imagenes de abajo.


![Alt text](https://github.com/martinnff/physarum-simulation/blob/main/i1.jpeg "procedural landscape")
![Alt text](https://github.com/martinnff/physarum-simulation/blob/main/i2.jpeg "procedural landscape")
![Alt text](https://github.com/martinnff/physarum-simulation/blob/main/i3.jpeg "procedural landscape")
![Alt text](https://github.com/martinnff/physarum-simulation/blob/main/i4.jpeg "procedural landscape")

