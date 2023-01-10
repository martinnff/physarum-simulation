# physarum-simulation
A two-dimensional particle flow simulation based on the exploratory growth of genus physarium 


This implementation is based on the paper **Characteristics of pattern formation and evolution in approximations of physarum transport networks.**  https://doi.org/10.1162/artl.2010.16.2.16202 by Jeff Jones. The simulation uses two layers, one that stores the position and angle of movement of the particles and the other that contains the evolution of the trail left by these particles. The simulation proceeds in stages, each particle has three sensors (two in the laterals and one frontal) placed at a given distance and angle. It uses these sensors to evaluate the concentration of the trace in its environment and takes the direction with the highest concentration of attractant. As the particles move, they leave a trace behind them that eventually dissipates with each step. It also has a diffusion step in which the trace intensity of each pixel is computed as the mean value of the trace in the adjacent pixels.

The implementation has two variants of particles, each attracted to traces of the same type and repelled by those of the opposite type. With these simple rules it is possible to observe the appearance of complex patterns such as those shown in the images below. 


![Alt text](https://github.com/martinnff/physarum-simulation/blob/main/i1.jpeg "im1")
![Alt text](https://github.com/martinnff/physarum-simulation/blob/main/i2.jpeg "im2")
![Alt text](https://github.com/martinnff/physarum-simulation/blob/main/i3.jpeg "im3")
![Alt text](https://github.com/martinnff/physarum-simulation/blob/main/i4.jpeg "im4")

## Usage

To run this code you need to install opencv for c++. On linux I followed this guide https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/. A copy of the eigen library is also needed, there is one present in this repository downloaded from the oficial site https://eigen.tuxfamily.org/index.php?title=Main_Page. If you use another version of eigen you need to change the CMakeLists.txt accordingly.

If you haven't installed cmake and make install them via:

```bash
sudo apt update
sudo apt install make
sudo apt install cmake
```

Open a termial on the project folder and create a new directory inside named build, then move into the build folder.

```bash
mkdir build
cd build
```
Inside the build folder run cmake and make to generate the compiling instructions and create an extecutable.

```bash
cmake ..
make
```
Then you can run the executable via:

```bash
./physarum_sim
```
