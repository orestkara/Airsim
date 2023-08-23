# Airsim
The C++ codes contained in this repository were written for the purpose of my thesis. The codes contain an alogorithm 
implementation for A* for AirSim. The code does not take into account various hurdles. There is a suggested implementation within.

The other code contains a suggested implementation for a RRT algorithm for AirSim. That code contains various mechanisms 
for hurdle avoidance. However, it's only a suggested draft and thus wan never finished. The suggested implementations have been
commented in the code.

In order to run these scripts, the following steps must be taken.

1) Unreal, Visual Studio 2022 and AirSim must be installed on your computer based on the documentation. https://microsoft.github.io/AirSim/
2) You need to make an empty project in visual studio or implement the codes in your project. 
3) Add the references of AirLib & MavLinkCom libraries by containing their .vcxproj files to your project.
4) Adjust the dependencies, so that it is similar to those of the AirSim library, as follows:
   $(ProjectDir)..\AirLib\deps\rpclib\include;
   include;$(ProjectDir)..\AirLib\deps\eigen3;
   $(ProjectDir)..\AirLib\include

   $(ProjectDir)\..\AirLib\deps\MavLinkCom\lib\$(Platform)\$(Configuration);
   $(ProjectDir)\..\AirLib\deps\rpclib\lib\$(Platform)\$(Configuration);
   $(ProjectDir)\..\AirLib\lib\$(Platform)\$(Configuration)
5) Adjust the values to fit your scenario.
6) Build the projetc.
7) Press play in the Unreal Environment and execute the code.

