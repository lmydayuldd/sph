################################################################################
## INSTALLATION & EXECUTION ####################################################
################################################################################

Run SPH.exe. Most DDLs are present. If it fails, then install attached
  vcredist_x64.exe. Everything shall run smoothly then. If you have problem
  installing vcredist_x64.exe, which is possible for example on Windows 8.1,
  then you must make sure to install overdue Windows Updates, since one of
  them is prerequisite for MSVC 2015.

################################################################################
## OPTIONS & CONTROLS ##########################################################
################################################################################

The program does not have menu at the moment. Most of options are set at
  compilation time and possible to modify at Settings.cpp. However you do have
  some possibilites of control, besides just using the mouse; these include:

    WSAD - move the camera in space,
    LCTRL + Left Mouse Button - lock and move a particle,
    P - turn off / turn on OMP Parallelization (on by default, you can see
    	the gain it brings on multicore processor),
    Space Bar - start/stop simulation,
    R - when simulation is stopped, progress by one frame only,
    ...

################################################################################
## POSSIBLE BUGS & DEBUGGING ###################################################
################################################################################

There were trials to implement some operations on GPU, there are functions
  allowing it in the code. This causes the program to search for a CUDA-enabled
  GPU. Situation where CUDA-enabled GPU is not available is handled by it.
  However a situation where you do have a CUDA-enabled GPU, but it is different
  than GeForce GTX 1060 has not been tested and may possibly cause a runtime
  error.

If you run the program on a Virtual Machine, please make sure to turn off
  Mouse Integration! Otherwise you won't be able to use mouse to direct
  the camera properly.
