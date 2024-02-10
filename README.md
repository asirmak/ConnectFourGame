# ConnectFourGame 
A project which plays Connect4 using machine vision, algorithm and Ned2 robot

To run the project you need to install the following dependencies:

- pyniryo2
- pyniryo
- argparse
- python-opencv
- connect4game (project itself, included inside the requirements.txt)

Important Note: pyniryo2 dependency fails to install when run through pip on modern version of python.
To install it manually install cmake and scikit-build then run install pyniryo2 using:
pip install pyniryo2 --no-build-isolation
