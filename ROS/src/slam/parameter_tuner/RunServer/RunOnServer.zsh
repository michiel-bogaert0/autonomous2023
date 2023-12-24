#bin/zsh

sudo apt install python3-pip
pip install -r requirement.txt
roslaunch parameter_tuner tuner.launch
python3 RunEval.py




