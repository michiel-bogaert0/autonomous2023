# Ubuntu 20.04 installation
This installation guide assumes a clean build of Ubuntu 20.04 or using WSL2.

## WSL2 installation (you can skip if you are running Ubuntu natively)

Follow the Ubuntu [tutorial](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-10#1-overview), make sure to also install Windows Terminal (it will make your life a lot easier).

To enable CUDA through WSL2, make sure you have the correct [Nvidia drivers](https://developer.nvidia.com/cuda/wsl/download).

## ROS Installation

### Install git lfs
Make sure Git LFS is enabled on your system, you can verify this by typing git lfs and the prompt should return
```
git-lfs/2.13.3 (GitHub; linux amd64; go 1.16.2)  
git lfs <command> [<args>]  
  
Git LFS is a system for managing...
```

If LFS isn't installed, run the following lines:
```
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install
```

### Install UGR repo
If you don't have your SSH keys set up: [SSH setup](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)

1. `cd ~`
2. `git clone git@github.com:UgentRacing/autonomous.git`
3. `git clone git@github.com:UgentRacing/autonomous_binaries.git`

### Install ROS

Firstly follow the ROS Noetic installation instructions for Ubuntu at [link](http://wiki.ros.org/noetic/Installation/Ubuntu), I recommend installing the desktop version. Here is a short summary:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop
source /opt/ros/noetic/setup.bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### Install dependencies

1. `cd ~/autonomous`
2. `scripts/setup_repo.sh`
3. `export PATH="'"/home/$(id -un)/.local/bin"':$PATH''"`
4. `source /opt/ros/noetic/setup.bash`

### Run ROS pipeline

1. `cd autonomous/ROS`
2. `catkin init` 
3. `catkin build` 
4. `source devel/setup.bash` (Or use your new shortcut `sdev`)
6. Done! You can start coding by typing `code .`

### Testing

If the `catkin build` succeeds without any problem, there are some more quick tests you can run to ensure your system is working smoothly.

Check the X11 forwarding by running `roscore` in one terminal and `rviz` in the other. Now check that the Rviz window pops up.
Check the CUDA drivers by running:
```
python3
import torch
torch.cuda.is_available()
exit (or ctrl+D)
```

### Possible problems
#### Torch not running on newest GPU (RTX)
Fix: `pip3 install torch==1.9.0+cu111 torchvision==0.10.0+cu111 torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html`
