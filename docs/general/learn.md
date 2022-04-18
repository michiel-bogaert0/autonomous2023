# General

Here you can learn about useful tips and tricks while programming.

Worth the research:
* ROS
* Docker

Nice software:
* [WSL2](https://docs.microsoft.com/en-us/windows/wsl/install-win10) - light virtual machine capable of using host GPU natively (handy for CUDA)
* [WSLg](https://github.com/microsoft/wslg) - WSL2 with native GUI support (only on Windows 11)
* VS Code - light and versatile IDE, useful when working with Docker
* Pycharm - very nice Python IDE with good Git and Conda support
* [Oh My Zsh](https://ohmyz.sh/) - very nice Zsh manager, Zsh is a kind of shell (like bash) but prettier and more useful
  * [Powerlevel10k](https://github.com/romkatv/powerlevel10k) - a very fast and customizable Zsh theme 
* Terminator - a very customisable terminal, very handy when you feel the need to split terminal windows
* [Just](https://github.com/casey/just) - handy way to run commands, similar to make

Tips and tricks:
* Ubuntu has a .bashrc/.zshrc file which is very useful when you want commands to run when you start a shell
  * Useful for: sourcing certain files (ROS setup), exporting variables ($PATH) and aliases
* You can create symbolic links in Ubuntu. I use this for linking the UGentRacing folder containing all my repos inside of WSL2: `cd ~ && ln -s /mnt/d/whatever/path/to/UGentRacing UGentRacing` (edit, I no longer do this. please dualboot xp)
* You can press '.' when browsing a Github repo to open it up in an online VSCode window. This is very handy when trying to understand how a repo works zithout having to download it.
* When using the simulator, think about having a separate terminal with the roscore and visualisation running. This way you don't have to restart these when you restart the sim/AS.


## Basic setup

### Install a fresh shell

First we install some basic tools you will definitely need at some point or another:
```
sudo apt install -y zsh curl git build-essential vim
```

Next we'll install Oh My Zsh and the Powerlevel10k theme:

Firstly, install the [recommended font](https://github.com/romkatv/powerlevel10k#meslo-nerd-font-patched-for-powerlevel10k). If you're using WSL, you can change the font used in the settings, located on the top bar of the terminal window. On Ubuntu you can right click and click properties or something like that.
```
sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
# Press enter when asked to change your default shell to zsh. You can always change shell using the chsh command.

git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="powerlevel10k\/powerlevel10k"/' ~/.zshrc
source ~/.zshrc
```

After following the installation prompt, Powerlevel10k should be working. Now let's install some handy plugins:
```
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
sed -Ei 's/plugins=(.+)/plugins=(git sudo zsh-autosuggestions)/' ~/.zshrc
source ~/.zshrc
```
The autosuggestions plugin gives suggestions while typing to make your life a lot easier. The sudo plugin is used when you ran a command and forgot to put sudo in from of it. Instead of retyping the command or using `sudo !!`, now you can just hit escape twice and it retypes the command for you.
