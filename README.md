
## Installation

[Instructions on setting up your Jetson Nano](https://docs.google.com/document/d/1Zwriuj1YhczBsMVh11xQKaoRo1WXXvaLQYEdnHrO4jg/edit?usp=sharing)

On your computer (separate device) Install VSCode

Install "VEX Robotics" Extension in VSCode

Installing Whooplib VEXCode (On your computer):

```bash
  cd Desktop

  git clone https://github.com/ConnorAtmos/WhoopLibVEXCode
```

Installing Whooplib Python (SSH On your Jetson Nano via "ssh jetson@ip"):

```bash
  cd ~/Desktop

  mkdir WhoopLibPython

  git clone https://github.com/ConnorAtmos/WhoopLibPython
```

Modify the whooplibpython.service if necessary, then after:

```bash
  sudo cp whooplibpython.service /etc/systemd/system/whooplibpython.service

  sudo systemctl enable whooplibpython.service 

  sudo systemctl restart whooplibpython.service
```