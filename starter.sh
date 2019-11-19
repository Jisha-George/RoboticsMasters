#!/bin/bash

wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add -

echo 'deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main' | sudo tee /etc/apt/sources.list.d/google-chrome.list
#sudo apt-get install google-chrome-stable
sudo bash Extension.sh

wget -qO - https://packagecloud.io/AtomEditor/atom/gpgkey | sudo apt-key add -

sudo sh -c 'echo "deb [arch=amd64] https://packagecloud.io/AtomEditor/atom/any/ any main" > /etc/apt/sources.list.d/atom.list'

wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt

sudo apt-get update

sudo apt-get install atom

sudo apt-get install apt-transport-https

sudo apt-get install sublime-text

sudo apt-get install ros-kinetic-uol-cmp9767m-base
sudo apt-get install ros-kinetic-uol-cmp9767m-tutorial

sudo apt-get update && sudo apt-get upgrade
#git clone https://github.com/Jisha-George/RoboticsM

#git clone https://github.com/Jisha-George/HackRF
