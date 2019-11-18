#!/bin/bash

#cd ~/Desktop/RoboticsM

@ECHO off
git config user.name "Jisha-George"
git config user.email "jishathakidiyel@yahoo.com"

git config --global user.name "Jisha-George"
git config --global user.email "jishathakidiyel@yahoo.com"

git add .

git commit -am "Automatic Commit"

git push origin

git config user.name unset
git config user.email unset
git config --global user.name unset
git config --global user.email unset

#cd ~/Desktop/HackRF

#@ECHO off
#git config user.name "Jisha-George"
#git config user.email "jishathakidiyel@yahoo.com"

#git config --global user.name "Jisha-George"
#git config --global user.email "jishathakidiyel@yahoo.com"

#git add .

#git commit -am "Automatic Commit"

#git push origin

#git config user.name unset
#git config user.email unset
#git config --global user.name unset
#git config --global user.email unset

#cd ..

#rm -r -f HackRF

#rm -r -f RoboticsM
