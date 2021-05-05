#!/bin/zsh

cd ../lib7842
make template
prosv5 c f lib7842@*.zip
cd ../7842F-Tower-Code
prosv5 c u lib7842 --force-apply
