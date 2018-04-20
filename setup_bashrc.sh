#!/bin/bash
user=`id -u -n`

echo " " >> ~/.bashrc
echo "# virtualenv and virtualenvwrapper" >> ~/.bashrc
echo "export WORKON_HOME=$HOME/.virtualenvs" >> ~/.bashrc
echo "source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc
echo " " >> ~/.bashrc
source /$user/.bashrc
