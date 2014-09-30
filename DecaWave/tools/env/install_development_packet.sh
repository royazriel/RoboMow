#!/bin/sh
if (test $EUID -eq 0); then
    echo "Cannot run in root mode, please exit sudo or login to user mode"
    exit 1
fi

cd ~

echo "#########################################################################"
echo "# UBUNTU UPDATING PACKAGES ...                                          #"
echo "#########################################################################"

sudo apt-get install 		\
	     build-essential 	\
	     htop 		\
	     vim-gnome 		\
	     subversion 	\
	     openssh-server 	\
	     cscope 		\
	     patch 		\
	     minicom 		\
	     nfs-kernel-server 	\
	     exuberant-ctags 	\
	     ncurses-dev 	\
	     atftp 		\
	     git 		\
	     libc6:i386 	\
	     libgcc1:i386 	\
	     gcc-4.6-base:i386
         libc6-i386 \
         lib32z1 \
         lib32stdc++6 \
         --force-yes 	\
	     -y

sudo apt-get install 		\
	     apt-file 		\
	     -y
sudo apt-file update



echo "##########################################################################"
echo "# UBUNTU UPDATING PACKAGES FINISHED                                      #"
echo "##########################################################################"

if [ -d ~/.vim_runtime ]; then
    echo "######################################################################"
    echo "# VIM CONFIGURATION EXISTS. IF NEEDS TO UPDATE, REMOVE ~/.vim_runtime#"
    echo "######################################################################"
else
	cd ~/.vim_runtime
	git clone git://github.com/amix/vimrc.git ~/.vim_runtime
	sh ~/.vim_runtime/install_awesome_vimrc.sh
fi

echo "##########################################################################"
echo "# INSTALLING SOURCERY CODE                                               #"
echo "##########################################################################"

if [ -d /opt/rpi/tools ]; then
    echo "######################################################################"
    echo "# SOURCERY CODE EXISTS.                                              #"
    echo "######################################################################"
else
    sudo mkdir -p  /opt/rpi
    sudo chmod 755  /opt/rpi
    sudo chown roy:roy /opt/rpi
    cd /opt/rpi
    git clone git://github.com/raspberrypi/tools.git
fi
