#!/bin/bash

#### Find the script directory regardless of symlinks, etc
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
    DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
    SOURCE="$(readlink "$SOURCE")"
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
    # if $SOURCE was a relative symlink, we need to resolve it relative
    # to the path where the symlink file was located
done
THISDIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
ROOTDIR=${THISDIR%/scripts}
####

color_bold=`tput bold`
color_redf=`tput setaf 1`
color_reset=`tput sgr0`

STRICT=true
SINGLE_MODE=false
USE_JULIA=true
JULIA_0_6_MODE=false
while getopts "ijms" opt; do
    case $opt in
        i) STRICT=false ;;
        j) USE_JULIA=false ;;
        m) JULIA_0_6_MODE=true ;;
        s) SINGLE_MODE=true ;;
        \?) exit 1 ;;
    esac
done

mkdir -p $ROOTDIR/deps

PKGS=''
PIP_PKGS=''

## Dependency dependencies
PKGS+='wget '

## Waf dependencies
PKGS+='pkg-config '

## Basic C compiler dependency
PKGS+='build-essential '

## Lib ZMQ
PKGS+='libzmq3-dev '

## Java
PKGS+='default-jdk default-jre '

## Python
PKGS+='python python-pip '
PIP_PKGS+='Cython '

echo "Updating apt repos"
sudo apt-get update
ret=$?
if [[ $ret -ne 0 && "$STRICT" == "true" ]]; then
    echo "Failed to update"
    exit $ret
fi

echo "Installing from apt"
if $SINGLE_MODE; then
    for pkg in $PKGS; do
        echo "Installing $pkg"
        sudo apt-get install -yq $pkg
        ret=$?
        if [[ $ret -ne 0 && "$STRICT" == "true" ]]; then
            echo "Failed to install packages"
            exit $ret
        fi
    done
else
    sudo apt-get install -yq $PKGS
fi

sudo pip install --upgrade pip
ret=$?
if [[ $ret -ne 0 && "$STRICT" == "true" ]]; then
    echo "Failed to upgrade pip"
    exit $ret
fi

sudo pip install $PIP_PKGS
ret=$?
if [[ $ret -ne 0 && "$STRICT" == "true" ]]; then
    echo "Failed to install pip packages"
    exit $ret
fi

echo "Updating db"
sudo updatedb
ret=$?
if [[ $ret -ne 0 && "$STRICT" == "true" ]]; then
    echo "Failed to updatedb"
    exit $ret
fi

exit 0
