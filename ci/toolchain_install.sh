#!/bin/bash

# exit when any command fails
set -e

REPOSITORY=https://github.com/vortexgpgpu/vortex-toolchain-prebuilt/raw/master

DESTDIR="${DESTDIR:=/opt}"

riscv()
{
    for x in {a..o} 
    do
        wget -c -nc $REPOSITORY/riscv-gnu-toolchain/ubuntu/bionic/riscv-gnu-toolchain.tar.bz2.parta$x
    done
    cat riscv-gnu-toolchain.tar.bz2.parta* > riscv-gnu-toolchain.tar.bz2
    tar -xvf riscv-gnu-toolchain.tar.bz2
    rm -f riscv-gnu-toolchain.tar.bz2*
    cp -r riscv-gnu-toolchain $DESTDIR
   #rm -rf riscv-gnu-toolchain
}

riscv64()
{
    for x in {a..j} 
    do
        wget -c -nc $REPOSITORY/riscv64-gnu-toolchain/ubuntu/bionic/riscv64-gnu-toolchain.tar.bz2.parta$x
    done
    cat riscv64-gnu-toolchain.tar.bz2.parta* > riscv64-gnu-toolchain.tar.bz2
    tar -xvf riscv64-gnu-toolchain.tar.bz2
    rm -f riscv64-gnu-toolchain.tar.bz2*
    cp -r riscv64-gnu-toolchain $DESTDIR
    #rm -rf riscv64-gnu-toolchain
}

llvm()
{
    for x in {a..b} 
    do
        wget -c -nc $REPOSITORY/llvm-riscv/ubuntu/bionic/llvm-riscv.tar.bz2.parta$x
    done
    cat llvm-riscv.tar.bz2.parta* > llvm-riscv.tar.bz2
    tar -xvf llvm-riscv.tar.bz2
    rm -f llvm-riscv.tar.bz2*
    cp -r llvm-riscv $DESTDIR
    #rm -rf llvm-riscv
}

pocl()
{
    wget -c -nc $REPOSITORY/pocl/ubuntu/bionic/pocl.tar.bz2
    tar -xvf pocl.tar.bz2
    rm -f pocl.tar.bz2
    cp -r pocl $DESTDIR
    #rm -rf pocl
}

verilator()
{
    wget -c -nc $REPOSITORY/verilator/ubuntu/bionic/verilator.tar.bz2
    tar -xvf verilator.tar.bz2
    rm -f verilator.tar.bz2
    cp -r verilator $DESTDIR
    #rm -rf verilator
}

usage()
{
    echo "usage: toolchain_install [[-riscv] [-riscv64] [-llvm] [-pocl] [-verilator] [-all] [-h|--help]]"
}

while [ "$1" != "" ]; do
    case $1 in
        -pocl ) pocl
                ;;
        -verilator ) verilator
                     ;;
        -riscv ) riscv
                 ;;
        -riscv64 ) riscv64
                 ;;
        -llvm ) llvm
                ;;
        -all ) riscv
               riscv64
               llvm
               pocl
               verilator
               ;;
        -h | --help ) usage
                      exit
                      ;;
        * )           usage
                      exit 1
    esac
    shift
done
