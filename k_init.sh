#!/bin/bash

# Set the variables
TARGET_SYSTEM="x86"
LIBNAME="libkoala"
LIBVER="1.0"
KOALA_LANGUAGE_CPP="g++" 
KOALA_LANGUAGE_C="gcc"
KOALA_LANGUAGE_PYTHON="python"
KOALA_FLAGS="-lm"


echo "Setting up the build environment..."
echo "Target system: $TARGET_SYSTEM"
echo "Library name: $LIBNAME"
echo "Library version: $LIBVER"


echo "Step 1: Navigating to the source directory (src)..."
cd src

echo "Step 2: Uninstall old libraries and includes from /usr/lib and /usr/include"
sudo make uninstall -w TARGET_SYSTEM="$TARGET_SYSTEM" LIBNAME="$LIBNAME" LIBVER="$LIBVER" KOALA_LANGUAGE="$KOALA_LANGUAGE_C"
sudo make uninstall -w TARGET_SYSTEM="$TARGET_SYSTEM" LIBNAME="$LIBNAME" LIBVER="$LIBVER" KOALA_LANGUAGE="$KOALA_LANGUAGE_CPP"

echo "Step 3: Clean and build the C and C++ and Python libraries and includes..."
sudo make clean -w TARGET_SYSTEM="$TARGET_SYSTEM" LIBNAME="$LIBNAME" LIBVER="$LIBVER" KOALA_LANGUAGE="$KOALA_LANGUAGE_C"
sudo make -w TARGET_SYSTEM="$TARGET_SYSTEM" LIBNAME="$LIBNAME" LIBVER="$LIBVER" KOALA_LANGUAGE="$KOALA_LANGUAGE_C"

sudo make python -w TARGET_SYSTEM="$TARGET_SYSTEM" LIBNAME=koala LIBVER="$LIBVER" KOALA_LANGUAGE="$KOALA_LANGUAGE_PYTHON"

sudo make clean -w TARGET_SYSTEM="$TARGET_SYSTEM" LIBNAME="$LIBNAME" LIBVER="$LIBVER" KOALA_LANGUAGE="$KOALA_LANGUAGE_CPP"
sudo make -w TARGET_SYSTEM="$TARGET_SYSTEM" LIBNAME="$LIBNAME" LIBVER="$LIBVER" KOALA_LANGUAGE="$KOALA_LANGUAGE_CPP"

echo "Step 4: Do you want to install IMPORTANT: C(not C++ and not Python) libraries and includes to /usr/lib and /usr/include? (y/n)"
read -r install_choice

if [[ $install_choice == 'y' || $install_choice == 'Y' ]]; then
    echo "Installing the library..."
    sudo make -w install TARGET_SYSTEM="$TARGET_SYSTEM" LIBNAME="$LIBNAME" LIBVER="$LIBVER" KOALA_LANGUAGE="$KOALA_LANGUAGE_C"
else
    echo "Installation skipped."
fi

echo "Step 5: Navigating back to the main directory..."
cd ..

echo "Done! The build and installation process is complete."
