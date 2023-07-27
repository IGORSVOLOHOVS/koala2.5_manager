#!/bin/bash

# Set the variables
TARGET_SYSTEM="x86"
LIBNAME="libkoala"
LIBVER="1.0"

echo "Setting up the build environment..."
echo "Target system: $TARGET_SYSTEM"
echo "Library name: $LIBNAME"
echo "Library version: $LIBVER"

echo "Step 1: Navigating to the source directory (src)..."
cd src

echo "Step 2: Cleaning the build artifacts..."
sudo make clean -w TARGET_SYSTEM="$TARGET_SYSTEM" LIBNAME="$LIBNAME" LIBVER="$LIBVER"

echo "Build artifacts cleaned!"

echo "Step 3: Building the project..."
sudo make -w TARGET_SYSTEM="$TARGET_SYSTEM" LIBNAME="$LIBNAME" LIBVER="$LIBVER"

echo "Step 4: Installing the library..."
sudo make -w install TARGET_SYSTEM="$TARGET_SYSTEM" LIBNAME="$LIBNAME" LIBVER="$LIBVER"

echo "Library installed successfully!"

echo "Step 5: Navigating back to the main directory..."
cd ..

echo "Done! The build and installation process is complete."
