#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Running command: cd examples"
  cd examples

  echo "Running command: make clean"
  make clean

  echo "Running command: make -w"
  make -w

  echo "List of executable files in examples directory:"
  for file in *; do
    if [ -x "$file" ]; then
      echo "$file"
    fi
  done

  echo "Running command: cd .."
  cd ..

  echo "k_examples shell script executed successfully!"
else
  executable_file="$1"

  echo "Running command: cd examples"
  cd examples

  echo "Running command: make clean"
  make clean

  echo "Running command: make -w"
  make -w

  echo "List of executable files in examples directory:"
  for file in *; do
    if [ -x "$file" ]; then
      echo "$file"
    fi
  done

  if [ -x "$executable_file" ]; then
    echo "Running command: ./$executable_file"
    ./"$executable_file"
  else
    echo "Error: Executable file '$executable_file' not found or is not executable."
  fi

  cd ..
fi
