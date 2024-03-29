#!/bin/bash
# @echo off

for i in $(find ./src/ -iname "*.cpp"); do clang-format -i -style=file $i || true
done

for i in $(find ./include/main/ -iname "*.hpp"); do clang-format -i -style=file $i || true
done
