#!/bin/bash

process=`ps`
for i in `ps | grep -v bash | grep -v ps | grep -v awk | grep -v my_kill_all.bash | awk '{print $1}' | tail -n +2`
do
  echo "Killing process $i"
  kill -9 $i
done