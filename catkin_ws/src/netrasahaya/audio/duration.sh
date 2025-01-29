#!/usr/bin/env bash

for file in *.wav
do
  duration=$(ffprobe "$file" 2>&1 | awk '/Duration/ { print $2 }')
  echo -e "$duration\t$file"
done | sort -n
