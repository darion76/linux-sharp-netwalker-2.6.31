#!/bin/bash

ls patches/*.patch | while read i; do
	echo "--- applying $i"
	patch -p1 --no-backup-if-mismatch < "$i"
	echo
done

if [ -f patches/linux-2.6.31-ER1-efikamx.config ]; then
	cp -f patches/linux-2.6.31-ER1-efikamx.config .config
fi
