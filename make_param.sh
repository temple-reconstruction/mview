#/bin/bash

rm -rf ./data/rathaus/parameter.txt
for file in ./data/rathaus/*.camera; do
	echo "${file%.camera}" >> ./data/rathaus/parameter.txt;
	cat $file | tr "\n" " " >> ./data/rathaus/parameter.txt;
done

