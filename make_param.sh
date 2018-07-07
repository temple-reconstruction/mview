#/bin/bash

rm -rf ./data/rathaus/parameter.txt
for file in ./data/rathaus/*.camera; do
	echo -n "${file%.camera} " >> ./data/rathaus/parameter.txt;
	cat $file | tr "\n" " " >> ./data/rathaus/parameter.txt;
	echo >> ./data/rathaus/parameter.txt
done

