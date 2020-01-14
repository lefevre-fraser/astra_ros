#!/bin/bash
if [[ -f "../.gitignore" ]]; then
    rm "../.gitignore"
fi

# Concatenate all the ignore files into a master ignore file
for f in ./*.gitignore;
do
    cat "$f" >> "../.gitignore"
    printf "\n\n" >> "../.gitignore"
done