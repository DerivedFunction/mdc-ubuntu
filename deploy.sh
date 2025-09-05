#!/bin/bash

# scp -Cr ../mdc-ubuntu capstone@mdc-nx.local:/home/capstone/mdc-ubuntu
rsync -avz --filter=':- .gitignore' \
    --exclude='.git' \
    --exclude='*.db' \
    ../mdc-ubuntu capstone@mdc-nx.local:/home/capstone/