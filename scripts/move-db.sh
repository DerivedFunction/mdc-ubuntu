#!/bin/bash
if [[ "$1" == "m" ]]; then
    rm -rf ../temp_rtabmap_data
    mkdir ../temp_rtabmap_data
    mv rtabmap_data/*.db ../temp_rtabmap_data
fi
if [[ "$1" == "r" ]]; then
    mv ../temp_rtabmap_data/*.db rtabmap_data/
fi