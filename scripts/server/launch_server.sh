#!/bin/bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate base
cd $( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
python run_server.py
