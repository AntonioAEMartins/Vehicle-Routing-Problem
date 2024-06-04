#!/bin/bash
#SBATCH --partition=espec
#SBATCH --nodes=4
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=2G         
#SBATCH --job-name=global_parallelization
#SBATCH --output=output/slurm-%j.out  

mpirun ./execs/global_parallelization