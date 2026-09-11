conda env create -f environment.yml

conda activate lidar-env

jupyter notebook


conda run -p ./environment python play_lidar_scenario.py
