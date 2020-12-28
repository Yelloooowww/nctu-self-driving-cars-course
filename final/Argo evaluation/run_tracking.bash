# RESULT_DIR="/home/yellow/self-driving-cars-course/final/argoverse_cbgs_kf_tracker/my_result_files/val-split-track-preds-maxage14-minhits5-conf0.3"
RESULT_DIR="/home/yellow/self-driving-cars-course/final/argoverse_cbgs_kf_tracker/results/results_tracking_val_cbgs"
DATA_DIR="/home/yellow/self-driving-cars-course/final/argoverse-tracking/val"

python3 eval_tracking.py \
--path_tracker_output=$RESULT_DIR \
--path_dataset=$DATA_DIR
