DETECTIONS_DATAROOT="/home/yellow/self-driving-cars-course/final/argoverse_detections_2020" # replace with your own path
POSE_DIR="/home/yellow/self-driving-cars-course/final/argoverse-tracking/val" # should be either val or test set directory
SPLIT="val" # should be either 'val' or 'test'
TRACK_DUMP_DIR='my_result_files'

MAX_AGE=14
MIN_HITS=5
MIN_CONF=0.3

echo "DETECTIONS_DATAROOT=" $DETECTIONS_DATAROOT
echo "POSE_DIR=" $POSE_DIR
echo "SPLIT=" $SPLIT
echo "MAX_AGE=" $MAX_AGE
echo "MIN_HITS=" $MIN_HITS
echo "TRACK_DUMP_DIR=" $TRACK_DUMP_DIR
echo "MIN_CONF=" $MIN_CONF
echo "----------------Start----------------"
python3 run_ab3dmot_GMF.py --dets_dataroot $DETECTIONS_DATAROOT --pose_dir $POSE_DIR --split $SPLIT --max_age $MAX_AGE --min_hits $MIN_HITS --tracks_dump_dir $TRACK_DUMP_DIR --min_conf $MIN_CONF



# echo "-----generate submission file-----"
# zip -r resulllllttttttttttttt.zip /home/yellow/self-driving-cars-course/final/argoverse_cbgs_kf_tracker/my_result_files/test-split-track-preds-maxage15-minhits5-conf0.3/
# echo "-----Finish-----"
