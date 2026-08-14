PKG=$(rospack find inesctec_mrdt_slam_2d_datasets)

# aces  (FLASER)
rosrun inesctec_mrdt_slam_2d_datasets clf2bag.py \
  $PKG/data/aces.clf        $PKG/bag/aces.clf2bag.bag \
  --range-min 0.30 --range-max 50.0

# intel  (FLASER)
rosrun inesctec_mrdt_slam_2d_datasets clf2bag.py \
  $PKG/data/intel.clf       $PKG/bag/intel.clf2bag.bag \
  --range-min 0.10 --range-max 80.0

# fr079  (FLASER, laser 0.04 m forward)
rosrun inesctec_mrdt_slam_2d_datasets clf2bag.py \
  $PKG/data/fr079.clf       $PKG/bag/fr079.clf2bag.bag \
  --range-min 0.10 --range-max 80.0 --laser-x 0.04

# mit-csail  (FLASER)
rosrun inesctec_mrdt_slam_2d_datasets clf2bag.py \
  $PKG/data/mit-csail.clf   $PKG/bag/mit-csail.clf2bag.bag \
  --range-min 0.10 --range-max 80.0

# mit-killian  (FLASER)
rosrun inesctec_mrdt_slam_2d_datasets clf2bag.py \
  $PKG/data/mit-killian.clf $PKG/bag/mit-killian.clf2bag.bag \
  --range-min 0.10 --range-max 80.0

# fr-clinic  (ROBOTLASER1 — see note)
rosrun inesctec_mrdt_slam_2d_datasets clf2bag.py \
  $PKG/data/fr-clinic_robotlaser.clf $PKG/bag/fr-clinic_robotlaser.clf2bag.bag \
  --range-min 0.10
