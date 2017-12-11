./build/tools/caffe train --gpu 2 --solver='/home/map/hebei/caffe_hebei/segnet/Models/segnet_solver.prototxt' --weights='/home/map/hebei/caffe_hebei/caffe-segnet/models/VGG_ILSVRC_16_layers.caffemodel'  


./build/tools/caffe train --gpu 2 --solver='/home/map/hebei/caffe_hebei/segnet/Models/weixingtu_solver_1vs10_wuxichangzhou.prototxt' --weights='/home/map/hebei/caffe_hebei/caffe-segnet/models/VGG_ILSVRC_16_layers.caffemodel'



./build/tools/caffe train --gpu 2 --solver=/home/map/hebei/caffe_hebei/segnet/Models/solver_localization.prototxt --weights=/home/map/hebei/caffe_hebei/caffe-segnet/models/VGG_ILSVRC_16_layers.caffemodel
