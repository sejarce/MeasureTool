## OpenPose配置指南

#### 3rdparty依赖库

- Caffe

- Boost

- OpenCV

- CUDA (需自己安装)

- CUDNN (需自己安装至CUDA的包含目录和库目录下)

  ​

  ​

#### Include头文件包含

*$(OPENPOSE)*\include;

*$(OPENPOSE)*\3rdparty\windows\opencv\include;

*$(OPENPOSE)*\3rdparty\windows\caffe\include;

*$(OPENPOSE)*\3rdparty\windows\caffe\include2;

*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\include;

*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\include\boost-1_61;_

_$(CUDA_PATH_V8_0)\include;



#### Dependencies附加依赖项

*$(OPENPOSE)*\3rdparty\windows\caffe\lib\caffe.lib
*$(OPENPOSE)*\3rdparty\windows\caffe\lib\caffeproto.lib
*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\lib\boost_filesystem-vc140-mt-1_61.lib
*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\lib\boost_system-vc140-mt-1_61.lib
*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\lib\caffehdf5.lib
*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\lib\caffehdf5_hl.lib
*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\lib\caffezlib.lib
*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\lib\gflags.lib
*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\lib\glog.lib
*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\lib\leveldb.lib
*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\lib\libopenblas.dll.a
*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\lib\libprotobuf.lib
*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\lib\lmdb.lib
*$(OPENPOSE)*\3rdparty\windows\caffe3rdparty\lib\snappy.lib
*$(OPENPOSE)*\3rdparty\windows\opencv\x64\vc14\lib\opencv_world310.lib

*$(OPENPOSE)*\windows\x64\Release\OpenPose.lib

_$(CUDA_PATH_V8_0)\lib\x64\cudnn.lib;



#### Configure Notices

1. 在项目属性--->C/C++--->预处理器--->定义处加上宏定义USE_CAFFE
2. 一定要将OpenPose相关的头文件包含放在最前面一排，否则会报错



