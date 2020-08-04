1. 由于工程中eai_gazebo 模拟仿真包使用的protobuf 版本是2.6.1 (安装ros kinetic时，默认安装的版本)
如果用户自己笔记本ubuntu系统 编译过cartographer 算法， 它默认会安装3.0.* 版本的protobuf，
此时可能会导致eai_gazebo 包编译出错

解决方法如下:重新编译安装protobuf 2.6.1
cd eai_tutorials/protobuf-2.6.1-artifacts
make clean
./configure
make 
sudo make install 

