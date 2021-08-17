sudo apt-get install git
sudo apt-get install libeigen-dev
mkdir -p ~/chaser_ws/src
cd ~/chaser_ws/src
# qpOASES
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build && cd build
cmake .. -DCMAKE_CXX_FLAGS=-fPIC
sudo make install
# trajgen
cd ~/chaser_ws/src
git clone https://github.com/icsl-Jeon/traj_gen.git
cd ./traj_gen/cpp
mkdir build && cd build
cmake ..
make && sudo make install
#