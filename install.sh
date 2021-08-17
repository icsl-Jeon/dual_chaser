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
#