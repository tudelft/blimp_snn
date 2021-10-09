# Download desired version + extract files
wget https://github.com/Kitware/CMake/releases/download/v3.17.0/cmake-3.17.0.tar.gz
tar -zxvf cmake-3.17.0.tar.gz
# Install OpenSSL package
sudo apt-get install libssl-dev
# Create swapfile before any compilation
sudo dd if=/dev/zero of=/swapfile bs=1024 count=1M
sudo chown root.root /swapfile
sudo chmod 0600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
# Compile and install cmake
cd cmake-3.17.0
./bootstrap
make
sudo make install
