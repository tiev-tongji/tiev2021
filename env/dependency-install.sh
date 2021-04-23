#!/bin/bash
# Create the file repository configuration:
sudo sh -c 'echo "deb http://apt.postgresql.org/pub/repos/apt $(lsb_release -cs)-pgdg main" > /etc/apt/sources.list.d/pgdg.list'

# Import the repository signing key:
wget --quiet -O - https://www.postgresql.org/media/keys/ACCC4CF8.asc | sudo apt-key add -

# Update the package lists:
sudo apt-get update

# Install the latest version of PostgreSQL.
# If you want a specific version, use 'postgresql-12' or similar instead of 'postgresql':
sudo apt install postgresql-server-dev-all -y

#install libpqxx
wget -O libpqxx-7.3.1.zip https://github.com/jtv/libpqxx/archive/7.3.1.zip
unzip libpqxx-7.3.1 && rm libpqxx-7.3.1.zip
cd libpqxx-7.3.1 && mkdir build && cd build
cmake .. && make -j8 && sudo make install
