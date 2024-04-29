# Setup 

cmake -B build -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Packages

libboost-dev-all protobuf-compiler

# Notes

WPILib uses a version of Eigen from https://github.com/wpilibsuite/allwpilib/blob/main/upstream_utils/update_eigen.py#L100 SHA is 96880810295b65d77057f4a7fb83a99a590122ad
