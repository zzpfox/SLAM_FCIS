echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
# cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ..
make -j8

cd ..
BinVocFile=./Vocabulary/ORBvoc.bin
VocSize="44365017"
if [ -e $BinVocFile ]
then
    VOCFILESIZE=$(stat -c%s "$BinVocFile")
    if [ $VOCFILESIZE -eq $VocSize ]
    then 
        echo "Binary Vocabulary already exists"
    else
        echo "Recreating vocabulary to binary"
        ./tools/bin_vocabulary
    fi
else
    echo "Converting vocabulary to binary"
    ./tools/bin_vocabulary
fi
