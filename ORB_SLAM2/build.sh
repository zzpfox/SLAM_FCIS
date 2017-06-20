# ./bash.sh or ./bash.sh -c clean
echo "Configuring and building Thirdparty/DBoW2 ..."

while getopts c: option
do
 case "${option}"
 in
 c) CLEAN=${OPTARG};;
 esac
done

echo $CLEAN
if ! [ -z $CLEAN ]; then
    echo "Cleaning the built files first ..."
    cd Thirdparty/DBoW2
    echo "Removing build folder in Thirdparty/DBoW2"
    rm -rf build
    cd ../g2o
    echo "Removing build folder in Thirdparty/g2o"
    rm -rf build
    cd ../../
    echo "Removing build folder in ."
    rm -rf build
else
    echo "Build without Cleaning First ..."
fi

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



cd Vocabulary
TxtVocFile=./ORBvoc.txt
TxtVocSize="145250924"
if [ -e $TxtVocFile ]
then
    TXTVOCFILESIZE=$(stat -c%s "$TxtVocFile")
    if [ $TXTVOCFILESIZE -eq $TxtVocSize ]
    then 
        echo "Txt Vocabulary already exists"
    else
        echo "Reuncompress vocabulary ..."
        tar -xf ORBvoc.txt.tar.gz
    fi
else
    echo "Uncompress vocabulary ..."
    tar -xf ORBvoc.txt.tar.gz
fi

cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
# cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ..
make -j8

cd ..
BinVocFile=./Vocabulary/ORBvoc.bin
BinVocSize="44365017"
if [ -e $BinVocFile ]
then
    BINVOCFILESIZE=$(stat -c%s "$BinVocFile")
    if [ $BINVOCFILESIZE -eq $BinVocSize ]
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
