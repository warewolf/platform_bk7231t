#!/bin/sh
APP_BIN_NAME=$1
APP_VERSION=$2
TARGET_PLATFORM=$3
USER_CMD=$4

echo APP_BIN_NAME=$APP_BIN_NAME
echo APP_VERSION=$APP_VERSION
echo TARGET_PLATFORM=$TARGET_PLATFORM
echo USER_CMD=$USER_CMD

USER_SW_VER=`echo $APP_VERSION | cut -d'-' -f1`

echo "Start Compile"
set -e
APP_PATH=../../../apps

for i in `find ${APP_PATH}/$APP_BIN_NAME/src -type d`
do
#    echo $i
    rm -rf $i/*.o
done

for i in `find ../tuya_common -type d`
do
#    echo $i
    rm -rf $i/*.o
done

for i in `find ../../../components -type d`
do
#    echo $i
    rm -rf $i/*.o
done

if [ -z $CI_PACKAGE_PATH ]; then
    echo "not is ci build"
else
	make APP_BIN_NAME=$APP_BIN_NAME USER_SW_VER=$USER_SW_VER APP_VERSION=$APP_VERSION clean -C ./
fi

make APP_BIN_NAME=$APP_BIN_NAME USER_SW_VER=$USER_SW_VER APP_VERSION=$APP_VERSION $USER_CMD -C ./

echo "Start Combined"
cp ${APP_PATH}/$APP_BIN_NAME/output/$APP_VERSION/${APP_BIN_NAME}_${APP_VERSION}.bin tools/generate/

cd tools/generate/
./otafix ${APP_BIN_NAME}_${APP_VERSION}.bin &
./encrypt ${APP_BIN_NAME}_${APP_VERSION}.bin 510fb093 a3cbeadc 5993a17e c7adeb03 10000
python mpytools.py ${APP_BIN_NAME}_${APP_VERSION}_enc.bin

./beken_packager config.json


echo "End Combined"
cp all_1.00.bin ${APP_BIN_NAME}_QIO_${APP_VERSION}.bin
rm all_1.00.bin

cp ${APP_BIN_NAME}_${APP_VERSION}_enc_uart_1.00.bin ${APP_BIN_NAME}_UA_${APP_VERSION}.bin
rm ${APP_BIN_NAME}_${APP_VERSION}_enc_uart_1.00.bin

#generate ota file
echo "generate ota file"
./rt_ota_packaging_tool_cli -f ${APP_BIN_NAME}_${APP_VERSION}.bin -v $CURRENT_TIME -o ${APP_BIN_NAME}_${APP_VERSION}.rbl -p app -c gzip -s aes -k 0123456789ABCDEF0123456789ABCDEF -i 0123456789ABCDEF
./package.bin ${APP_BIN_NAME}_${APP_VERSION}.rbl ${APP_BIN_NAME}_UG_${APP_VERSION}.bin $APP_VERSION 
rm ${APP_BIN_NAME}_${APP_VERSION}.rbl
rm ${APP_BIN_NAME}_${APP_VERSION}.bin
rm ${APP_BIN_NAME}_${APP_VERSION}.cpr
rm ${APP_BIN_NAME}_${APP_VERSION}.out
rm ${APP_BIN_NAME}_${APP_VERSION}_enc.bin

echo "ug_file size:"
ls -l ${APP_BIN_NAME}_UG_${APP_VERSION}.bin | awk '{print $5}'
if [ `ls -l ${APP_BIN_NAME}_UG_${APP_VERSION}.bin | awk '{print $5}'` -gt 679936 ];then
	echo "**********************${APP_BIN_NAME}_$APP_VERSION.bin***************"
	echo "************************** too large ********************************"
	rm ${APP_BIN_NAME}_UG_${APP_VERSION}.bin
	rm ${APP_BIN_NAME}_UA_${APP_VERSION}.bin
	rm ${APP_BIN_NAME}_QIO_${APP_VERSION}.bin
	exit 1
fi

echo "$(pwd)"
cp ${APP_BIN_NAME}_UG_${APP_VERSION}.bin ../../${APP_PATH}/$APP_BIN_NAME/output/$APP_VERSION/${APP_BIN_NAME}_UG_${APP_VERSION}.bin
cp ${APP_BIN_NAME}_UA_${APP_VERSION}.bin ../../${APP_PATH}/$APP_BIN_NAME/output/$APP_VERSION/${APP_BIN_NAME}_UA_${APP_VERSION}.bin
cp ${APP_BIN_NAME}_QIO_${APP_VERSION}.bin ../../${APP_PATH}/$APP_BIN_NAME/output/$APP_VERSION/${APP_BIN_NAME}_QIO_${APP_VERSION}.bin

echo "*************************************************************************"
echo "*************************************************************************"
echo "*************************************************************************"
echo "*********************${APP_BIN_NAME}_$APP_VERSION.bin********************"
echo "*************************************************************************"
echo "**********************COMPILE SUCCESS************************************"
echo "*************************************************************************"

FW_NAME=$APP_NAME
if [ -n $CI_IDENTIFIER ]; then
        FW_NAME=$CI_IDENTIFIER
fi

if [ -z $CI_PACKAGE_PATH ]; then
    echo "not is ci build"
	exit
else
	mkdir -p ${CI_PACKAGE_PATH}

   cp ../../${APP_PATH}/$APP_BIN_NAME/output/$APP_VERSION/${APP_BIN_NAME}_UG_${APP_VERSION}.bin ${CI_PACKAGE_PATH}/$FW_NAME"_UG_"$APP_VERSION.bin
   cp ../../${APP_PATH}/$APP_BIN_NAME/output/$APP_VERSION/${APP_BIN_NAME}_UA_${APP_VERSION}.bin ${CI_PACKAGE_PATH}/$FW_NAME"_UA_"$APP_VERSION.bin
   cp ../../${APP_PATH}/$APP_BIN_NAME/output/$APP_VERSION/${APP_BIN_NAME}_QIO_${APP_VERSION}.bin ${CI_PACKAGE_PATH}/$FW_NAME"_QIO_"$APP_VERSION.bin
fi
