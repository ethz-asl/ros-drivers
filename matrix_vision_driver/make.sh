#!/bin/bash

PACKAGE_DIR=$(rospack find matrix_vision_driver)
echo "### downloading and unpacking drivers to" $PACKAGE_DIR "###"


BLUECOUGAR_URL="http://www.matrix-vision.com/GigE-Vision-camera-mvbluecougar-x.html?file=tl_files/mv11/support/mvIMPACT_Acquire/01"
BLUEFOX_URL="http://www.matrix-vision.com/USB2.0-single-board-camera-mvbluefox-mlc.html?file=tl_files/mv11/support/mvIMPACT_Acquire/01"

BLUEFOX_FILE=""
BLUECOUGAR_FILE=""

API=mvIMPACT_acquire
VERSION=2.0.5
BLUEFOX_VERSION=2.0.5
ABI=ABI2

TARGET=$(uname -m)
if [ "$TARGET" == "i686" ]; then
    TARGET=x86
else
    TARGET=x86_64
fi

BLUECOUGAR_NAME=mvBlueCOUGAR
BLUECOUGAR_TARNAME=$BLUECOUGAR_NAME-$TARGET"_"$ABI-$VERSION

BLUEFOX_NAME=mvBlueFOX
BLUEFOX_TARNAME=$BLUEFOX_NAME-$TARGET"_"$ABI-$BLUEFOX_VERSION


# cleanup first
rm -rf $BLUECOUGAR_NAME* $BLUEFOX_NAME* $API* tmp


#### download driver archives ####
mkdir -p download
cd download
wget -O $BLUECOUGAR_TARNAME.tgz -nc $BLUECOUGAR_URL/$BLUECOUGAR_TARNAME.tgz 
wget -O $BLUEFOX_TARNAME.tgz -nc $BLUEFOX_URL/$BLUEFOX_TARNAME.tgz


#### unpack blueCougar ####

mkdir -p $PACKAGE_DIR/tmp/$BLUECOUGAR_NAME
cd $PACKAGE_DIR/tmp/$BLUECOUGAR_NAME
tar -xf $PACKAGE_DIR/download/$BLUECOUGAR_TARNAME.tgz --overwrite


#### BlueCougar runtime stuff ####

cd $PACKAGE_DIR
mkdir -p $BLUECOUGAR_NAME"_runtime"
cd $BLUECOUGAR_NAME"_runtime"
tar xf $PACKAGE_DIR/tmp/$BLUECOUGAR_NAME/$BLUECOUGAR_NAME"_"runtime-$VERSION.tar --overwrite

cd lib
# lick out devicemanager and prophandling, since this is not device specific
#rm -f libmvDeviceManager* 
#rm -f libMvPropHandling*
# create missing symlinks
ln -fs libmvBlueCOUGAR.so.$VERSION libmvBlueCOUGAR.so
ln -fs libmvTLIClientGigE.so.$VERSION libmvTLIClientGigE.so
ln -fs libmvTLIClientGigE.so mvTLIClientGigE.cti


#### mvImpact (device independent stuff) ####
cd $PACKAGE_DIR
tar xf $PACKAGE_DIR/tmp/$BLUECOUGAR_NAME/$API-$VERSION.tar --overwrite
mv $API-$VERSION $API
mv $API/lib/$TARGET/* $API/lib/
rm -rf $API/lib/$TARGET

#### GenICam runtime/compile time stuff ####

cd $PACKAGE_DIR/$BLUECOUGAR_NAME"_runtime"
mkdir -p GenICam
cd GenICam
# the runtime tar contains either the i86 or the x64 tgz
if [ -r ../GenICam_Runtime_gcc40_Linux32_i86_v*.tgz ]; then
   tar xfz ../GenICam_Runtime_gcc40_Linux32_i86_v*.tgz;
   # move platform specific suffix away
   mv bin/Linux32_i86 bin/Linux
   if [ x$TARGET != xx86 ]; then
      echo 'Platform conflict : GenICam runtime is 32bit, but target is 64bit'
   fi
fi
if [ -r ../GenICam_Runtime_gcc40_Linux64_x64_v*.tgz ]; then
   tar xfz ../GenICam_Runtime_gcc40_Linux64_x64_v*.tgz;
   # move platform specific suffix away
   mv bin/Linux64_x64 bin/Linux
   if [ x$TARGET = xx86 ]; then
      echo 'Platform conflict : GenICam runtime is 64bit, but target is 32bit'
   fi
fi

# move platform specific suffix away
#mv bin/Linux* bin/Linux


#### bluefox runtime ####
# unpack
mkdir -p $PACKAGE_DIR/tmp/$BLUEFOX_NAME
cd $PACKAGE_DIR/tmp/$BLUEFOX_NAME
tar -xf $PACKAGE_DIR/download/$BLUEFOX_TARNAME.tgz --overwrite

# copy blueFOX runtime libs
mkdir -p $PACKAGE_DIR/$BLUEFOX_NAME"_runtime/lib"
cp $PACKAGE_DIR/tmp/$BLUEFOX_NAME/$API-$TARGET-$BLUEFOX_VERSION/lib/$TARGET/libmvBlueFOX.* $PACKAGE_DIR/$BLUEFOX_NAME"_runtime/lib"

# copy udev rule
mkdir -p $PACKAGE_DIR/$BLUEFOX_NAME"_scripts"
cp $PACKAGE_DIR/tmp/$BLUEFOX_NAME/$API-$TARGET-$BLUEFOX_VERSION/Scripts/51-mvbf.rules $PACKAGE_DIR/$BLUEFOX_NAME"_scripts"

#### clean up ####
rm -rf $PACKAGE_DIR/tmp


#echo $GENVER
