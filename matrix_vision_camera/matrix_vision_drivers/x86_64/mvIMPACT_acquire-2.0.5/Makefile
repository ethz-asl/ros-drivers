#! /bin/make
SUB_DIRS=                \
    wxPropGrid           \
    apps                 \

#-------------------------------------------------------------------------
.PHONY:    all clean strip new build x86_64 info native install

#=========================================================================
all:
	make x86_64
	@(															\
		if [ -e Makefile.mvHYPERION ];							\
		then													\
			echo "Making mvHYPERION kernel module";				\
			$(MAKE) --silent -f Makefile.mvHYPERION;			\
			echo "=============================================================";	\
			echo "To install the mvHYPERION kernel module now make sure that you are root and type:";	\
			echo "make install";								\
			echo "=============================================================";	\
		fi														\
	)

install:
	@(															\
		if [ -e Makefile.mvHYPERION ];							\
		then													\
			echo "Installing mvHYPERION kernel module";         \
			$(MAKE) --silent -f Makefile.mvHYPERION install;   \
						echo "*************************************************************";          \
						echo "after successfully installation start the kernelmodules with";           \
						echo "modprobe hyperion";                                                      \
						echo "modprobe hyperion2";                                                     \
						echo "please also read the README file for further information about loading the kernelmodules automatically";\
						echo "*************************************************************";          \
		fi                                                      \
	)

clean strip new build x86_64 info native:
	@for sdir in $(SUB_DIRS) ;                                         \
	do                                                                  \
		echo $$sdir;                                                  \
		(                                                               \
			DRV_DIR=$$sdir;                                           \
			if [ -d $$DRV_DIR ];                                      \
			then                                                        \
				if [ -e $$DRV_DIR/Makefile ];                         \
				then                                                    \
					echo "make in :"$$DRV_DIR;                        \
					$(MAKE) --silent -C $$DRV_DIR $@ || exit $$?; \
				fi;                                                     \
			fi                                                          \
		)                                                               \
	done
#-------------------------------------------------------------------------
