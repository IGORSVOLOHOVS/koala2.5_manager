SRCS		= $(wildcard *.c)
OBJS		= $(patsubst %.c,%.o,${SRCS})

KOALA_LANGUAGE	= gcc
KOALA_FLAGS	= -fPIC

BUILD		= ../build-${TARGET_SYSTEM}-${KOALA_LANGUAGE}
INSTALL_PATH = /usr

ifeq ($(DEBUG),1)
KOALA_FLAGS	 	+= -g 
else
KOALA_FLAGS	 	+= -O3
endif

LIBS		= -lm

#---------------------------------------------------------------------
# Rules
#---------------------------------------------------------------------
all: 	${LIBNAME}

doc: docs
docs:
	doxygen

${LIBNAME}: ${OBJS}
	@echo "Targetting koala Library for ${TARGET_SYSTEM}"
ifeq (${DEBUG},1)
	@echo "DEBUG MODE"
else
	@echo "RELEASE MODE"
endif
	@echo "Building ${LIBNAME}.so.${LIBVER}"
	@mkdir -p ${BUILD}/lib
	@mkdir -p ${BUILD}/include/koala
	$(KOALA_LANGUAGE) -o ${BUILD}/lib/${LIBNAME}.so.${LIBVER} -shared $(LIBS) ${OBJS}
	@echo "Building ${LIBNAME}.a"
	@$(AR) r ${BUILD}/lib/${LIBNAME}.a ${OBJS}
	@echo "Adding includes"
	@cp *.h ${BUILD}/include/koala
	@echo "Creating symlink ${LIBNAME}.so"
	@rm -f ${BUILD}/lib/${LIBNAME}.so
	@cd ${BUILD}/lib &&  cp -s ${LIBNAME}.so.${LIBVER} ${LIBNAME}.so

clean: 
	@echo "Cleaning"
	@rm -f *.o *~ .depend core*

install:
	@echo "Installing library and headers in the system (sudo required)"
	@echo "Copying library to /usr/lib"
	@cp ${BUILD}/lib/${LIBNAME}.so.${LIBVER} ${INSTALL_PATH}/lib
	@echo "Copying headers to /usr/include/koala"
	@mkdir -p ${INSTALL_PATH}/include/koala
	@cp *.h ${INSTALL_PATH}/include/koala
	@echo "Creating symlink /usr/lib/${LIBNAME}.so"
	@rm -f ${INSTALL_PATH}/lib/${LIBNAME}.so
	@cd ${INSTALL_PATH}/lib &&  cp -s ${LIBNAME}.so.${LIBVER} ${LIBNAME}.so

uninstall:
	@echo "Uninstalling library and headers (sudo required)"
	@rm -f ${INSTALL_PATH}/lib/${LIBNAME}.so.${LIBVER}
	@echo "${INSTALL_PATH}/lib/${LIBNAME}.so.${LIBVER} uninstalled!"
	@rm -f ${INSTALL_PATH}/lib/${LIBNAME}.so
	@echo "${INSTALL_PATH}/lib/${LIBNAME}.so uninstalled!"
	@rm -f ${INSTALL_PATH}/lib/${LIBNAME}.a
	@echo "${INSTALL_PATH}/lib/${LIBNAME}.a uninstalled!"
	@rm -rf ${INSTALL_PATH}/include/koala
	@echo "${INSTALL_PATH}/include/koala uninstalled!"

depend:	
	@echo "Building dependencies"
	@rm -f .depend
	@touch .depend
	@makedepend ${SYS_INCLUDES} -Y -f .depend ${SRCS}

%.o: %.c %.h
	@echo "Building $@"
	@$(KOALA_LANGUAGE) -c $(KOALA_FLAGS) $< -o $@

.PHONY: all clean depend docs install uninstall

ifeq (.depend,$(wildcard .depend))
include .depend 
endif
