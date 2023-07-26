SRCS		= $(wildcard *.c)
OBJS		= $(patsubst %.c,%.o,${SRCS})



BUILD		= ../build-${TARGET_SYSTEM}
INSTALL_PATH = /usr

ifeq ($(DEBUG),1)
CFLAGS 		= -g -fPIC
else
CFLAGS 		= -O3 -fPIC
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
	$(CC) -o ${BUILD}/lib/${LIBNAME}.so.${LIBVER} -shared $(LIBS) ${OBJS}
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

depend:	
	@echo "Building dependencies"
	@rm -f .depend
	@touch .depend
	@makedepend ${SYS_INCLUDES} -Y -f .depend ${SRCS}


%.o: %.c %.h
	@echo "Building $@"
	@$(CC) -c $(CFLAGS) $< -o $@

.PHONY: all clean depend docs install

ifeq (.depend,$(wildcard .depend))
include .depend 
endif

