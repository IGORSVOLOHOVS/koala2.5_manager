#---------------------------------------------------------------------
# Makefile - koala library examples
#---------------------------------------------------------------------


SRCS		= $(wildcard *.c)
OBJS		= $(patsubst %.c,%.o,${SRCS})



BUILD		= build-${TARGET_SYSTEM}

ifeq ($(DEBUG),1)
CFLAGS 		= -g -fPIC
else
CFLAGS 		= -O3 -fPIC
endif

# Librairies
LIBS		= -L../${BUILD}/lib -lkoala -lm

# Includes 
INCS	= -I../${BUILD}/include


#Examples Programs
TARGETS	= koala_example koala_test koala_auto_mode_test

#Examples Programs, requiring threads
PTHREAD_TARGETS = 

#---------------------------------------------------------------------
# Rules
#---------------------------------------------------------------------




all: 	${TARGETS} ${PTHREAD_TARGETS}



$(TARGETS): % : %.o
	@echo "Building $@"
ifeq (${DEBUG},1)
	@echo "DEBUG MODE"
else
	@echo "RELEASE MODE"
endif	
	@$(CC) -o $@ $? $(LIBS) $(CFLAGS) 
	


$(PTHREAD_TARGETS): % : %.o
	@echo "Building $@"
ifeq (${DEBUG},1)
	@echo "DEBUG MODE"
else
	@echo "RELEASE MODE"
endif	
	@$(CC) -o $@ $? -pthread $(LIBS) $(CFLAGS) 
	

clean: 
	@echo "Cleaning"
	@rm -f *.o *~ ${TARGETS} ${PTHREAD_TARGETS} .depend core*


depend:	
	@echo "Building dependencies"
	@rm -f .depend
	@touch .depend
	@makedepend ${SYS_INCLUDES} -Y -f .depend ${SRCS}


%.o: %.c
	@echo "Compiling $@"
	@$(CC) $(INCS) -c $(CFLAGS) $< -o $@

.PHONY: all clean depend docs

ifeq (.depend,$(wildcard .depend))
include .depend 
endif

