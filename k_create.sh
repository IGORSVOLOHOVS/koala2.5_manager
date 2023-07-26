#!/bin/bash

if [ $# -ne 2 ]; then
    echo "Usage: $0 <language> <project_name>"
    exit 1
fi

language="$1"
project_name="$2"

if [ "$language" != "c" ] && [ "$language" != "cpp" ]; then
    echo "Invalid language. Use 'c' for C projects or 'cpp' for C++ projects."
    exit 1
fi

# Create project directories
mkdir -p "${project_name}/src" "${project_name}/include" "${project_name}/lib"

# Create main.c or main.cpp based on the selected language
if [ "$language" == "c" ]; then
    echo -e "#include <stdio.h>\n\nint main() {\n    printf(\"Hello, world!\");\n    return 0;\n}" > "${project_name}/src/main.c"
    echo -e "/* src/${project_name}.c */\n#include \"../include/${project_name}.h\"\n\n/* Implementation of ${project_name} functions here */" > "${project_name}/src/${project_name}.c"
else
    echo -e "#include <iostream>\n\nint main() {\n    std::cout << \"Hello, world!\" << std::endl;\n    return 0;\n}" > "${project_name}/src/main.cpp"
    echo -e "/* src/${project_name}.cpp */\n#include \"../include/${project_name}.h\"\n\n/* Implementation of ${project_name} functions here */" > "${project_name}/src/${project_name}.cpp"
fi

echo -e "/* include/${project_name}.h */\n\n#ifndef ${project_name}_H\n#define ${project_name}_H\n\n/* Declarations of ${project_name} functions here */\n\n#endif /* ${project_name}_H */" > "${project_name}/include/${project_name}.h"

# Create Makefile
if [ "$language" == "c" ]; then
    echo -e "SRCS            = \$(wildcard src/*.c)\nOBJS            = \$(patsubst src/%.c, build/%.o, \${SRCS})\n\nBUILD           = build-x86\n\nifeq (\$(DEBUG),1)\nCFLAGS          = -g -fPIC\nelse\nCFLAGS          = -O3 -fPIC\nendif\n\n# Includes\nINCS            = -I./\${BUILD}/include\n\n#Examples Programs\nTARGETS = ${project_name}\n\n#Examples Programs, requiring threads\nPTHREAD_TARGETS =\n\n#---------------------------------------------------------------------\n# Rules\n#---------------------------------------------------------------------\n\n\n\nall:    \${TARGETS} \${PTHREAD_TARGETS}\n\n\n\$(TARGETS): \$(OBJS)\n\t@echo \"Building \$@\"\nifeq (\$(DEBUG),1)\n\t@echo \"DEBUG MODE\"\nelse\n\t@echo \"RELEASE MODE\"\nendif\n\t@\$(CC) -o \$@ \$^ \$(CFLAGS)\n\n\nclean:\n\t@echo \"Cleaning\"\n\t@rm -rf build/ \${TARGETS} \${PTHREAD_TARGETS} .depend core*\n\n\ndepend:\n\t@echo \"Building dependencies\"\n\t@rm -f .depend\n\t@mkdir -p build\n\t@touch .depend\n\t@makedepend \${SYS_INCLUDES} -Y -f .depend \$(CFLAGS) \${SRCS}\n\n\nbuild/%.o: src/%.c\n\t@echo \"Compiling \$@\"\n\t@mkdir -p \$(dir \$@)\n\t@\$(CC) \$(INCS) -c \$(CFLAGS) \$< -o \$@\n\n.PHONY: all clean depend docs run\n\nifeq (.depend,\$(wildcard .depend))\ninclude .depend\nendif\n\nrun: \$(TARGETS)\n\t@echo \"Running ${project_name}\"\n\t@./\$(TARGETS)" > "${project_name}/Makefile"
else
    echo -e "SRCS            = \$(wildcard src/*.cpp)\nOBJS            = \$(patsubst src/%.cpp, build/%.o, \${SRCS})\n\nBUILD           = build-x86\n\nifeq (\$(DEBUG),1)\nCXXFLAGS        = -g -fPIC -std=c++17\nelse\nCXXFLAGS        = -O3 -fPIC -std=c++17\nendif\n\n# Compiler\nCXX             = g++\n\n#Examples Programs\nTARGETS = ${project_name}\n\n#Examples Programs, requiring threads\nPTHREAD_TARGETS =\n\n#---------------------------------------------------------------------\n# Rules\n#---------------------------------------------------------------------\n\n\n\nall:    \${TARGETS} \${PTHREAD_TARGETS}\n\n\n\$(TARGETS): \$(OBJS)\n\t@echo \"Building \$@\"\nifeq (\$(DEBUG),1)\n\t@echo \"DEBUG MODE\"\nelse\n\t@echo \"RELEASE MODE\"\nendif\n\t@\$(CXX) -o \$@ \$^ \$(CXXFLAGS)\n\n\nclean:\n\t@echo \"Cleaning\"\n\t@rm -rf build/ \${TARGETS} \${PTHREAD_TARGETS} .depend core*\n\n\ndepend:\n\t@echo \"Building dependencies\"\n\t@rm -f .depend\n\t@mkdir -p build\n\t@touch .depend\n\t@makedepend \${SYS_INCLUDES} -Y -f .depend \$(CXXFLAGS) \${SRCS}\n\n\nbuild/%.o: src/%.cpp\n\t@echo \"Compiling \$@\"\n\t@mkdir -p \$(dir \$@)\n\t@\$(CXX) \$(INCS) -c \$(CXXFLAGS) \$< -o \$@\n\n.PHONY: all clean depend docs run\n\nifeq (.depend,\$(wildcard .depend))\ninclude .depend\nendif\n\nrun: \$(TARGETS)\n\t@echo \"Running ${project_name}\"\n\t@./\$(TARGETS)" > "${project_name}/Makefile"
fi

chmod +x "${project_name}/Makefile"

# Copy files from ../build-x86 to the new project directory
cp -r "build-x86/include" "${project_name}"
cp "build-x86/lib/libkoala.so" "${project_name}/lib/libkoala.so"
cp "build-x86/lib/libkoala.a" "${project_name}/lib/libkoala.a"
cp "build-x86/lib/libkoala.so.1.0" "${project_name}/lib/libkoala.so.1.0"

echo "Project '${project_name}' created successfully!"
