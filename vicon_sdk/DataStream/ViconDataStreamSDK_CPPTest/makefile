# Autogenerated makefile for ViconDataStreamSDK_CPPTest

ifndef VERBOSE
.SILENT :
endif
.SUFFIXES :

ifdef CONFIG
ifneq ($(CONFIG), Debug)
ifneq ($(CONFIG), InternalRelease)
ifneq ($(CONFIG), Release)
Error: unknown configuration.
endif
endif
endif
else
CONFIG=Debug
endif

ifeq ($(CONFIG), Debug)
DEFINES=-DTCM_LINUX -DTCM_UNIX -D_DEBUG -DPROJECT_SOURCE_PATH=\".\" 
SYSTEMINCLUDEPATHS=-isystem../../../../thirdparty/Boost/boost-1.75.0-linux-x64/include 
INCLUDEPATHS=-I. -I../.. -I.. -I../ViconDataStreamSDK_CPP -I. -IDebug 
LIBRARYPATHS=-L../../../../lib/Debug -L../../../../thirdparty/Boost/boost-1.75.0-linux-x64/lib  -L../../../../bin/Debug
LIBRARIES=-lboost_atomic-mt-d-x64 -lboost_chrono-mt-d-x64 -lboost_container-mt-d-x64 -lboost_context-mt-d-x64 -lboost_contract-mt-d-x64 -lboost_coroutine-mt-d-x64 -lboost_date_time-mt-d-x64 -lboost_exception-mt-d-x64 -lboost_fiber-mt-d-x64 -lboost_filesystem-mt-d-x64 -lboost_graph-mt-d-x64 -lboost_json-mt-d-x64 -lboost_locale-mt-d-x64 -lboost_log-mt-d-x64 -lboost_log_setup-mt-d-x64 -lboost_math_c99-mt-d-x64 -lboost_math_c99f-mt-d-x64 -lboost_math_c99l-mt-d-x64 -lboost_math_tr1-mt-d-x64 -lboost_math_tr1f-mt-d-x64 -lboost_math_tr1l-mt-d-x64 -lboost_nowide-mt-d-x64 -lboost_prg_exec_monitor-mt-d-x64 -lboost_program_options-mt-d-x64 -lboost_random-mt-d-x64 -lboost_regex-mt-d-x64 -lboost_serialization-mt-d-x64 -lboost_stacktrace_addr2line-mt-d-x64 -lboost_stacktrace_backtrace-mt-d-x64 -lboost_stacktrace_basic-mt-d-x64 -lboost_stacktrace_noop-mt-d-x64 -lboost_system-mt-d-x64 -lboost_test_exec_monitor-mt-d-x64 -lboost_thread-mt-d-x64 -lboost_timer-mt-d-x64 -lboost_type_erasure-mt-d-x64 -lboost_unit_test_framework-mt-d-x64 -lboost_wave-mt-d-x64 -lboost_wserialization-mt-d-x64 
DEPENDENCIES=-lStreamCommon -lViconCGStream -lViconCGStreamClient -lViconCGStreamClientSDK -lViconDataStreamSDK_CPP -lViconDataStreamSDKCore -lViconDataStreamSDKCoreUtils 
endif
ifeq ($(CONFIG), InternalRelease)
DEFINES=-DVICON_INTERNAL_RELEASE -DNDEBUG -DTCM_LINUX -DTCM_UNIX -DPROJECT_SOURCE_PATH=\".\" 
SYSTEMINCLUDEPATHS=-isystem../../../../thirdparty/Boost/boost-1.75.0-linux-x64/include 
INCLUDEPATHS=-I. -I../.. -I.. -I../ViconDataStreamSDK_CPP -I. -IInternalRelease 
LIBRARYPATHS=-L../../../../lib/InternalRelease -L../../../../thirdparty/Boost/boost-1.75.0-linux-x64/lib  -L../../../../bin/InternalRelease
LIBRARIES=-lboost_atomic-mt-x64 -lboost_chrono-mt-x64 -lboost_container-mt-x64 -lboost_context-mt-x64 -lboost_contract-mt-x64 -lboost_coroutine-mt-x64 -lboost_date_time-mt-x64 -lboost_exception-mt-x64 -lboost_fiber-mt-x64 -lboost_filesystem-mt-x64 -lboost_graph-mt-x64 -lboost_json-mt-x64 -lboost_locale-mt-x64 -lboost_log-mt-x64 -lboost_log_setup-mt-x64 -lboost_math_c99-mt-x64 -lboost_math_c99f-mt-x64 -lboost_math_c99l-mt-x64 -lboost_math_tr1-mt-x64 -lboost_math_tr1f-mt-x64 -lboost_math_tr1l-mt-x64 -lboost_nowide-mt-x64 -lboost_prg_exec_monitor-mt-x64 -lboost_program_options-mt-x64 -lboost_random-mt-x64 -lboost_regex-mt-x64 -lboost_serialization-mt-x64 -lboost_stacktrace_addr2line-mt-x64 -lboost_stacktrace_backtrace-mt-x64 -lboost_stacktrace_basic-mt-x64 -lboost_stacktrace_noop-mt-x64 -lboost_system-mt-x64 -lboost_test_exec_monitor-mt-x64 -lboost_thread-mt-x64 -lboost_timer-mt-x64 -lboost_type_erasure-mt-x64 -lboost_unit_test_framework-mt-x64 -lboost_wave-mt-x64 -lboost_wserialization-mt-x64 
DEPENDENCIES=-lStreamCommon -lViconCGStream -lViconCGStreamClient -lViconCGStreamClientSDK -lViconDataStreamSDK_CPP -lViconDataStreamSDKCore -lViconDataStreamSDKCoreUtils 
endif
ifeq ($(CONFIG), Release)
DEFINES=-DNDEBUG -DTCM_OFF_SITE -DTCM_LINUX -DTCM_UNIX -DPROJECT_SOURCE_PATH=\".\" 
SYSTEMINCLUDEPATHS=-isystem../../../../thirdparty/Boost/boost-1.75.0-linux-x64/include 
INCLUDEPATHS=-I. -I../.. -I.. -I../ViconDataStreamSDK_CPP -I. -IRelease 
LIBRARYPATHS=-L../../../../lib/Release -L../../../../thirdparty/Boost/boost-1.75.0-linux-x64/lib  -L../../../../bin/Release
LIBRARIES=-lboost_atomic-mt-x64 -lboost_chrono-mt-x64 -lboost_container-mt-x64 -lboost_context-mt-x64 -lboost_contract-mt-x64 -lboost_coroutine-mt-x64 -lboost_date_time-mt-x64 -lboost_exception-mt-x64 -lboost_fiber-mt-x64 -lboost_filesystem-mt-x64 -lboost_graph-mt-x64 -lboost_json-mt-x64 -lboost_locale-mt-x64 -lboost_log-mt-x64 -lboost_log_setup-mt-x64 -lboost_math_c99-mt-x64 -lboost_math_c99f-mt-x64 -lboost_math_c99l-mt-x64 -lboost_math_tr1-mt-x64 -lboost_math_tr1f-mt-x64 -lboost_math_tr1l-mt-x64 -lboost_nowide-mt-x64 -lboost_prg_exec_monitor-mt-x64 -lboost_program_options-mt-x64 -lboost_random-mt-x64 -lboost_regex-mt-x64 -lboost_serialization-mt-x64 -lboost_stacktrace_addr2line-mt-x64 -lboost_stacktrace_backtrace-mt-x64 -lboost_stacktrace_basic-mt-x64 -lboost_stacktrace_noop-mt-x64 -lboost_system-mt-x64 -lboost_test_exec_monitor-mt-x64 -lboost_thread-mt-x64 -lboost_timer-mt-x64 -lboost_type_erasure-mt-x64 -lboost_unit_test_framework-mt-x64 -lboost_wave-mt-x64 -lboost_wserialization-mt-x64 
DEPENDENCIES=-lStreamCommon -lViconCGStream -lViconCGStreamClient -lViconCGStreamClientSDK -lViconDataStreamSDK_CPP -lViconDataStreamSDKCore -lViconDataStreamSDKCoreUtils 
endif

ENV_CPU=x64
BUILDDIRECTORY=../../../../../../../../Source/Build
SOURCEDIRECTORY=../../../..
PROJECTPATH=.
BINARYDIRECTORY=../../../..
INTERMEDIATEDIRECTORY=.
LIBRARYDIRECTORY=../../../../lib
OUTPUTDIRECTORY=../../../../bin

include $(BINARYDIRECTORY)/gcc.mk

HIDE_BOOST_SCRIPT=hide_boost_version_script
ifneq ($(HIDE_BOOST),)
    HIDE_BOOST_LD_PARAM= -Wl,--version-script=$(HIDE_BOOST_SCRIPT)
    HIDE_BOOST_LD_PREREQ=$(HIDE_BOOST_SCRIPT)
endif
all: all_$(CONFIG)

all_Debug: $(OUTPUTDIRECTORY)/$(CONFIG)/ViconDataStreamSDK_CPPTest
all_InternalRelease: $(OUTPUTDIRECTORY)/$(CONFIG)/ViconDataStreamSDK_CPPTest
all_Release: $(OUTPUTDIRECTORY)/$(CONFIG)/ViconDataStreamSDK_CPPTest

OBJECTS=$(CONFIG)/ViconDataStreamSDK_CPPTest.o

CXXFLAGS+=$(SYSTEMINCLUDEPATHS) $(INCLUDEPATHS) $(DEFINES)
CCFLAGS+=$(SYSTEMINCLUDEPATHS) $(INCLUDEPATHS) $(DEFINES)
LDFLAGS+=$(LIBRARYPATHS)
# Android toolchain does not include librt but integrates some of its functionality into Android libc.
ifndef ANDROID_TARGET_ARCH
LDFLAGS+=-lrt
endif


$(OUTPUTDIRECTORY)/Debug/ViconDataStreamSDK_CPPTest: makefile $(OBJECTS) $(LIBRARYDIRECTORY)/$(CONFIG)/libStreamCommon.a $(LIBRARYDIRECTORY)/$(CONFIG)/libViconCGStream.a $(LIBRARYDIRECTORY)/$(CONFIG)/libViconCGStreamClient.a $(LIBRARYDIRECTORY)/$(CONFIG)/libViconCGStreamClientSDK.a $(OUTPUTDIRECTORY)/$(CONFIG)/libViconDataStreamSDK_CPP.so $(LIBRARYDIRECTORY)/$(CONFIG)/libViconDataStreamSDKCore.a $(LIBRARYDIRECTORY)/$(CONFIG)/libViconDataStreamSDKCoreUtils.a
	@echo \[1\;32mLinking EXE $@\[0m
	@mkdir -p $(@D)
	$(LD) -Wl,--as-needed -export-dynamic $(LDFLAGS) -o $@ $(OBJECTS) -Wl,--start-group $(DEPENDENCIES) $(LIBRARIES) -Wl,--end-group -pthread -ldl -Wl,-rpath='$$ORIGIN:.' -Wl,-rpath-link=.:$(@D)

$(OUTPUTDIRECTORY)/InternalRelease/ViconDataStreamSDK_CPPTest: makefile $(OBJECTS) $(LIBRARYDIRECTORY)/$(CONFIG)/libStreamCommon.a $(LIBRARYDIRECTORY)/$(CONFIG)/libViconCGStream.a $(LIBRARYDIRECTORY)/$(CONFIG)/libViconCGStreamClient.a $(LIBRARYDIRECTORY)/$(CONFIG)/libViconCGStreamClientSDK.a $(OUTPUTDIRECTORY)/$(CONFIG)/libViconDataStreamSDK_CPP.so $(LIBRARYDIRECTORY)/$(CONFIG)/libViconDataStreamSDKCore.a $(LIBRARYDIRECTORY)/$(CONFIG)/libViconDataStreamSDKCoreUtils.a
	@echo \[1\;32mLinking EXE $@\[0m
	@mkdir -p $(@D)
	$(LD) -Wl,--as-needed -export-dynamic $(LDFLAGS) -o $@ $(OBJECTS) -Wl,--start-group $(DEPENDENCIES) $(LIBRARIES) -Wl,--end-group -pthread -ldl -Wl,-rpath='$$ORIGIN:.' -Wl,-rpath-link=.:$(@D)

$(OUTPUTDIRECTORY)/Release/ViconDataStreamSDK_CPPTest: makefile $(OBJECTS) $(LIBRARYDIRECTORY)/$(CONFIG)/libStreamCommon.a $(LIBRARYDIRECTORY)/$(CONFIG)/libViconCGStream.a $(LIBRARYDIRECTORY)/$(CONFIG)/libViconCGStreamClient.a $(LIBRARYDIRECTORY)/$(CONFIG)/libViconCGStreamClientSDK.a $(OUTPUTDIRECTORY)/$(CONFIG)/libViconDataStreamSDK_CPP.so $(LIBRARYDIRECTORY)/$(CONFIG)/libViconDataStreamSDKCore.a $(LIBRARYDIRECTORY)/$(CONFIG)/libViconDataStreamSDKCoreUtils.a
	@echo \[1\;32mLinking EXE $@\[0m
	@mkdir -p $(@D)
	$(LD) -Wl,--as-needed -export-dynamic $(LDFLAGS) -o $@ $(OBJECTS) -Wl,--start-group $(DEPENDENCIES) $(LIBRARIES) -Wl,--end-group -pthread -ldl -Wl,-rpath='$$ORIGIN:.' -Wl,-rpath-link=.:$(@D)

# Source Files
$(CONFIG)/ViconDataStreamSDK_CPPTest.o: makefile $(SOURCEDIRECTORY)/Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPPTest/ViconDataStreamSDK_CPPTest.cpp
	@echo \[1\;34mCompiling ViconDataStreamSDK_CPPTest.cpp\[0m
	@mkdir -p $(@D)
	find $(CONFIG) -name *.gch -exec cp '{}' . \;
	$(CXX) -fPIC -MMD -MP -I$(CONFIG)/ $(CXXFLAGS)  -o $@ -c $(SOURCEDIRECTORY)/Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPPTest/ViconDataStreamSDK_CPPTest.cpp

-include $(CONFIG)/ViconDataStreamSDK_CPPTest.d

# Other Files

clean:
	@echo \[1\;31mCleaning $(CONFIG) build\[0m
	find . -path '*/$(CONFIG)/*' \( -name '*.[od]' -o -name '*.gch' \) -exec rm -f {} ';' \n
	rm -f moc_*.cxx

$(HIDE_BOOST_SCRIPT): makefile
	echo -n >$@
	echo "{" >>$@
	echo "  local: *N5boost*; *NK5boost*;" >>$@
	echo "};" >>$@
