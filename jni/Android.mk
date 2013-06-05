LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := NavJni
LOCAL_C_INCLUDES := /usr/include/eigen3
LOCAL_SRC_FILES := NavJni.cpp

include $(BUILD_SHARED_LIBRARY)
