GVT_SOURCE := gvt.o params.o aperture_gm.o mmio.o handlers.o instance.o \
		trace_points.o interrupt.o gtt.o cfg_space.o opregion.o utility.o \
		fb_decoder.o display.o edid.o control.o execlist.o

ccflags-y			+= -I$(src) -I$(src)/.. -Wall -Werror -Wno-unused-function
i915_gvt-y			:= $(GVT_SOURCE)
obj-$(CONFIG_I915_GVT)		+= i915_gvt.o
