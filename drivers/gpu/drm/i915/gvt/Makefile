GVT_SOURCE := gvt.o params.o aperture_gm.o mmio.o handlers.o fb_decoder.o

ccflags-y			+= -I$(src) -I$(src)/.. -Wall -Werror -Wno-unused-function
i915_gvt-y			:= $(GVT_SOURCE)
obj-$(CONFIG_I915_GVT)		+= i915_gvt.o
