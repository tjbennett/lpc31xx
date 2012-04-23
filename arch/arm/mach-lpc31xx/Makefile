#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y += irq.o evtr.o time.o generic.o dma.o usb.o clock.o sram.o
obj-$(CONFIG_OF) += dt.o


# Specific board support
obj-$(CONFIG_MACH_EA313X) += board-ea313x.o
obj-$(CONFIG_MACH_EA3152) += board-ea313x.o
obj-$(CONFIG_LPC3152_AD) += psu.o

# Power Management
obj-$(CONFIG_PM)		+= pm.o pm_standby.o

ifeq ($(CONFIG_PM_DEBUG),y)
CFLAGS_pm.o += -DDEBUG
endif
