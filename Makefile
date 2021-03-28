include ../../makefile.conf
NAME=main
STARTUP_DEFS=-D__STARTUP_CLEAR_BSS -D__START=main

LDSCRIPTS=-L. -L$(BASE)/ldscripts -T gcc.ld #nokeep.ld
LFLAGS=$(USE_NANO) $(USE_NOHOST) $(LDSCRIPTS) $(MAP) $(GC) 

SUB=lpc8xx_clkconfig.o lpc8xx_gpio.o lpc8xx_uart.o system_LPC8xx.o my_uart.o my_string.o my_queue.o my_command.o #systick.o iap_driver.o 
DIR-SUB = $(addprefix ../../common/,$(SUB))

TEMP_SUB = my_sctimer.o

COMMON_INC=../../common/include

all:$(NAME)-$(CORE).bin

write: $(NAME)-$(CORE).bin
	lpc21isp -bin $^ /dev/ttyUSB0 115200 12000
$(NAME)-$(CORE).axf: $(NAME).o $(STARTUP) $(TEMP_SUB)
	$(CC) $^ -I. -I$(COMMON_INC) $(CFLAGS) $(LFLAGS) $(DIR-SUB) -o $@

%.o:%.c
	$(CC) -c $^ -I. -I$(COMMON_INC) $(CFLAGS) $(LFLAGS) -o $@


clean: 
	rm -f $(NAME)*.axf *.map *.o *.bin

%.bin:%.axf
	arm-none-eabi-objcopy -O binary $^ $@
