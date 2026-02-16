.PHONY: all clean stm32 stm32-clean esp32 esp32-clean esp32-fullclean

all: stm32

clean: stm32-clean

stm32:
	$(MAKE) -C stm32 all

stm32-clean:
	$(MAKE) -C stm32 clean

esp32:
	cd esp32-c3 && idf.py build

esp32-clean:
	cd esp32-c3 && idf.py clean

esp32-fullclean:
	cd esp32-c3 && idf.py fullclean
