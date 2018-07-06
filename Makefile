ASF?= xdk-asf-3.40.0
PRJ_PATH= ${ASF}

include ${ASF}/sam0/utils/make/Makefile.sam.in

.PHONY: debug-emacs
debug-emacs: all
	@$(GDB) -i=mi -x "$(PRJ_PATH)/$(DEBUG_SCRIPT_FLASH)" -ex "reset" -readnow -se $(TARGET_FLASH)
