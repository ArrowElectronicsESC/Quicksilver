NAME :=App_dct_rgb

$(NAME)_SOURCES := dct_rgb.c \
                   ../ArrowConnect/Drivers/LED/APA102/apa102.c

$(NAME)_COMPONENTS :=  utilities/command_console
					   

APPLICATION_DCT := rgb_dct.c