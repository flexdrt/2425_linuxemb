# Objet principal
obj-m := hello.o

# Chemin vers les sources du noyau 
KERNEL_SOURCE := /usr/lib/arm-linux-gnueabihf-gcc


# Options pour la compilation des modules
CFLAGS_MODULE := -fno-pic

# Règles de compilation
all :
	make -C $(KERNEL_SOURCE) M=$(PWD) modules
clean :
	make -C $(KERNEL_SOURCE) M=$(PWD) clean
install :
	make −C $(KERNEL_SOURCE) M=$(PWD) modules install



