#
# Javier Abellán. 18 de Abril de 2003
#
# Makefile para la demostración de librerías estáticas y dinámicas.
CFLAGS=-Wall -I.

# Por defecto se hacen p1 y p2. Son el mismo ejecutable, pero p1 usa librería
# estática y p2 librería dinámica.
all: p1 p2

# Se construye p1, haciendo también la librería estática.
p1: principal.o liblibreria1.a
	cc -o p1 $(CFLAGS) principal.o -L. -llibreria1

# Se construye p2, haciendo también la librería dinámica.
p2: principal.o liblibreria1.so
	cc -o p2 $(CFLAGS) principal.o -L. -Bdynamic -llibreria1

# Dependencias de la librería estática. La construcción se hace con la
# regla implicita de make.
liblibreria1.a: liblibreria1.a(suma.o resta.o)

# Construción de la librería dinámica.
liblibreria1.so: suma.c resta.c
	cc $(CFLAGS) -c -o suma.o suma.c
	cc $(CFLAGS) -c -o resta.o resta.c
	ld -o liblibreria1.so suma.o resta.o -shared
	rm suma.o resta.o

# Objetivo extra para borrado de librerías y ejecutables.
clean:
	rm p1 p2 liblibreria1.a liblibreria1.so
