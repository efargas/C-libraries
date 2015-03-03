#
# Javier Abell�n. 18 de Abril de 2003
#
# Makefile para la demostraci�n de librer�as est�ticas y din�micas.
CFLAGS=-Wall -I.

# Por defecto se hacen p1 y p2. Son el mismo ejecutable, pero p1 usa librer�a
# est�tica y p2 librer�a din�mica.
all: p1 p2

# Se construye p1, haciendo tambi�n la librer�a est�tica.
p1: principal.o liblibreria1.a
	cc -o p1 $(CFLAGS) principal.o -L. -llibreria1

# Se construye p2, haciendo tambi�n la librer�a din�mica.
p2: principal.o liblibreria1.so
	cc -o p2 $(CFLAGS) principal.o -L. -Bdynamic -llibreria1

# Dependencias de la librer�a est�tica. La construcci�n se hace con la
# regla implicita de make.
liblibreria1.a: liblibreria1.a(suma.o resta.o)

# Construci�n de la librer�a din�mica.
liblibreria1.so: suma.c resta.c
	cc $(CFLAGS) -c -o suma.o suma.c
	cc $(CFLAGS) -c -o resta.o resta.c
	ld -o liblibreria1.so suma.o resta.o -shared
	rm suma.o resta.o

# Objetivo extra para borrado de librer�as y ejecutables.
clean:
	rm p1 p2 liblibreria1.a liblibreria1.so
