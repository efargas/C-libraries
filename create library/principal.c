#include <libreria1.h>
#include <stdio.h>

int main()
{
	int a=3;
	int b=5;

	int c;

	c=suma(a,b);
	printf ("%d + %d = %d\n", a, b, c);

	c=resta(a,b);
	printf ("%d - %d = %d\n", a, b, c);

	return 0;
}
