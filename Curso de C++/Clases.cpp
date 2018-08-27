#include <iostream>		// Libreria para trabajar con flujo de entrada y de salida en c++. Las comillas son para tomar la libreria de la carpeta del proyecto.
#include <stdio.h>	//  Libreria para trabajar con 
#include <stdlib.h> // System("clear")

using namespace std;

class Cuentas
{
	public: //Se pueden cambiar estos datos, ya sean variables o funciones. Pueden ser utilizados por los mismos elementos dentro de la clase.	
			int sumaDos(int arg);
			int sumaTres(int arg);
	private: //Estas propiedades no pueden ser cambiadas 
			int resultado;
};

int Cuentas::sumaDos(int arg)
{
	resultado = arg + 2;
	return resultado;
}

int Cuentas::sumaTres(int arg)
{
	resultado = arg + 3;
	return resultado;
}

int main()
{
	Cuentas cuenta1;
	cout << "+ 2 = " << cuenta1.sumaDos(0) << endl;
	cout << "+ 3 = " << cuenta1.sumaTres(0) << endl;
	system ("pause");
	return 0;
}