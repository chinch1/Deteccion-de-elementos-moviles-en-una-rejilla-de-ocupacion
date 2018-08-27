#define sumar(a,b) a+b
#define restar(a,b) a-b
#define multiplicacion(a,b) a*b
#define divisionentera(a,b) a/b

#include <iostream>		// Libreria para trabajar con flujo de entrada y de salida en c++. Las comillas son para tomar la libreria de la carpeta del proyecto.
#include <stdio.h>	//  Libreria para trabajar con 
#include <stdlib.h> // System("clear")
// #include <curses.h> //

using namespace std; // Clase que contiene a cout y cin

float Suma(float a, float b);
float Resta(float a, float b);
int Multiplicacion(int a, int b);
bool Bandera(float a, float b);

const int NumeroFijo = 1;
const bool BanderaTrue = true;
const bool BanderaFalse = false;

typedef unsigned int uint; // Esto permite cambiar el nombre al tipo de dato "unsigned int"
typedef int entero; // Cambie "int" por "entero"

//--------------------------------------------------main-------------------------------------------------------------
entero main()
{	
	int i = 0;
	float Num1;
	float Num2;
	char c, b, a, s, r, m;
	char v[4] = {'a','b','c','d'};
	string CadenaDeCaracateres = "abcdefg";
	bool bandera;
	double NumeroEnorme;

/*//--------------------Trabajo con if-------------------------------
	if (Num1 > Num2)
	{
		cout << "Si restamos " << Num1 << " con " << Num2 << ", obtenemos: " << Resta(Num1, Num2) << "\n" << endl;
	}
	else if (Num1 == Num2)
	{
		cout << "Los valores son iguales, la operacion no esta permitida. Por que?, porque me da la gana" << endl;
	}
		cout << "Si sumamos " << Num1 << " con " << Num2 << ", obtenemos: " << Suma(Num1,Num2) << "\n" << endl;*/

/*//--------------------Trabajo con switch---------------------------
	do
	{	
		// system("pause");
		system("clear");

		cout << "Ingrese dos valores para operar: " << "\n" << endl;
		cin >> Num1;
		cin >> Num2;
		cout << "\n Sus valores seleccionados son: " << Num1 << " y " << Num2 << "\n" << endl;
		cout << " s para sumar \n r para restar \n m para multiplicar \n" << endl;
		cout << "Ingrese una opcion: " << "\n" << endl;
		cin >> c;

			switch (c)
			{
				case 's':
					int ResultadoSuma;
					ResultadoSuma = Suma(Num1,Num2);
					cout << "El resultado de la suma es: " << ResultadoSuma << "\n" << endl;
					break;

				case 'r':
					int ResultadoResta;
					ResultadoResta = Resta(Num1,Num2);
					cout << "El resultado de la resta es: " << ResultadoResta << "\n" << endl;
					break;

				case 'm':
					int ResultadoMultiplicacion;
					ResultadoMultiplicacion = Multiplicacion(Num1,Num2);
					cout << "El resultado de la multiplicacion es: " << ResultadoMultiplicacion << "\n" << endl;
					break;

				default:
					cout << "La opcion introducida no es valida" << "\n" << endl;
			} 

		cout << "Desea continuar con el programa? Y/n: " << "\n" << endl;
		cin >> a;
		if (a == 'Y')
		{
			b = 'a';
		}
		else 
			b = 'b';
			cout << "Programa Terminado. Hasta Luego!" << endl;

	} while (!(b == 'b'));*/

/*//--------------------Trabajo con vectores-------------------------
	for(int j = 0; j < 4; j++)
	{
		cout << v[j]; 
	}
	cout << endl;*/

/*//--------------------Trabijo con #define--------------------------
	
	cout << "Ingrese dos valores para operar: " << "\n" << endl;
	cin >> Num1;
	cin >> Num2;

	int suma = sumar(Num1,Num2);
	int resta = restar(Num1,Num2);
	int multiplicacion = multiplicacion(Num1,Num2);
	int division = divisionentera(Num1,Num2);

	cout << "Los resultados son los siguientes: " << "\n" << "Suma: " << suma << "\n" << "Resta: " << resta << "\n" << "Multiplicacion: " << multiplicacion << "\n" << "Division: " << division << "\n" << "Salu2!" << endl;*/  

/*//--------------------Trabajo con estructura-----------------------
	struct Personaje
	{
		int edad;
		char sexo;
		string orientacionSexual;

		Personaje() {edad = 0; sexo = 'f'; orientacionSexual = "Muy Macho";}// Funcion constructora dentro de la estructura
		int getEdad(){return edad;}
		char getSexo(){return sexo;}
		string getOrientacionSexual(){return orientacionSexual;}
		int putEdad(int a){edad = a;}
		char putSexo(char m){sexo = m;}
		string putOrientacionSexual(string OS){orientacionSexual = OS;}

	}Broock, Curtis, Manuel, *puntero;
	
	// Broock.edad = cin.get();
	// Broock.sexo = cin.get();
	// Broock.orientacionSexual = cin.get();

	Broock.edad = 15;
	puntero = &Broock;

	cout << puntero << endl;
	cout << &Broock.edad << endl;
	cout << puntero->edad << endl; // Forma de apuntar a una variable dentro de la estructura


	cout << "Ingrese el la edad, el sexo y la orintacion sexual de Broock: " << "\n" << endl;

	int joder1;
	cin >> joder1;
	Broock.putEdad(joder1);

	char joder2;
	cin >> joder2;
	Broock.putSexo(joder2);

	cout << "\n" << "La edad de Broock es: " << Broock.edad << "\n" << "El sexo de Broock es: " << Broock.sexo << "\n" << "La orientacion sexual de Broock es : " << Broock.orientacionSexual << endl;
	system ("pause");*/

/*//--------------------Trabajo con punteros basico -----------------
	int array[5];
	int *p;
	
	p = array; // 
	cout << "direccion array[0] " << &array << endl;
	cout << "puntero " << p << endl;
	system ("pause");*/

}

// -----------------------------------------------------Funciones -----------------------------------------------------

float Suma(float a, float b)
{
	a = a + b;
	return a;
}

float Resta(float a, float b)
{
	a = a - b;
	return a;
}

int Multiplicacion(int a, int b)
{
	a = a*b;
	return a;
}

bool Bandera(float a, float b)
{
	bool bandera;
	if(Resta(a,b) > 0)
	{
		bandera = true;
		return bandera;
	}
}