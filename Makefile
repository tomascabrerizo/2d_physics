main:
	g++ -Wall -Wextra ./main.cpp -o 2dphy -lmingw32 -lSDL2main -lSDL2

firework:
	g++ -Wall -Wextra ./firework.cpp -o 2dphy -lmingw32 -lSDL2main -lSDL2

run:
	.\2dphy.exe

