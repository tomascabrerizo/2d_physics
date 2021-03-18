
INCLUDE_PATH = "D:\thirdparty\SDL2-2.0.14\include"
LIB_PATH = "D:\thirdparty\SDL2-2.0.14\lib\x64"

main:
	clang -Wall -Wextra ./main.cpp -g -o 2dphy.exe -I$(INCLUDE_PATH) -L$(LIB_PATH) -Xlinker /subsystem:console -lshell32 -lSDL2main -lSDL2

firework:
	clang -Wall -Wextra ./firework.cpp -g -o 2dphy.exe -I$(INCLUDE_PATH) -L$(LIB_PATH) -Xlinker /subsystem:console -lshell32 -lSDL2main -lSDL2

run:
	.\2dphy.exe

