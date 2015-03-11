#Set up directories

OBJ_DIR = obj
SRC_DIR = src
INC_DIR = inc
BIN_DIR = bin

# Define compilation flags  

CC = g++

Serial : $(OBJ_DIR)/main.o $(OBJ_DIR)/serial.o $(OBJ_DIR)/timeout.o  
	$(CC) $(OBJ_DIR)/main.o $(OBJ_DIR)/serial.o $(OBJ_DIR)/timeout.o -o Serial  

$(OBJ_DIR)/timeout.o : $(SRC_DIR)/timeout.cpp $(INC_DIR)/timeout.h 
	$(CC) -g -c $(SRC_DIR)/timeout.cpp -o $(OBJ_DIR)/timeout.o  

$(OBJ_DIR)/serial.o : $(SRC_DIR)/serial.cpp $(INC_DIR)/serial.h $(OBJ_DIR)/timeout.o
	$(CC) -g -c $(SRC_DIR)/serial.cpp -o $(OBJ_DIR)/serial.o  

$(OBJ_DIR)/main.o : $(SRC_DIR)/main.cpp $(OBJ_DIR)/serial.o
	$(CC) -g -c $(SRC_DIR)/main.cpp -o $(OBJ_DIR)/main.o
