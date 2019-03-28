
#include "comms/DataTable.h"

DataTable::DataTable() {
	for(int i = 0; i < TABLE_SIZE; i++) {
		variables[i] = 0;
		age[i] = 0;
	}
}

DataTable::~DataTable () {}

bool DataTable::update(Frame a) {
	if( a.var_address >= TABLE_SIZE || a.var_address < 0 ){
		return false;
	}

	// std::cout << "****************" << std::endl;

 //  	std::cout << (int)a.source << std::endl;
 //  	std::cout << (int)a.var_address << std::endl;
 //  	std::cout << a.data << std::endl;

 //  	std::cout << "****************" << std::endl;
	variables 	[a.var_address] = a.data;
	age 		[a.var_address] = millis();
	return true;
}

short DataTable::read (uint8_t addr) {
	return variables[addr];
}

unsigned long DataTable::card (uint8_t addr) {
	return age[addr];
}