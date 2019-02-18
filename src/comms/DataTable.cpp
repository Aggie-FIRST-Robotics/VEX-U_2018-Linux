
#include "comms/DataTable.h"

DataTable::DataTable() {
	//mtx.lock();
	for(int i = 0; i < TABLE_SIZE; i++) {
		variables[i] = 0;
		age[i] = 0;
	}
	//mtx.unlock();
}

DataTable::~DataTable () {}

bool DataTable::update(Frame a) {
	//mtx.lock();
	if( a.var_address >= TABLE_SIZE || a.var_address < 0 ){
		//mtx.unlock();
		return false;
	}
	
	variables 	[a.var_address] = a.data;
	age 		[a.var_address] = 0;
	//mtx.unlock();
	return true;
}

void DataTable::age_by(unsigned long period) {
	//mtx.lock();
	for(uint8_t i = 0; i < TABLE_SIZE; i++)
		age [i] += period;
	//mtx.unlock();
}

short DataTable::read (uint8_t addr) {
	return variables[addr];
}

unsigned long DataTable::card (uint8_t addr) {
	return age[addr];
}