/*
 * vector.cpp
 *
 * Created: 2012-12-16 02:29:11
 *  Author: x0163527
 */ 


class Vector {
	private:
	int16_t data[3];
	
	public:
	Vector(int16_t value) {
		for (int i = 0; i < 3; i++) {
			data[i] = value;
		}
	}
	Vector(Vector value) {
		for (int i = 0; i < 3; i++) {
			data[i] = value[i];
		}
	}	
	int16_t operator[](int i) {
		return data[i]
	}
	Vector operator+=(Vector value) {
		for (int i = 0; i < 3; i++) {
			data[i] += value.data[i];
		}
		return this;
	}
};

