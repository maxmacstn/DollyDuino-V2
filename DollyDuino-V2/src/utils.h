#ifndef UTILS_H
#define UTILS_H

double radsToDeg(double rads) {
	return rads * 180 / PI;
}

double degToRads(double deg) {
	return deg * PI / 180;
}

void printi(String text) {
	Serial.print(text);
}

void printi(String text, char c, String endText) {
	Serial.print(text);
	Serial.print(c);
	Serial.print(endText);
}

void printi(short num, String endText) {
	Serial.print(num);
	Serial.print(endText);
}

void printi(unsigned short num, String endText) {
	Serial.print(num);
	Serial.print(endText);
}

void printi(int num, String endText) {
	Serial.print(num);
	Serial.print(endText);
}

void printi(unsigned int num, String endText) {
	Serial.print(num);
	Serial.print(endText);
}

void printi(long num, String endText) {
	Serial.print(num);
	Serial.print(endText);
}


void printi(unsigned long num, String endText) {
	Serial.print(num);
	Serial.print(endText);
}

void printi(float num, int dp, String endText) {
	Serial.print(num, dp);
	Serial.print(endText);
}

void printi(double num, int dp, String endText) {
	Serial.print(num, dp);
	Serial.print(endText);
}

void printi(String text, short num, String endText) {
	Serial.print(text);
	Serial.print(num);
	Serial.print(endText);
}

void printi(String text, unsigned short num, String endText) {
	Serial.print(text);
	Serial.print(num);
	Serial.print(endText);
}

void printi(String text, int num, String endText) {
	Serial.print(text);
	Serial.print(num);
	Serial.print(endText);
}

void printi(String text, unsigned int num, String endText) {
	Serial.print(text);
	Serial.print(num);
	Serial.print(endText);
}

void printi(String text, long num, String endText) {
	Serial.print(text);
	Serial.print(num);
	Serial.print(endText);
}

void printi(String text, unsigned long num, String endText) {
	Serial.print(text);
	Serial.print(num);
	Serial.print(endText);
}

void printi(String text, float num, int dp, String endText) {
	Serial.print(text);
	Serial.print(num, dp);
	Serial.print(endText);
}

void printi(String text, double num, int dp, String endText) {
	Serial.print(text);
	Serial.print(num, dp);
	Serial.print(endText);
}


#endif