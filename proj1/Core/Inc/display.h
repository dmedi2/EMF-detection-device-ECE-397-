/*
 * display.h
 *
 *  Created on: Mar 2, 2025
 *      Author: citylion
 */

#ifndef SRC_DISPLAY_H_
#define SRC_DISPLAY_H_



#endif /* SRC_DISPLAY_H_ */

void imageTestLoop();

void display_setup();

void display_test();

void display_clear();

void display_image();

void display_update();

void buffer_clear();

void set_rawpixel(int pixelNum, _Bool value);

//sets 12 pixels
void twelve_write(int x, int y, int dat);

