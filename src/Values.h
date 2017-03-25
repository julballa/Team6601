/*
 * Values.h
 *
 *  Created on: Mar 20, 2017
 *      Author: CurtisMason
 */

#ifndef SRC_VALUES_H_
#define SRC_VALUES_H_
#define LEFT 0
#define RIGHT 1
#define ENCODER_FIX 1/3


#define ABUTTON 1
#define BBUTTON 2
#define XBUTTON 3
#define YBUTTON 4
#define LBBUTTON 5
#define RBBUTTON 6
#define BACKBUTTON 7
#define STARTBUTTON 8

	const std::string autoNameDefault = "Default";
	const std::string autoNameLeftPeg = "Left Peg";
	const std::string autoNameRightPeg = "Right Peg";
	const std::string autoNameMiddlePeg = "Middle Peg";
	float SpeedConstant = 1.0;
	bool ButtonReleased[5];



#endif /* SRC_VALUES_H_ */
