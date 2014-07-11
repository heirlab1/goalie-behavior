//============================================================================
// Name        : GoalieBehavior.cpp
// Author      : Kellen Carey
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "Vision.h"
#include "MotorController.h"

#define SIDE_STEP_OFFSET				2
#define SIDE_STEP_ODOMETRY_DISTANCE		5

double getUnixTime() {
	struct timespec tv;

	if (clock_gettime(CLOCK_REALTIME, &tv) != 0) {
		return 0;
	}

	return (((double)tv.tv_sec) + (tv.tv_nsec / 1000000000.0));
}

void doMotion(Vision &vis, MotorController &motorController) {
	bool done = false;
	double startTime = getUnixTime();
	double currentTime = getUnixTime();
	double stepTime;
	while (!done) {
		int step_result = motorController.step(false);
		switch (step_result) {
		case MUL8_STEP_NULL:
//			vis.grabFrame();
			break;
		case MUL8_STEP_FINISHED:
			// Loop through vision until it's time to execute the next motion
			startTime = getUnixTime();
			currentTime = getUnixTime();
			stepTime = motorController.getStepTime();
//			std::cout << "Step Time: " << stepTime << std::endl;
			stepTime -= 0.15;	// Subtract 150 mS to deal with frame processing time
			do {
				vis.nextFrame();
//				std::cout << "Step time: " << stepTime << "\t";
//				std::cout << "Difference between clocks: " << currentTime-startTime << std::endl;
				currentTime = getUnixTime();
				//				std::cout << "Ellapsed time: " << (((float)currentTime-startTime)) << std::endl;
			} while ((currentTime-startTime) < stepTime);
			break;
		case MUL8_MOTION_FINISHED:
			done = true;
			break;
		}
	}
}

int main() {
	MotorController motorController;
	motorController.init();
	Vision vis;
	vis.init(motorController);

	vis.updateRobotPosition(10, 0);
	vis.setAction(CENTER_BALL);
	bool ball_position_known = false;

	std::cout << "Starting loop" << std::endl;
	while (1) {
		vis.nextFrame();
		Point ball;
		if (ball_position_known || vis.knowsBallPosition()) {
			ball_position_known = true;

			ball.x = vis.getBallX();
			ball.y = vis.getBallY();
			std::cout << "Got ball X" << std::endl;
			double goal_position = vis.getRobotY() * (ball.x/ball.y);
			double current_position = vis.getRobotX();
			if (goal_position < (current_position - SIDE_STEP_OFFSET)) {
				motorController.setMotion("SSl");
				std::cout << "Doing motion SSL" << std::endl;
				doMotion(vis, motorController);
				vis.updateRobotPosition(SIDE_STEP_ODOMETRY_DISTANCE, 90);
			}
			else if (goal_position > (current_position + SIDE_STEP_OFFSET)) {
				motorController.setMotion("SSr");
				std::cout << "Doing motion SSR" << std::endl;
				doMotion(vis, motorController);
				vis.updateRobotPosition(SIDE_STEP_ODOMETRY_DISTANCE, -90);
			}
		}
	}

	return 0;
}
