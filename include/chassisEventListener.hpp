#pragma once

#include "main.h" // IWYU pragma: keep

namespace c {

void moveToPoint(float x, float y, int timeout,
                 lemlib::MoveToPointParams params = {}, bool async = true);

void moveToPose(float x, float y, float theta, int timeout,
                lemlib::MoveToPoseParams params = {}, bool async = true);

void turnToHeading(float theta, int timeout,
                   lemlib::TurnToHeadingParams params = {}, bool async = true);

void turnToPoint(float x, float y, int timeout,
                 lemlib::TurnToPointParams params = {}, bool async = true);

void waitUntilDone();

} // namespace c