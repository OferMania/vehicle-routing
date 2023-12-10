#pragma once

struct Coordinate {
    long double pickupX;
    long double pickupY;
    long double dropOffX;
    long double dropOffY;

    Coordinate()
    : pickupX(0)
    , pickupY(0)
    , dropOffX(0)
    , dropOffY(0) {}
};
