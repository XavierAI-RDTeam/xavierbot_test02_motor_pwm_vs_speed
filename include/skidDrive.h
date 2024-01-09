#include <cmath>

class SkidDrive
{
private:
    float wheelTrackFactor;
    float wheelBaseFactor;

public:
    SkidDrive(float wheelTrack, float wheelBase)
    { 
        wheelTrackFactor = wheelTrack/2;
        wheelBaseFactor  = wheelBase/2;
    }

    // void inverseKinematics(float translational, float rotational, float &rightFront, float &rightRear, float &leftFront, float &leftRear)
    void inverseKinematics(float translational, float rotational, float &rightFront, float &leftFront)
    {
        rightFront = translational + ((rotational/wheelTrackFactor)*(pow(wheelTrackFactor, 2) + pow(wheelBaseFactor, 2)));
        leftFront  = translational - ((rotational/wheelTrackFactor)*(pow(wheelTrackFactor, 2) + pow(wheelBaseFactor, 2)));
        // rightRear  = translational + ((rotational/wheelTrackFactor)*(pow(wheelTrackFactor, 2) + pow(wheelBaseFactor, 2)));
        // leftRear   = translational - ((rotational/wheelTrackFactor)*(pow(wheelTrackFactor, 2) + pow(wheelBaseFactor, 2)));
    }

    // void forwardKinematics(float rightFront, float rightRear, float leftFront, float leftRear, float &translational, float &rotational)
    void forwardKinematics(float rightFront, float leftFront, float &translational, float &rotational)
    {
        translational = (rightFront + leftFront) / 2;
        rotational    = (wheelTrackFactor * (rightFront - leftFront)) / (2 * (pow(wheelTrackFactor, 2) + pow(wheelBaseFactor, 2)));
    }
};