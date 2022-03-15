#pragma once

namespace iros {

    class StampedState
    {
    private:
        double stamp_;
        float x_, y_, z_;
        double roll_, pitch_, yaw_;
        float qx_, qy_,qz_,qw_;
        float vx_,vy_,vz_,wx_,wy_,wz_;

    public:
        StampedState(void):
            stamp_(0.0), x_(0.0f), y_(0.0f), z_(0.0f),
            qx_(0.0f), qy_(0.0f), qz_(0.0f), qw_(1.0f),
            vx_(0.0f), vy_(0.0f), vz_(0.0f), wx_(0.0f), wy_(0.0f), wz_(0.0f) {}
        //~StampedState();

        //set parameters
        inline void setStamp(double stamp) { stamp_ = stamp; }
        inline void setX(float x) { x_ = x; }
        inline void setY(float y) { y_ = y; }
        inline void setZ(float z) { z_ = z; }
        inline void setRoll(double roll) { roll_ = roll; }
        inline void setPitch(double  pitch) { pitch_ = pitch; }
        inline void setYaw(double yaw) { yaw_ = yaw; }
        inline void setQx(float qx) { qx_ = qx; }
        inline void setQy(float qy) { qy_ = qy; }
        inline void setQz(float qz) { qz_ = qz; }
        inline void setQw(float qw) { qw_ = qw; }
        inline void setPose(float x, float y, float z,double roll, double pitch, double yaw)
        { x_ = x, y_ = y, z_ = z, roll_ = roll, pitch_ = pitch, yaw_ = yaw;}

        inline void setPoseforHuman(float x,float y,float z)
        { x_ = x, y_ = y, z_ = z;}
        inline void setQuaternion(float qx, float qy, float qz, float qw)
        { qx_ = qx, qy_ = qy, qz_ = qz, qw_ = qw; }
        inline void setVx(float vx) { vx_ = vx; }
        inline void setVy(float vy) { vy_ = vy; }
        inline void setVz(float vz) { vz_ = vz; }
        inline void setWx(float wx) { wx_ = wx; }
        inline void setWy(float wy) { wy_ = wy; }
        inline void setWz(float wz) { wz_ = wz; }
        inline void setVelocities(float vx, float vy, float vz,
                                float wx, float wy, float wz)
        { vx_ = vx, vy_ = vy, vz_ = vz, wx_ = wx, wy_ = wy, wz_ = wz; }

        //get the value of all parameters
        inline double getStamp(void) { return stamp_; }
        inline float getX(void) { return x_; }
        inline float getY(void) { return y_; }
        inline float getZ(void) { return z_; }
        inline float getRoll(void) { return roll_; }
        inline float getPitch(void) { return pitch_; }
        inline float getYaw(void) { return yaw_; }
        inline float getQx(void) { return qx_; }
        inline float getQy(void) { return qy_; }
        inline float getQz(void) { return qz_; }
        inline float getQw(void) { return qw_; }
        inline float getVx(void) { return vx_; }
        inline float getVy(void) { return vy_; }
        inline float getVz(void) { return vz_; }
        inline float getWx(void) { return wx_; }
        inline float getWy(void) { return wy_; }
        inline float getWz(void) { return wz_; }

    }; //class StampedPose
    
    // StampedState::StampedState(/* args */)
    // {
    // }
    
    // StampedState::~StampedState()
    // {
    // }
    
} //namespace iros