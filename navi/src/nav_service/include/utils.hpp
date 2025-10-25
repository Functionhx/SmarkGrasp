#pragma once

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

double GetDistance(const geometry_msgs::PoseStamped & pose_1,
                            const geometry_msgs::PoseStamped & pose_2){
  return hypot(pose_1.pose.position.x-pose_2.pose.position.x,
               pose_1.pose.position.y-pose_2.pose.position.y);
}

// -------------need back service--------------
struct Point2D{
    double x, y;
    Point2D(double x_, double y_){
        this->x = x_;
        this->y = y_;
    }
    Point2D(){this->x = 0; this->y = 0;}

    Point2D& operator =(const Point2D& obj){
        this->x = obj.x;
        this->y = obj.y;
        return *this;
    }

    Point2D& operator -(const Point2D& obj){
        this->x -= obj.x;
        this->y -= obj.y;
        return *this;
    }

    double norm(){
        return sqrt(this->x * this->x + this->y * this->y);
    }
};

struct Point3D{
    double x, y, z;
    Point3D(double x_, double y_, double z_){
        this->x = x_;
        this->y = y_;
        this->z = z_;
    }
    Point3D(){this->x = 0; this->y = 0; this->z = 0;}

    Point3D& operator = (const Point3D& obj){
        this->x = obj.x;
        this->y = obj.y;
        this->z = obj.z;
        return *this;
    }
};

double crossProduct(const Point2D& A, const Point2D& B, const Point2D& C) {
    return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
}

// -------------nav service--------------
namespace nav {

enum NavMode {
    PATROLLING = 0,
    CHASEING = 2,
    SINGLE_POINT = 1,
    STOP = -1
};

enum NavPoint {
    MY_HOME_BASE = 0,
    MY_HOME_CIR = 1,
    MY_HOME_BUFF = 2,

    MY_T_HIGH_LAND = 3,
    MY_OUTPOST = 4,
    MY_HOSPITAL = 5,

    ENEMY_OUTPOST = 6,
    ENEMY_BASE = 7,
    ENEMY_T_HIGH_LAND = 8
};

}