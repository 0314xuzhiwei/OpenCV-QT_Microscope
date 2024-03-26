#ifndef MYDEFINE_H
#define MYDEFINE_H


#define GLUE 5

#define ZERO_X 8
#define ZERO_Y 9
#define ZERO_Z 10

#define AXISFORWARD 3   //正转回到原点后会自动设置DPOS值为0
#define AXISREVERSE 4   //反转回到原点后会自动设置DPOS值为0

#define RUNCONTINUE 0   //持续运动
#define RUNMOTION   1   //寸动

#define RUNFRONT  true  //正转
#define RUNREVERSAL false   //反转

#define WAITETIME 6000

#define AXISXMAX 2850
#define AXISYMAX 2830
#define AXISZMAX 250

enum AXIS
{
    AXIS_X,
    AXIS_Y,
    AXIS_Z
};

typedef struct
{
    float x;
    float y;
    float z;
}AXIS_P;

typedef struct
{
    int type;
    AXIS_P* p0;
}GLUEPOINT;


#endif // MYDEFINE_H



