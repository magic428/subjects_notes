#ifndef _CONST_ENUM_INLINE_HEADER_H_
#define _CONST_ENUM_INLINE_HEADER_H_

#include <string>
#include <iostream>

/**
 * const, enum, inline 替换 #define
 * (1) const 常量替换 #define 常量;
 * (2) class 专属常量, 为确保常量只有一份, 将其定义为 static 成员;
 * (3) enum 替换 #define 常量;
 * (4) inline 函数替换 #define 宏函数;
*/

// const 常量替换 #define 常量;
const double AspectRatio = 0.05;
const std::string authorName("Scott Meyers");

// class 专属常量, 为确保常量只有一份, 将其定义为 static 成员;
class GamePlayer{
public:
    GamePlayer() { std::cout << "GamePlayer Constructor" << std::endl; }

private:
    static const int NumTurns = 5;
    int scores[NumTurns]; 
};

// 对于不支持 static 成员在其声明式上获得初值的情形, 如上述 GamePlayer::NumTurns 的声明. 
class CostEstimate{
public:
    CostEstimate() {}

private:
    static const double NumFudgeFactor;  // 声明常量, 然后在实现文件中定义该倡常量
    // const double CostEstimate::NumFudgeFactor = 1.5;
};

// enum 替换 #define 常量;
class DisplayScreen{
public:
    DisplayScreen() { std::cout << "DisplayScreen Constructor" << std::endl; }

private:
    enum {
        NumScreens = 5,
    };

    int screen[NumScreens]; 
};


// inline 函数替换 #define 宏函数;
template <typename T>
void func(const T &)
{
    // do something here...
}

template <typename T>
inline void callWithMax(const T &lhs, const T &rhs)
{
    func(lhs > rhs ? lhs : rhs);
}

#endif