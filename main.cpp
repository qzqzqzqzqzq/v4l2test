#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include <cmath>

#include "gpio.h"
#include "motor.h"
#include "servo.h"
#include "pid.h"

#include "timer.h"
#include "message.h"

#include "control.h"
#include "vision.h"

#include "globals.h"

#include <csignal>





// ------------------
//Ctrl+C信号处理函数
void signal_handler(int signum) {
    allfinishflag.store(true);
}
// ------------------
int main() {
    signal(SIGINT, signal_handler);    // 注册信号处理函数：捕获 Ctrl+C
    double kp, ki, kd;
    std::cout << "请输入 PID 参数 (Kp Ki Kd): ";
    std::cin >> kp >> ki >> kd;
    //初始化
    motor_init();
    servo_init(105, 90, 120);
    pid_init(kp, ki, kd, 15);
    //测试
    VisionTaskState = State::ToBlueCone;
    send_message("Start");
    //初始化帧率检测进程
    std::thread thread3(timedelayIT);
    std::thread t_vision(vision_loop);    //视觉处理线程
    std::thread t_control(control_loop_timer);                    //电机，舵机控制线程

    // --- 等待退出信号 ---
    while (!allfinishflag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    // --- 回收线程 ---
    std::cout << "[INFO] Joining threads..." << std::endl;
    t_vision.join();
    std::cout << "[INFO] Vision thread joined." << std::endl;
    t_control.join();
    std::cout << "[INFO] Control thread joined." << std::endl;
    thread3.join();
    std::cout << "[INFO] Timer thread joined." << std::endl;

    // ------------------

    std::cout << "[INFO] All threads joined. Releasing resources..." << std::endl;
    SaveResultsToCSV("lane_result.csv");
    std::cout << "[INFO] Program finished." << std::endl;
    return 0;
}