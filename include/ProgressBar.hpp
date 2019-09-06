/**
 * @file ProgressBar.hpp
 * @author Stephan Xu
 * @brief 控制台下进度条有关设置
 * @version 0.1
 * @date 2019-09-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef PROGRESS_BAR_HPP
#define PROGRESS_BAR_HPP

#include <string>
#include <vector>
#include <sstream>
#include <functional>
#include <thread>
#include <chrono>
#include <iostream>

/**
 * @brief 进度条实现
 * 简化后的进度条实现，采用构造函数中的 maxCount 来控制计数范围。使用 update 函数来增加数值
 */
class ProgressBar
{
public:
	explicit ProgressBar();

	/**
	 * @brief 建立一个 ProcessBar 对象
	 * 
	 * @param maxCount 最大计数
	 */
	explicit ProgressBar(int maxCount);

	explicit ProgressBar(int maxCount, std::string message);

	/**
	 * @brief 增加计数并刷新显示
	 * 
	 * @param delta 增量
	 */
	void update(int delta);

	std::string Message();
	void SetMessage(std::string message);

	int MaxCount();
	void SetMaxCount(int maxCount);

private:
	std::string m_Msg{"Processing"};
	int m_MaxCount{0};
	int m_Count{0};

	/* generate a bar string */
	std::string GenerateBar(double value);

private:
	const int c_BarWidth{32};
	const std::string c_BarPrefix{" |"};
	const std::string c_BarSuffix{"| "};
	const std::string c_Padding{" "};
	const std::vector<std::string> c_Fill{" ", "▏", "▎", "▍", "▌", "▋", "▊", "▉", "█"};
};

#endif
